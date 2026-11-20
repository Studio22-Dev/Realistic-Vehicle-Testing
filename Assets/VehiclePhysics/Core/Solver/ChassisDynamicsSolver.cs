using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public struct FixedStepTireCache
    {
        // Supports up to 16 wheels per single rigid vehicle body in registers with zero heap/unsafe usage.
        private TireStepResult _r0, _r1, _r2, _r3, _r4, _r5, _r6, _r7;
        private TireStepResult _r8, _r9, _r10, _r11, _r12, _r13, _r14, _r15;

        public TireStepResult Get(int i) => i switch
        {
            0 => _r0, 1 => _r1, 2 => _r2, 3 => _r3,
            4 => _r4, 5 => _r5, 6 => _r6, 7 => _r7,
            8 => _r8, 9 => _r9, 10 => _r10, 11 => _r11,
            12 => _r12, 13 => _r13, 14 => _r14, _ => _r15
        };

        public void Set(int i, in TireStepResult val)
        {
            switch (i)
            {
                case 0: _r0 = val; break; case 1: _r1 = val; break;
                case 2: _r2 = val; break; case 3: _r3 = val; break;
                case 4: _r4 = val; break; case 5: _r5 = val; break;
                case 6: _r6 = val; break; case 7: _r7 = val; break;
                case 8: _r8 = val; break; case 9: _r9 = val; break;
                case 10: _r10 = val; break; case 11: _r11 = val; break;
                case 12: _r12 = val; break; case 13: _r13 = val; break;
                case 14: _r14 = val; break; default: _r15 = val; break;
            }
        }
    }

    public static class ChassisDynamicsSolver
    {
        public static void Integrate6Dof(
            ref VehicleState veh,
            in VehicleConfig vCfg,
            NativeArray<ContactManifold> contactManifolds,
            NativeArray<WheelState> wheels,
            in FixedStepTireCache tireCache,
            bool applyExternalImpulses,
            float dtSub)
        {
            if (applyExternalImpulses && (veh.TierFlags & VehicleTierFlags.ExternalImpulsePending) != 0)
            {
                veh.LinearVelocity += veh.ExternalImpulse * veh.InverseMass;

                float3 extAngLocal = math.rotate(math.conjugate(veh.Rotation), veh.ExternalAngularImpulse);
                float3 extDeltaOmega = math.rotate(veh.Rotation, veh.InverseInertiaLocal * extAngLocal);
                veh.AngularVelocity += extDeltaOmega;

                veh.ExternalImpulse = float3.zero;
                veh.ExternalAngularImpulse = float3.zero;
                veh.TierFlags = (byte)(veh.TierFlags & ~VehicleTierFlags.ExternalImpulsePending);
            }

            float mass = 1.0f / math.max(veh.InverseMass, 1e-6f);
            float3 totalForce = new float3(0f, -vCfg.GravityMs2 * mass, 0f);
            float3 totalTorque = float3.zero;

            // Quadratic Aerodynamic Drag & Front/Rear Downforce
            float speedSq = math.lengthsq(veh.LinearVelocity);
            if (speedSq > 1e-4f)
            {
                float speed = math.sqrt(speedSq);
                float qDyn = 0.5f * vCfg.AirDensityKgM3 * speedSq;
                float3 dragForce = -veh.LinearVelocity * (qDyn * vCfg.AeroDragAreaCdA / speed);

                float3 aeroCoPRel = vCfg.AeroCenterOfPressureLocal - vCfg.CenterOfMassLocalOffset;
                float3 aeroWorldPos = veh.Position + math.rotate(veh.Rotation, aeroCoPRel);

                totalForce += dragForce;
                totalTorque += math.cross(aeroWorldPos - veh.Position, dragForce);

                float3 worldDown = -math.rotate(veh.Rotation, new float3(0f, 1f, 0f));
                float3 worldForward = math.rotate(veh.Rotation, new float3(0f, 0f, 1f));
                float halfWb = 0.5f * vCfg.WheelbaseM;

                float fDf = qDyn * vCfg.AeroFrontDownforceClA;
                float rDf = qDyn * vCfg.AeroRearDownforceClA;
                totalForce += worldDown * (fDf + rDf);
                totalTorque += math.cross(worldForward * halfWb, worldDown * fDf);
                totalTorque += math.cross(-worldForward * halfWb, worldDown * rDf);
            }

            // Wheel Suspension & Tire Forces
            int start = veh.WheelStartIndex;
            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];
                if ((ws.WheelFlags & WheelFlagBits.IsGrounded) == 0)
                {
                    ws.LastWorldForce = float3.zero;
                    wheels[slot] = ws;
                    continue;
                }

                ContactManifold manifold = contactManifolds[slot];
                TireStepResult tr = tireCache.Get(w);

                float3 wheelForce = ws.GroundLoadN * manifold.BlendedNormal
                                  + tr.Fx * tr.LongDir
                                  + tr.Fy * tr.LatDir;

                ws.LastWorldForce = wheelForce;
                wheels[slot] = ws;

                totalForce += wheelForce;
                totalTorque += math.cross(tr.WorldApplicationPoint - veh.Position, wheelForce);
            }

            // Symplectic Euler Linear + Body-Frame Gyroscopic Angular Integration
            float3 linearAccel = totalForce * veh.InverseMass;
            float3 newVel = veh.LinearVelocity + linearAccel * dtSub;

            float3 invI = veh.InverseInertiaLocal;
            float3 inertia = 1.0f / math.max(invI, new float3(1e-5f));
            quaternion invRot = math.conjugate(veh.Rotation);

            float3 omegaLocal = math.rotate(invRot, veh.AngularVelocity);
            float3 torqueLocal = math.rotate(invRot, totalTorque);
            float3 gyroTorque = math.cross(omegaLocal, inertia * omegaLocal);
            float3 newOmegaLocal = omegaLocal + invI * (torqueLocal - gyroTorque) * dtSub;
            float3 newOmega = math.rotate(veh.Rotation, newOmegaLocal);

            veh.LinearVelocity = newVel;
            veh.AngularVelocity = newOmega;
            veh.Position += newVel * dtSub;

            float omegaMag = math.length(newOmega);
            if (omegaMag > 1e-6f)
            {
                float halfAngle = 0.5f * omegaMag * dtSub;
                float3 axis = newOmega / omegaMag;
                quaternion dq = new quaternion(new float4(axis * math.sin(halfAngle), math.cos(halfAngle)));
                veh.Rotation = math.normalize(math.mul(dq, veh.Rotation));
            }

            float3 chassisForward = math.rotate(veh.Rotation, new float3(0f, 0f, 1f));
            veh.ForwardSpeedMs = math.dot(newVel, chassisForward);
            veh.LocalAcceleration = math.rotate(math.conjugate(veh.Rotation), linearAccel);
        }
    }
}
