using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public struct FixedStepTireCache
    {
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
        private const float SidewallStiffnessNPerM = 160000f;
        private const float SidewallDampingNsPerM  = 3500f;
        private const float SidewallMaxForceN      = 35000f;

        private const float WallStiffnessNPerM     = 220000f;
        private const float WallDampingNsPerM      = 4500f;
        private const float WallMaxForceN          = 45000f;

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

            float3 chassisUp = math.rotate(veh.Rotation, new float3(0f, 1f, 0f));

            // Wheel Ground Patch, Sidewall C3, Non-Climbable Wall Impacts & Dynamic Body Impulses
            int start = veh.WheelStartIndex;
            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];
                if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0)
                {
                    ws.LastWorldForce = float3.zero;
                    wheels[slot] = ws;
                    continue;
                }

                ContactManifold manifold = contactManifolds[slot];
                bool manifoldDirty = false;
                float3 wheelForceTotal = float3.zero;

                // 1. Ground patch forces (normal + longitudinal + lateral)
                if ((ws.WheelFlags & WheelFlagBits.IsGrounded) != 0 && manifold.IsGrounded != 0)
                {
                    TireStepResult tr = tireCache.Get(w);
                    float3 patchForce = ws.GroundLoadN * manifold.BlendedNormal
                                      + tr.Fx * tr.LongDir
                                      + tr.Fy * tr.LatDir;

                    wheelForceTotal += patchForce;
                    totalTorque += math.cross(tr.WorldApplicationPoint - veh.Position, patchForce);

                    if ((manifold.Flags & ManifoldFlags.TouchesDynamicBody) != 0)
                    {
                        float3 reactPatchImp = -patchForce * dtSub;
                        float dynW =
                            (manifold.ClusterBottom.DynamicBodyIndex   >= 0 ? manifold.ClusterBottom.ContactWeight   : 0f) +
                            (manifold.ClusterLeading.DynamicBodyIndex  >= 0 ? manifold.ClusterLeading.ContactWeight  : 0f) +
                            (manifold.ClusterTrailing.DynamicBodyIndex >= 0 ? manifold.ClusterTrailing.ContactWeight : 0f);

                        if (dynW > 1e-5f)
                        {
                            float invW = 1.0f / dynW;
                            if (manifold.ClusterBottom.DynamicBodyIndex >= 0)
                                manifold.ClusterBottom.PendingObjectImpulse += reactPatchImp * (manifold.ClusterBottom.ContactWeight * invW);
                            if (manifold.ClusterLeading.DynamicBodyIndex >= 0)
                                manifold.ClusterLeading.PendingObjectImpulse += reactPatchImp * (manifold.ClusterLeading.ContactWeight * invW);
                            if (manifold.ClusterTrailing.DynamicBodyIndex >= 0)
                                manifold.ClusterTrailing.PendingObjectImpulse += reactPatchImp * (manifold.ClusterTrailing.ContactWeight * invW);
                            manifoldDirty = true;
                        }
                    }
                }

                // 2. Sidewall / Curb Rub C3 (active even when airborne)
                float fSide = 0f;
                if (manifold.ClusterSidewall.IsValid != 0 && manifold.ClusterSidewall.PenetrationM > 0f)
                {
                    fSide = EvaluateContactPushback(
                        in veh, in manifold.ClusterSidewall,
                        SidewallStiffnessNPerM, SidewallDampingNsPerM, SidewallMaxForceN);

                    if (fSide > 0f)
                    {
                        float3 sideForce = manifold.ClusterSidewall.WorldNormal * fSide;
                        wheelForceTotal += sideForce;
                        totalTorque += math.cross(manifold.ClusterSidewall.WorldPoint - veh.Position, sideForce);
                        manifold.ClusterSidewall.ContactForceN = fSide;

                        if (manifold.ClusterSidewall.DynamicBodyIndex >= 0)
                            manifold.ClusterSidewall.PendingObjectImpulse += -sideForce * dtSub;

                        manifoldDirty = true;
                    }
                }

                if ((manifold.Flags & ManifoldFlags.HasWallImpact) != 0)
                {
                    if (manifold.ClusterLeading.IsValid != 0 &&
                        manifold.ClusterLeading.PenetrationM > 0f &&
                        math.dot(manifold.ClusterLeading.WorldNormal, chassisUp) < 0.30f)
                    {
                        float fW1 = EvaluateContactPushback(
                            in veh, in manifold.ClusterLeading,
                            WallStiffnessNPerM, WallDampingNsPerM, WallMaxForceN);
                        if (fW1 > 0f)
                        {
                            float3 wf1 = manifold.ClusterLeading.WorldNormal * fW1;
                            wheelForceTotal += wf1;
                            totalTorque += math.cross(manifold.ClusterLeading.WorldPoint - veh.Position, wf1);
                            manifold.ClusterLeading.ContactForceN = fW1;
                            if (manifold.ClusterLeading.DynamicBodyIndex >= 0)
                                manifold.ClusterLeading.PendingObjectImpulse += -wf1 * dtSub;
                            manifoldDirty = true;
                        }
                    }

                    if (manifold.ClusterTrailing.IsValid != 0 &&
                        manifold.ClusterTrailing.PenetrationM > 0f &&
                        math.dot(manifold.ClusterTrailing.WorldNormal, chassisUp) < 0.30f)
                    {
                        // 3. Solid Wall Pushback in C1 / C2 (non-climbable horizontal strikes)
                        float fW2 = EvaluateContactPushback(
                            in veh, in manifold.ClusterTrailing,
                            WallStiffnessNPerM, WallDampingNsPerM, WallMaxForceN);
                        if (fW2 > 0f)
                        {
                            float3 wf2 = manifold.ClusterTrailing.WorldNormal * fW2;
                            wheelForceTotal += wf2;
                            totalTorque += math.cross(manifold.ClusterTrailing.WorldPoint - veh.Position, wf2);
                            manifold.ClusterTrailing.ContactForceN = fW2;
                            if (manifold.ClusterTrailing.DynamicBodyIndex >= 0)
                                manifold.ClusterTrailing.PendingObjectImpulse += -wf2 * dtSub;
                            manifoldDirty = true;
                        }
                    }
                }

                ws.LastWorldForce = wheelForceTotal;
                wheels[slot] = ws;
                totalForce += wheelForceTotal;

                if (manifoldDirty)
                    contactManifolds[slot] = manifold;
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

            // Advance C3 and solid-wall penetrations at substep rate so rebounds unload smoothly at 540Hz
            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                if ((wheels[slot].WheelFlags & WheelFlagBits.IsDetached) != 0)
                    continue;

                ContactManifold m = contactManifolds[slot];
                bool changed = false;

                if (m.ClusterSidewall.IsValid != 0 && m.ClusterSidewall.PenetrationM > 0f)
                {
                    AdvanceSubstepPenetration(in veh, ref m.ClusterSidewall, dtSub);
                    changed = true;
                }
                if ((m.Flags & ManifoldFlags.HasWallImpact) != 0)
                {
                    if (m.ClusterLeading.IsValid != 0 && m.ClusterLeading.PenetrationM > 0f &&
                        math.dot(m.ClusterLeading.WorldNormal, chassisUp) < 0.30f)
                    {
                        AdvanceSubstepPenetration(in veh, ref m.ClusterLeading, dtSub);
                        changed = true;
                    }
                    if (m.ClusterTrailing.IsValid != 0 && m.ClusterTrailing.PenetrationM > 0f &&
                        math.dot(m.ClusterTrailing.WorldNormal, chassisUp) < 0.30f)
                    {
                        AdvanceSubstepPenetration(in veh, ref m.ClusterTrailing, dtSub);
                        changed = true;
                    }
                }

                if (changed)
                    contactManifolds[slot] = m;
            }

            float3 chassisForward = math.rotate(veh.Rotation, new float3(0f, 0f, 1f));
            veh.ForwardSpeedMs = math.dot(newVel, chassisForward);
            veh.LocalAcceleration = math.rotate(math.conjugate(veh.Rotation), linearAccel);
        }

        private static float EvaluateContactPushback(
            in VehicleState veh,
            in ManifoldContactPoint contact,
            float stiffness,
            float damping,
            float maxForce)
        {
            float3 vPoint = veh.LinearVelocity
                          + math.cross(veh.AngularVelocity, contact.WorldPoint - veh.Position)
                          - contact.SurfaceVelocity;
            float vRelN = math.dot(vPoint, contact.WorldNormal);
            return math.clamp(stiffness * contact.PenetrationM - damping * vRelN, 0f, maxForce);
        }

        private static void AdvanceSubstepPenetration(
            in VehicleState veh,
            ref ManifoldContactPoint contact,
            float dtSub)
        {
            float3 vPost = veh.LinearVelocity
                         + math.cross(veh.AngularVelocity, contact.WorldPoint - veh.Position)
                         - contact.SurfaceVelocity;
            contact.PenetrationM = math.max(0f, contact.PenetrationM - math.dot(vPost, contact.WorldNormal) * dtSub);
        }
    }
}
