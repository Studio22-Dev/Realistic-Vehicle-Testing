using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class WheelSpinFallbackSolver
    {
        public static void SolveWheelSpinAndBrakes(
            in VehicleState veh,
            in VehicleConfig vCfg,
            NativeArray<WheelConfig> wheelConfigs,
            NativeArray<WheelInput> wheelInputs,
            NativeArray<ContactManifold> contactManifolds,
            NativeArray<SurfaceMaterialData> surfaceMaterials,
            NativeArray<WheelState> wheels,
            in FixedStepTireCache tireCache,
            float dtSub)
        {
            int start = veh.WheelStartIndex;
            int cfgStart = vCfg.WheelConfigStartIndex;
            float invDt = 1.0f / math.max(dtSub, 1e-6f);

            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];

                if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0)
                {
                    ws.AngularVelocity = 0f;
                    ws.BrakeTorqueAppliedNm = 0f;
                    ws.WarmStartBrakeImpulse = 0f;
                    ws.NetDriveTorqueNm = 0f;
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsBrakeLocked);
                    wheels[slot] = ws;
                    continue;
                }

                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                WheelInput wIn = wheelInputs[slot];
                ContactManifold manifold = contactManifolds[slot];

                bool grounded = (ws.WheelFlags & WheelFlagBits.IsGrounded) != 0 && manifold.IsGrounded != 0;
                TireStepResult tr = tireCache.Get(w);

                float inertiaBase = math.max(wCfg.WheelInertiaKgM2, 0.2f);
                float iEff = inertiaBase + (grounded ? dtSub * ws.EffectiveRollingRadius * tr.dFx_dOmega : 0f);

                float roadReactionTorque = grounded ? (-tr.Fx * ws.EffectiveRollingRadius) : 0f;
                float omegaStar = ws.AngularVelocity + (roadReactionTorque * dtSub) / iEff;

                // Passive zero-clamped rolling resistance (prevents parked creep and keeps brake telemetry clean)
                if (grounded)
                {
                    int matIdx = math.clamp(manifold.PrimaryMaterialIndex, 0, surfaceMaterials.Length - 1);
                    float rrCoeff = surfaceMaterials[matIdx].RollingResistance;
                    float jRrMax = math.max(0f, rrCoeff * ws.GroundLoadN * ws.EffectiveRollingRadius) * dtSub;
                    float dJRr = math.clamp(-iEff * omegaStar, -jRrMax, jRrMax);
                    omegaStar += dJRr / iEff;
                }

                // Accumulated brake impulse clamp + warm-start clamping
                float totalBrakeCapNm = math.max(0f, wIn.ServiceBrakeTorqueNm + wIn.HandbrakeTorqueNm);
                float jBrakeMax = totalBrakeCapNm * dtSub;
                float jWarm = math.clamp(ws.WarmStartBrakeImpulse, -jBrakeMax, jBrakeMax);
                float omegaWork = omegaStar + jWarm / iEff;

                float dJStop = -iEff * omegaWork;
                float jBrakeTotal = math.clamp(jWarm + dJStop, -jBrakeMax, jBrakeMax);
                float dJApply = jBrakeTotal - jWarm;

                ws.AngularVelocity = omegaWork + dJApply / iEff;
                ws.BrakeTorqueAppliedNm = math.abs(jBrakeTotal) * invDt;
                ws.WarmStartBrakeImpulse = jBrakeTotal;
                ws.NetDriveTorqueNm = 0f;

                if (math.abs(ws.AngularVelocity) <= 1e-4f && jBrakeMax > dtSub)
                    ws.WheelFlags |= WheelFlagBits.IsBrakeLocked;
                else
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsBrakeLocked);

                ws.SpinAngle = math.fmod(ws.SpinAngle + ws.AngularVelocity * dtSub, math.PI * 2.0f);
                if (ws.SpinAngle < 0f) ws.SpinAngle += math.PI * 2.0f;

                wheels[slot] = ws;
            }
        }
    }
}
