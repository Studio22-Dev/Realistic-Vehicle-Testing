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
            NativeArray<WheelContact> wheelContacts,
            NativeArray<WheelState> wheels,
            in FixedStepTireCache tireCache,
            float dtSub)
        {
            int start = veh.WheelStartIndex;
            int cfgStart = vCfg.WheelConfigStartIndex;

            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                WheelInput wIn = wheelInputs[slot];
                WheelContact contact = wheelContacts[slot];
                WheelState ws = wheels[slot];

                bool grounded = (ws.WheelFlags & WheelFlagBits.IsGrounded) != 0;
                TireStepResult tr = tireCache.Get(w);

                float inertiaBase = math.max(wCfg.WheelInertiaKgM2, 0.2f);
                float iEff = inertiaBase + (grounded ? dtSub * ws.EffectiveRollingRadius * tr.dFx_dOmega : 0f);

                float roadReactionTorque = grounded ? (-tr.Fx * ws.EffectiveRollingRadius) : 0f;
                float omegaStar = ws.AngularVelocity + (roadReactionTorque * dtSub) / iEff;

                float rollingTorque = grounded
                    ? contact.RollingResistance * ws.GroundLoadN * ws.EffectiveRollingRadius
                    : 0f;
                float totalResistCap = wIn.ServiceBrakeTorqueNm + wIn.HandbrakeTorqueNm + rollingTorque;

                float jStop = -iEff * omegaStar;
                float jMax = totalResistCap * dtSub;
                float jApplied = math.clamp(jStop, -jMax, jMax);

                ws.AngularVelocity = omegaStar + jApplied / iEff;
                ws.BrakeTorqueAppliedNm = math.abs(jApplied) / math.max(dtSub, 1e-6f);
                ws.WarmStartBrakeImpulse = jApplied;
                ws.NetDriveTorqueNm = 0f;

                if (math.abs(ws.AngularVelocity) <= 1e-4f && totalResistCap > 1f)
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
