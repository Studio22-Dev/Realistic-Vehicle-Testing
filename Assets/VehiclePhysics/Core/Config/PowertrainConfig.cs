using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct PowertrainConfig
    {
        // 16 evenly spaced WOT engine torque samples [CurveMinOmega .. CurveMaxOmega] (Nm)
        public float4 WotTorque0;
        public float4 WotTorque1;
        public float4 WotTorque2;
        public float4 WotTorque3;

        // Forward Gear Ratios 1..8
        public float4 ForwardGears1To4;
        public float4 ForwardGears5To8;

        public float ReverseGearRatio; // Negative value (e.g., -3.45f)
        public float FinalDriveRatio;
        public float EngineInertia;
        public float GearboxInertia;

        public float ClutchMaxTorqueNm;
        public float ClutchStaticHysteresis; // e.g., 1.15f
        public float DrivelineEfficiency;    // e.g., 0.90f
        public float DrivelineBaseDragNm;

        public float IdleOmega;
        public float RedlineOmega;
        public float RevLimiterDropOmega;
        public float RevLimiterCutTimeS;

        public float CurveMinOmega;
        public float CurveMaxOmega;
        public float EngineFrictionBaseNm;
        public float EngineFrictionViscous; // Nm per (rad/s)

        public float FrontAxleTorqueSplit; // 0 = RWD, 1 = FWD, 0.4 = 40/60 AWD
        public float ShiftDurationS;
        public float FrontLsdPreloadNm;
        public float FrontLsdLockBias;

        public float FrontLsdViscous;
        public float RearLsdPreloadNm;
        public float RearLsdLockBias;
        public float RearLsdViscous;

        public float AutoShiftUpOmega;
        public float AutoShiftDownOmega;
        public sbyte ForwardGearCount;
        public byte AutoTransmission; // 1 = Auto, 0 = Manual
        private ushort _pad0;
        private int _pad1;

        public float GetGearRatio(sbyte gear)
        {
            if (gear == 0) return 0f;
            if (gear < 0) return ReverseGearRatio;
            if (gear > ForwardGearCount) return 0f;

            return gear switch
            {
                1 => ForwardGears1To4.x,
                2 => ForwardGears1To4.y,
                3 => ForwardGears1To4.z,
                4 => ForwardGears1To4.w,
                5 => ForwardGears5To8.x,
                6 => ForwardGears5To8.y,
                7 => ForwardGears5To8.z,
                8 => ForwardGears5To8.w,
                _ => 0f
            };
        }

        public float EvaluateNetEngineTorque(float omega, float throttle)
        {
            float span = math.max(CurveMaxOmega - CurveMinOmega, 1e-3f);
            float tNorm = math.saturate((omega - CurveMinOmega) / span) * 15.0f;
            int idx0 = math.clamp((int)math.floor(tNorm), 0, 14);
            int idx1 = idx0 + 1;
            float frac = tNorm - idx0;

            float wot = math.lerp(SampleWot(idx0), SampleWot(idx1), frac);
            float friction = EngineFrictionBaseNm + EngineFrictionViscous * math.max(omega, 0f);
            // At 0 throttle, net torque equals negative engine braking; at full throttle, WOT curve.
            return math.lerp(-friction, wot, math.saturate(throttle));
        }

        private float SampleWot(int index)
        {
            float4 v = (index >> 2) switch
            {
                0 => WotTorque0,
                1 => WotTorque1,
                2 => WotTorque2,
                _ => WotTorque3
            };
            return (index & 3) switch
            {
                0 => v.x,
                1 => v.y,
                2 => v.z,
                _ => v.w
            };
        }
    }
}
