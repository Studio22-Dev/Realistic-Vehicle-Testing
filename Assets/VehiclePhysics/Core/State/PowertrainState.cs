using System.Runtime.InteropServices;

namespace VehiclePhysics.Core
{
    public static class ClutchLockStates
    {
        public const byte Open     = 0;
        public const byte Slipping = 1;
        public const byte Locked   = 2;
    }

    public static class DrivelineFlagBits
    {
        public const byte None            = 0;
        public const byte IsShifting      = 1 << 0;
        public const byte RevLimiterCut   = 1 << 1;
        public const byte AutoClutch      = 1 << 2;
        public const byte EngineStalled   = 1 << 3;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PowertrainState
    {
        public float EngineOmega;
        public float GearboxInputOmega;
        public float DriveshaftOmega;
        public float EffectiveGearRatio;

        public float EngineTorqueNm;
        public float ClutchTorqueNm;
        public float ClutchEngagement;
        public float ClutchSlipOmega;

        public float ThrottleFiltered;
        public float ShiftTimer;
        public float RevLimiterTimer;
        public float WarmStartClutchImpulse;

        public float WarmStartFrontLsdImpulse;
        public float WarmStartRearLsdImpulse;
        public float CrankAngleRad;
        public sbyte CurrentGear;      // -1 = R, 0 = N, 1..8 = Forward
        public sbyte TargetGear;
        public byte ClutchLockState;
        public byte DrivelineFlags;
    }
}
