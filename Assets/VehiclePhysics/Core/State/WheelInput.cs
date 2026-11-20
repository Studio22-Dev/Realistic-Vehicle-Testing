using System.Runtime.InteropServices;

namespace VehiclePhysics.Core
{
    public static class WheelControlFlagBits
    {
        public const byte None        = 0;
        public const byte AllowAbs    = 1 << 0;
        public const byte AllowTcs    = 1 << 1;
        public const byte IsDriven    = 1 << 2;
        public const byte IsSteerable = 1 << 3;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WheelInput
    {
        public float TargetSteerAngleRad;
        public float ServiceBrakeTorqueNm;
        public float HandbrakeTorqueNm;
        public float DriveTorqueShare;

        public int ContactSampleStartIndex;
        public int ContactSampleCount;
        public float DynamicTirePressureKPa; // Reserved for Phase 2
        public byte PatchMode;               // 0 = Phase 1 single-ray, 1 = Phase 2 volumetric
        public byte ControlFlags;
        private ushort _pad0;
    }
}
