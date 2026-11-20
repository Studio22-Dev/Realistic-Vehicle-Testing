using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class WheelFlagBits
    {
        public const byte None          = 0;
        public const byte IsGrounded    = 1 << 0;
        public const byte IsBrakeLocked = 1 << 1;
        public const byte AbsActive     = 1 << 2;
        public const byte TcsActive     = 1 << 3;
        public const byte IsDetached    = 1 << 4; // Missing wheel
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WheelState
    {
        public float3 LastWorldForce;
        public float AngularVelocity;

        public float SpinAngle;
        public float SteerAngleRad;
        public float CamberAngleRad;
        public float SuspensionCompression;

        public float SuspensionVelocity;
        public float GroundLoadN;
        public float LongitudinalDeflection;
        public float LateralDeflection;

        public float EffectiveSlipRatio;
        public float EffectiveSlipAngleRad;
        public float NetDriveTorqueNm;
        public float BrakeTorqueAppliedNm;

        public float WarmStartBrakeImpulse;
        public float EffectiveRollingRadius;
        public float FrictionUtilization;
        public byte WheelFlags;
        public byte AxleIndex;
        public byte SideIndex;
        private byte _pad0;
    }
}
