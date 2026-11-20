using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class VehicleTierFlags
    {
        public const byte None                   = 0;
        public const byte IsPlayerTier           = 1 << 0;
        public const byte IsGroundedAny          = 1 << 1;
        public const byte IsSleeping             = 1 << 2;
        public const byte ExternalImpulsePending = 1 << 3;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct VehicleState
    {
        public quaternion Rotation;

        public float3 Position;
        public float InverseMass;

        public float3 LinearVelocity;
        public float ForwardSpeedMs;

        public float3 AngularVelocity;
        public int WheelStartIndex;

        public float3 InverseInertiaLocal;
        public int WheelCount;

        public float3 ExternalImpulse;
        public uint VehicleId;

        public float3 ExternalAngularImpulse;
        public int ConfigIndex;

        public float3 LocalAcceleration;
        public ushort SubstepCount;
        public byte TierFlags;
        public byte LodTier; // 0 = Player, 1 = Close AI, 2 = Far AI, 3 = Culled
    }
}
