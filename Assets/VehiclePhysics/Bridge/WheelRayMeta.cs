using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Bridge
{
    /// <summary>
    /// Cached per-wheel volumetric frame shared between PrepareWheelRaycastsJob and ProcessWheelHitsJob.
    /// Exact size: 80 bytes (5 x 16B).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelRayMeta
    {
        public float3 HubEstimate;      // 0..11
        public float  CapsuleRadiusM;   // 12..15

        public float3 WheelUp;          // 16..27
        public float  ArcBackoffM;      // 28..31

        public float3 WheelOutward;     // 32..43
        public float  SideRadiusM;      // 44..47

        public float3 WheelForward;     // 48..59
        public float  SideBackoffM;     // 60..63

        public float  HubDropCurrentM;  // 64..67
        public float  HalfWidthM;       // 68..71
        public float  CenterRayDistM;   // 72..75
        public byte   CapsuleCount;     // 76
        public byte   SphereCount;      // 77
        public byte   LodTier;          // 78
        private byte  _pad0;            // 79 -> 80B total
    }
}
