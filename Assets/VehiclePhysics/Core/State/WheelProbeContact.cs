using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    /// <summary>
    /// Raw volumetric contact plane from Capsulecast/Spherecast for Step 3 Non-Rotating XPBD Ring.
    /// Exact size: 48 bytes (3 x 16B).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelProbeContact
    {
        public float3 WorldPoint;       // 0..11
        public float  PenetrationM;     // 12..15 (>0 = inside undeformed tire envelope)

        public float3 WorldNormal;      // 16..27
        public float  ProbeAngleRad;    // 28..31 (0=bottom, +=fwd, -=rear, +/-PI/2=sidewall)

        public float3 SurfaceVelocity;  // 32..43
        public int    DynamicBodyIndex; // 44..47 (-1 = static world)
    }
}
