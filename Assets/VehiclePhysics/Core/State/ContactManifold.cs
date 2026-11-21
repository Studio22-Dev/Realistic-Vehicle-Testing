using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    /// <summary>
    /// Raw volumetric contact plane from Capsulecast/Spherecast (Step 2 -> Step 3 XPBD ring).
    /// Exact size: 64 bytes
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelContactSample
    {
        public float3 Point;               // 0..11 (12B)
        public float  PenetrationDepth;    // 12..15 (4B) -> 16B

        public float3 Normal;              // 16..27 (12B)
        public float  StaticFriction;      // 28..31 (4B) -> 32B

        public float3 SurfaceVelocity;     // 32..43 (12B)
        public float  DynamicFriction;     // 44..47 (4B) -> 48B

        public float  RollingResistance;   // 48..51 (4B)
        public float  SampleWeight;        // 52..55 (4B)
        public float  ProbeAngleRad;       // 56..59 (4B)

        public byte   IsValid;             // 60 (1B)
        public byte   SurfaceMaterialType; // 61 (1B)
        public byte   ZoneId;              // 62 (1B) WheelProbeZones
        private byte  _pad0;               // 63 (1B) -> 64B total
    }

    public static class WheelProbeZones
    {
        public const byte Bottom   = 0;
        public const byte Leading  = 1;
        public const byte Trailing = 2;
        public const byte Sidewall = 3;
    }

    public static class ManifoldClusterFlags
    {
        public const byte None           = 0;
        public const byte HitDynamicBody = 1 << 0;
        public const byte HitRimSidewall = 1 << 1;
        public const byte Submerged      = 1 << 2;
        public const byte DeformableSoil = 1 << 3;
    }

    public static class ManifoldFlags
    {
        public const byte None               = 0;
        public const byte HasRimImpact       = 1 << 0;
        public const byte InStandingWater    = 1 << 1;
        public const byte OnDeformableSoil   = 1 << 2;
        public const byte TouchesDynamicBody = 1 << 3;
        public const byte HasWallImpact      = 1 << 4; // Non-climbable horizontal impact in C1/C2
    }

    /// <summary>
    /// Single clustered contact zone (C0 Bottom, C1 Leading, C2 Trailing, C3 Sidewall).
    /// Exact size: 80 bytes (5 x 16B).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ManifoldContactPoint
    {
        public float3 WorldPoint;             // 0..11   (12B)
        public float  PenetrationM;           // 12..15  ( 4B)

        public float3 WorldNormal;            // 16..27  (12B)
        public float  ContactForceN;          // 28..31  ( 4B)

        public float3 SurfaceVelocity;        // 32..43  (12B)
        public int    ColliderInstanceID;     // 44..47  ( 4B)

        public float3 PendingObjectImpulse;   // 48..59  (12B) Drained post-step onto dynamic bodies
        public int    DynamicBodyIndex;       // 60..63  ( 4B) -1 = static world

        public float  ContactWeight;          // 64..67  ( 4B) Normalized zone blend weight [0..1]
        public float  LocalFluidOrLooseDepthM;// 68..71  ( 4B) Puddle/loose-soil layer depth (m)
        public float  FrictionScale;          // 72..75  ( 4B) Per-cluster collider friction multiplier
        public byte   SurfaceMaterialIndex;   // 76      ( 1B)
        public byte   IsValid;                // 77      ( 1B) 1 = active cluster
        public byte   SampleCount;            // 78      ( 1B) Number of raw probes merged into cluster
        public byte   ClusterFlags;           // 79      ( 1B) ManifoldClusterFlags -> 80B total
    }

    /// <summary>
    /// 4-Zone volumetric wheel contact manifold replacing Phase 1 WheelContact.
    /// Exact size: 384 bytes (4 x 80B clusters + 64B summary = 6 x 64B cache lines).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ContactManifold
    {
        // --- Cache Lines 0..4 (0..319): 4 Directional Contact Zones ---
        public ManifoldContactPoint ClusterBottom;   // C0: Downward patch (0..79)
        public ManifoldContactPoint ClusterLeading;  // C1: Forward climb / curb (80..159)
        public ManifoldContactPoint ClusterTrailing; // C2: Rear departure (160..239)
        public ManifoldContactPoint ClusterSidewall; // C3: Lateral sidewall / rim (240..319)

        // --- Cache Line 5 (320..383): Aggregate Patch & Terrain/Water State ---
        public float3 BlendedPoint;            // 320..331 (12B)
        public float  ContactPatchLengthM;     // 332..335 ( 4B)

        public float3 BlendedNormal;           // 336..347 (12B)
        public float  WaterDepthM;             // 348..351 ( 4B)

        public float3 BlendedVelocity;         // 352..363 (12B)
        public float  TerrainSinkageM;         // 364..367 ( 4B)

        public float  HydroplaningFactor;      // 368..371 ( 4B) [0..1] Horne wedge lift
        public float  BulldozingDragN;         // 372..375 ( 4B) Bekker-Wong bulldozing resistance
        public float  BlendedFrictionScale;    // 376..379 ( 4B) Weighted collider friction scale
        public byte   ActiveClusterMask;       // 380      ( 1B) Bits 0..3 for C0..C3
        public byte   PrimaryMaterialIndex;    // 381      ( 1B) Index into SurfaceMaterials
        public byte   IsGrounded;              // 382      ( 1B) 1 if any cluster valid
        public byte   Flags;                   // 383      ( 1B) ManifoldFlags -> 384B total
    }
}
