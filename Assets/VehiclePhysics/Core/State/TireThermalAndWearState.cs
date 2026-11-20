using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public enum TireCompoundType : byte
    {
        Street     = 0,
        SemiSlick  = 1,
        Slick      = 2,
        AllTerrain = 3,
        MudTerrain = 4,
        Winter     = 5,
        Drag       = 6
    }

    public static class TireThermalFlags
    {
        public const byte None       = 0;
        public const byte Graining   = 1 << 0;
        public const byte Blistering = 1 << 1;
        public const byte Overheated = 1 << 2;
        public const byte FlatSpot   = 1 << 3;
    }

    /// <summary>
    /// 3-zone (Inner, Middle, Outer) tire temperature, wear, clogging, and 16-bin abrasion flat spots.
    /// Exact size: 128 bytes (2 x 64B cache lines).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct TireThermalAndWearState
    {
        // --- Cache Line 0 (0..63): 3-Zone Temps, Gas Temp & Wear ---
        public float3 TreadSurfaceTempC;       // 0..11   (12B) x=Inner, y=Middle, z=Outer
        public float  CarcassBulkTempC;        // 12..15  ( 4B)

        public float3 TreadSubsurfaceTempC;    // 16..27  (12B) x=Inner, y=Middle, z=Outer
        public float  CoreGasTempC;            // 28..31  ( 4B) Coupled to pressure via Ideal Gas Law

        public float3 RemainingTreadDepthMm;   // 32..43  (12B) x=Inner, y=Middle, z=Outer
        public float  MudAndDebrisClogging;    // 44..47  ( 4B) [0..1]

        public float  ThermalFlatSpotSeverity; // 48..51  ( 4B) [0..1] transient parked flat spot
        public float  ThermalFlatSpotAngleRad; // 52..55  ( 4B) Spin phase of thermal flat spot
        public float  CurrentThermalGripScale; // 56..59  ( 4B) Cached compound thermal multiplier
        public byte   CompoundType;            // 60      ( 1B) TireCompoundType
        public byte   Flags;                   // 61      ( 1B) TireThermalFlags
        private byte  _pad0;                   // 62      ( 1B)
        private byte  _pad1;                   // 63      ( 1B)

        // --- Cache Line 1 (64..127): 16-Bin Circumferential Abrasion Flat-Spot Depths (mm) ---
        public float4 FlatSpotDepthBins0;      // 64..79   (16B) Bins  0..3
        public float4 FlatSpotDepthBins1;      // 80..95   (16B) Bins  4..7
        public float4 FlatSpotDepthBins2;      // 96..111  (16B) Bins  8..11
        public float4 FlatSpotDepthBins3;      // 112..127 (16B) Bins 12..15 -> 128B total

        public readonly float GetFlatSpotBinMm(int binIndex)
        {
            int idx = binIndex & 15;
            int lane = idx & 3;
            return (idx >> 2) switch
            {
                0 => FlatSpotDepthBins0[lane],
                1 => FlatSpotDepthBins1[lane],
                2 => FlatSpotDepthBins2[lane],
                _ => FlatSpotDepthBins3[lane]
            };
        }

        public void AddFlatSpotBinMm(int binIndex, float deltaMm, float maxDepthMm)
        {
            int idx = binIndex & 15;
            int lane = idx & 3;
            switch (idx >> 2)
            {
                case 0: FlatSpotDepthBins0[lane] = math.min(FlatSpotDepthBins0[lane] + deltaMm, maxDepthMm); break;
                case 1: FlatSpotDepthBins1[lane] = math.min(FlatSpotDepthBins1[lane] + deltaMm, maxDepthMm); break;
                case 2: FlatSpotDepthBins2[lane] = math.min(FlatSpotDepthBins2[lane] + deltaMm, maxDepthMm); break;
                default: FlatSpotDepthBins3[lane] = math.min(FlatSpotDepthBins3[lane] + deltaMm, maxDepthMm); break;
            }
        }
    }
}
