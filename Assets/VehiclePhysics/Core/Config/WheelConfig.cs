using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    /// <summary>
    /// Unmanaged per-wheel archetype configuration.
    /// Exact size: 144 bytes (9 x 16B), zero hidden alignment gaps.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelConfig
    {
        public float3 MountPositionLocal;          // 0..11   (12B)
        public float  UnloadedRadiusM;             // 12..15  ( 4B)

        public float3 StrutAxisLocal;              // 16..27  (12B) Normalized, typically (0, -1, 0)
        public float  RestLengthM;                 // 28..31  ( 4B)

        public float  MaxTravelM;                  // 32..35
        public float  SpringStiffnessNPerM;        // 36..39
        public float  BumpDamperNsPerM;            // 40..43
        public float  ReboundDamperNsPerM;         // 44..47

        public float  BumpStopStiffnessNPerM;      // 48..51
        public float  WheelInertiaKgM2;            // 52..55
        public float  RadialTireStiffnessNPerM;    // 56..59
        public float  MaxRadialSquishM;            // 60..63

        public float  RelaxationLengthX;           // 64..67
        public float  RelaxationLengthY;           // 68..71
        public float  PeakSlipRatio;               // 72..75
        public float  PeakSlipAngleRad;            // 76..79

        public float  MaxServiceBrakeTorqueNm;     // 80..83
        public float  MaxHandbrakeTorqueNm;        // 84..87
        public float  TirePeakMuScale;             // 88..91
        public float  TireSlideMuScale;            // 92..95

        public float  LowSpeedDampMaxForceScale;   // 96..99
        public float  MechanicalTrailM;            // 100..103
        public float  RimRadiusM;                  // 104..107
        public float  RadialStiffnessNPerM;        // 108..111

        public float  LateralStiffnessNPerM;       // 112..115
        public float  LongitudinalStiffnessNPerM;  // 116..119
        public float  NominalPressureKPa;          // 120..123
        public float  InitialTreadDepthMm;         // 124..127

        public float  UnsprungMassKg;              // 128..131
        private float _reserved0;                  // 132..135
        public byte   AxleIndex;                   // 136
        public byte   SideIndex;                   // 137
        public byte   DefaultControlFlags;         // 138 WheelControlFlagBits
        public TireCompoundType CompoundType;      // 139
        public TireRimType RimType;                // 140
        private byte  _pad0;                       // 141
        private byte  _pad1;                       // 142
        private byte  _pad2;                       // 143 -> 144B total
    }
}
