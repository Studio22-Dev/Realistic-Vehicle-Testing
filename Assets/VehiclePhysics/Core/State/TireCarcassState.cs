using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class TireCarcassFlags
    {
        public const byte None           = 0;
        public const byte RimBottomedOut = 1 << 0;
        public const byte IsDeBeaded     = 1 << 1;
        public const byte IsBlownOut     = 1 << 2;
        public const byte IsFlat         = 1 << 3;
    }

    public enum TireRimType : byte
    {
        StandardAlloy = 0,
        Steel         = 1,
        Forged        = 2,
        Beadlock      = 3
    }

    /// <summary>
    /// Option 0 tire carcass flex, inflation/integrity, steering FFB trail, and wheel rim sub-state.
    /// Exact size: 128 bytes (2 x 64B cache lines).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct TireCarcassState
    {
        // --- Cache Line 0 (0..63): Carcass Flex, FFB & Stiffness ---
        public float3 CarcassDeflectionLocal;     // 0..11   (12B) (x=Long, y=Radial, z=Lat) m
        public float  TorsionalTwistRad;          // 12..15  ( 4B) Rim-to-belt windup (rad)

        public float3 CarcassVelocityLocal;       // 16..27  (12B) (x=Long, y=Radial, z=Lat) m/s
        public float  TorsionalTwistRateRadS;     // 28..31  ( 4B) Twist rate (rad/s)

        public float3 RimLocalImpactOffset;       // 32..43  (12B) Rim-space impact offset (m)
        public float  RimRadiusM;                 // 44..47  ( 4B) Rigid wheel rim outer radius (m)

        public float  PneumaticTrailM;            // 48..51  ( 4B) Dynamic pneumatic trail (m)
        public float  SelfAligningTorqueNm;       // 52..55  ( 4B) Steering FFB SAT (Nm)
        public float  EffectiveLoadedRadiusM;     // 56..59  ( 4B) Loaded hub-to-ground radius (m)
        public float  RadialStiffnessNPerM;       // 60..63  ( 4B) Pressure-coupled vertical stiffness

        // --- Cache Line 1 (64..127): Pressure, Integrity & Rim Damage State ---
        public float  InternalPressureKPa;        // 64..67  ( 4B) Live cavity pressure (kPa)
        public float  ColdReferencePressureKPa;   // 68..71  ( 4B) Nominal cold pressure (kPa)
        public float  LeakRateKPaPerSec;          // 72..75  ( 4B) Puncture air loss rate (kPa/s)
        public float  BeadSealLeakRateKPaPerSec;  // 76..79  ( 4B) Chronic bead leak rate (kPa/s)

        public float  CarcassIntegrity;           // 80..83  ( 4B) [0..1]
        public float  BeadSeatingIntegrity;       // 84..87  ( 4B) [0..1]
        public float  BlowoutProgress;            // 88..91  ( 4B) [0..1]
        public float  RimGroundLoadN;             // 92..95  ( 4B) Rim hard-stop normal load (N)

        public float  RimBentAngleRad;            // 96..99  ( 4B) Wobble bend angle (rad)
        public float  RimBentPhaseRad;            // 100..103( 4B) Circumferential bend phase (rad)
        public float  RimRadialDentDepthM;        // 104..107( 4B) Rim flange dent depth (m)
        public float  RimStructuralHealth;        // 108..111( 4B) [0..1]

        public float  LateralStiffnessNPerM;      // 112..115( 4B)
        public float  LongitudinalStiffnessNPerM; // 116..119( 4B)
        public float  RimPeakImpactImpulseNs;     // 120..123( 4B) Peak rim strike impulse (N*s)
        public byte   Flags;                      // 124     ( 1B) TireCarcassFlags
        public byte   RimType;                    // 125     ( 1B) TireRimType
        private byte  _pad0;                      // 126     ( 1B)
        private byte  _pad1;                      // 127     ( 1B) -> 128B total
    }
}
