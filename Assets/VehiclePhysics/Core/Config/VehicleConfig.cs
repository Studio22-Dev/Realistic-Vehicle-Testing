using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct VehicleConfig
    {
        public float3 CenterOfMassLocalOffset;
        public float MassKg;

        public float3 InertiaLocal;
        public float GravityMs2;

        public float3 AeroCenterOfPressureLocal;
        public float AeroDragAreaCdA;

        public float AirDensityKgM3;
        public float AeroFrontDownforceClA;
        public float AeroRearDownforceClA;
        public float MaxSteerAngleRad;

        public float WheelbaseM;
        public float TrackWidthFrontM;
        public float TrackWidthRearM;
        public float AckermannFactor;

        public float FrontAntiRollStiffnessNPerM;
        public float RearAntiRollStiffnessNPerM;
        public float SteeringRateRadPerS;
        public float HighSpeedSteerScale;

        public int WheelConfigStartIndex;
        public int WheelCount;
        public byte HasPowertrain; // 1 = powered vehicle/cab, 0 = unpowered trailer
        private byte _pad0;
        private byte _pad1;
        private byte _pad2;
        private int _pad3;
    }
}
