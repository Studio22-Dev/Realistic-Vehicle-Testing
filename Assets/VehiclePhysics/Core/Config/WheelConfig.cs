using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelConfig
    {
        public float3 MountPositionLocal;
        public float UnloadedRadiusM;

        public float3 StrutAxisLocal; // Normalized, typically (0, -1, 0)
        public float RestLengthM;

        public float MaxTravelM;
        public float SpringStiffnessNPerM;
        public float BumpDamperNsPerM;
        public float ReboundDamperNsPerM;

        public float BumpStopStiffnessNPerM;
        public float WheelInertiaKgM2;
        public float RadialTireStiffnessNPerM;
        public float MaxRadialSquishM;

        public float RelaxationLengthX;
        public float RelaxationLengthY;
        public float PeakSlipRatio;
        public float PeakSlipAngleRad;

        public float MaxServiceBrakeTorqueNm;
        public float MaxHandbrakeTorqueNm;
        public float TirePeakMuScale;
        public float TireSlideMuScale;

        public float LowSpeedDampMaxForceScale;
        public float MechanicalTrailM;
        public byte AxleIndex;
        public byte SideIndex;
        public byte DefaultControlFlags; // WheelControlFlagBits
        private byte _pad0;
        private int _pad1;
    }
}
