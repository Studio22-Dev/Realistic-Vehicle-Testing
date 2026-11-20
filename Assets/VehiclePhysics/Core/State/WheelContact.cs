using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelContactSample
    {
        public float3 Point;
        public float PenetrationDepth;

        public float3 Normal;
        public float StaticFriction;

        public float3 SurfaceVelocity;
        public float DynamicFriction;

        public float RollingResistance;
        public float SampleWeight;
        public float SurfaceRoughness;
        public byte IsValid;
        public byte SurfaceMaterialType;
        private ushort _pad0;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WheelContact
    {
        public float3 PlanePoint;
        public float StaticFriction;

        public float3 PlaneNormal;
        public float DynamicFriction;

        public float3 SurfaceVelocity;
        public float RollingResistance;

        public float ContactPatchLength;
        public float LateralCoPOffsetLocal;
        public float SurfaceRoughness;
        public byte HasContact;
        public byte SurfaceMaterialType;
        private ushort _pad0;
    }
}
