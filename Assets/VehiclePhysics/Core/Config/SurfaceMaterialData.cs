using System.Runtime.InteropServices;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SurfaceMaterialData
    {
        public float StaticFriction;
        public float DynamicFriction;
        public float RollingResistance;
        public float SurfaceRoughness;
        public float TireGripMultiplier;
        public byte MaterialTypeId;
        public byte SurfaceFlags;
        private ushort _pad0;
        private long _pad1;
    }
}
