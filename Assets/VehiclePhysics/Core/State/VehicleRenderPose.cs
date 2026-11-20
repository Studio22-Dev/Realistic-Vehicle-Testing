using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct VehicleRenderPose
    {
        public quaternion PreviousRotation;
        public float3 PreviousPosition;
        private float _pad0;

        public quaternion CurrentRotation;
        public float3 CurrentPosition;
        private float _pad1;
    }
}
