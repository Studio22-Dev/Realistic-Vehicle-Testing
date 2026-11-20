using System.Runtime.InteropServices;

namespace VehiclePhysics.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct VehicleDriverInput
    {
        public float Throttle;         // [0..1]
        public float Brake;            // [0..1]
        public float Steer;            // [-1..1]
        public float ClutchPedal;      // [0..1], 0 = engaged, 1 = disengaged
        public float Handbrake;        // [0..1]
        public sbyte RequestedGear;    // -128 = auto, -1 = R, 0 = N, 1..8 = manual target
        public sbyte ShiftDelta;       // -1 = down, 0 = none, +1 = up
        public byte InputFlags;        // Bit0: AbsOff | Bit1: TcsOff | Bit2: Starter
        private byte _pad0;
        private long _pad1;
    }
}
