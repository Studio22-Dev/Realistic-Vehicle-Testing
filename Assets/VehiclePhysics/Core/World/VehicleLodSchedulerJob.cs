using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct VehicleLodSchedulerJob : IJobParallelFor
    {
        public float3 CameraOrPlayerPos;
        public ushort PlayerSubsteps;
        public ushort NearAiSubsteps;
        public ushort FarAiSubsteps;
        public float NearAiDistSq;
        public float SleepDistSq;

        public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<VehicleDriverInput> DriverInputs;

        public void Execute(int vehIdx)
        {
            VehicleState v = Vehicles[vehIdx];

            if ((v.TierFlags & VehicleTierFlags.IsPlayerTier) != 0)
            {
                v.LodTier = 0;
                v.SubstepCount = PlayerSubsteps;
                v.TierFlags = (byte)(v.TierFlags & ~VehicleTierFlags.IsSleeping);
                Vehicles[vehIdx] = v;
                return;
            }

            float distSq = math.lengthsq(v.Position - CameraOrPlayerPos);
            VehicleDriverInput input = DriverInputs[vehIdx];
            float speedSq = math.lengthsq(v.LinearVelocity);

            if (distSq > SleepDistSq && speedSq < 0.01f && input.Throttle < 0.01f)
            {
                v.LodTier = 3;
                v.SubstepCount = 1;
                v.TierFlags |= VehicleTierFlags.IsSleeping;
            }
            else if (distSq < NearAiDistSq)
            {
                v.LodTier = 1;
                v.SubstepCount = NearAiSubsteps;
                v.TierFlags = (byte)(v.TierFlags & ~VehicleTierFlags.IsSleeping);
            }
            else
            {
                v.LodTier = 2;
                v.SubstepCount = FarAiSubsteps;
                v.TierFlags = (byte)(v.TierFlags & ~VehicleTierFlags.IsSleeping);
            }

            Vehicles[vehIdx] = v;
        }
    }
}
