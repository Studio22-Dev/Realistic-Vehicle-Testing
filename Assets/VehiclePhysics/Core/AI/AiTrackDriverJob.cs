using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct AiTrackDriverJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<VehicleConfig> VehicleConfigs;
        [ReadOnly] public NativeArray<float3> Waypoints;

        public NativeArray<int> CurrentWaypointIndices;
        public NativeArray<VehicleDriverInput> DriverInputs;

        public float MaxAiSpeedMs;
        public float MinCornerSpeedMs;

        public void Execute(int vehIdx)
        {
            VehicleState veh = Vehicles[vehIdx];
            if ((veh.TierFlags & VehicleTierFlags.IsPlayerTier) != 0) return;

            int wpCount = Waypoints.Length;
            if (wpCount < 2) return;

            int wpIdx = CurrentWaypointIndices[vehIdx] % wpCount;
            if (wpIdx < 0) wpIdx += wpCount;

            float speed = math.length(veh.LinearVelocity);
            float advanceRadius = 11.0f + speed * 0.35f;

            // Advance waypoint when within lookahead radius
            float3 target = Waypoints[wpIdx];
            float3 flatDelta = target - veh.Position;
            flatDelta.y = 0f;
            if (math.lengthsq(flatDelta) < advanceRadius * advanceRadius)
            {
                wpIdx = (wpIdx + 1) % wpCount;
                CurrentWaypointIndices[vehIdx] = wpIdx;
                target = Waypoints[wpIdx];
            }

            // Look one waypoint ahead to gauge corner severity for braking
            float3 nextTarget = Waypoints[(wpIdx + 1) % wpCount];
            float3 segA = math.normalizesafe(target - veh.Position, new float3(0f, 0f, 1f));
            float3 segB = math.normalizesafe(nextTarget - target, segA);
            segA.y = 0f;
            segB.y = 0f;
            float cornerDot = math.dot(math.normalizesafe(segA), math.normalizesafe(segB));
            float turnSeverity = math.saturate((0.96f - cornerDot) * 1.4f);

            // Transform target into vehicle-local horizontal space
            float3 localDir = math.rotate(math.inverse(veh.Rotation), target - veh.Position);
            float steerAngleNeeded = math.atan2(localDir.x, math.max(localDir.z, 0.5f));
            if (localDir.z < 0f)
                steerAngleNeeded = math.select(-1.0f, 1.0f, localDir.x >= 0f) * 1.0f;

            VehicleConfig vCfg = VehicleConfigs[veh.ConfigIndex];
            float maxSteer = math.max(vCfg.MaxSteerAngleRad, 0.1f);
            float steerCmd = math.clamp(steerAngleNeeded / maxSteer, -1.0f, 1.0f);

            float targetSpeed = math.lerp(MaxAiSpeedMs, MinCornerSpeedMs, turnSeverity);
            // Reduce target speed if sharply misaligned
            if (math.abs(steerCmd) > 0.65f)
                targetSpeed = math.min(targetSpeed, MinCornerSpeedMs * 1.15f);

            float speedErr = targetSpeed - veh.ForwardSpeedMs;
            float throttle = speedErr > 0.25f ? math.saturate(speedErr * 0.25f) : 0f;
            float brake = speedErr < -1.5f ? math.saturate((-speedErr - 1.5f) * 0.20f) : 0f;

            DriverInputs[vehIdx] = new VehicleDriverInput
            {
                Throttle = throttle,
                Brake = brake,
                Steer = steerCmd,
                ClutchPedal = 0f,
                Handbrake = 0f,
                RequestedGear = -128, // Auto-transmission
                ShiftDelta = 0,
                InputFlags = 0
            };
        }
    }
}
