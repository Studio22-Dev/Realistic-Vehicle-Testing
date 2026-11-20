using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

namespace VehiclePhysics.Runtime
{
    using Core;

    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct ChassisTransformSyncJob : IJobParallelForTransform
    {
        public float Alpha;
        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<VehicleConfig> VehicleConfigs;
        [ReadOnly] public NativeArray<VehicleRenderPose> RenderPoses;

        public void Execute(int vehIdx, TransformAccess transform)
        {
            if ((uint)vehIdx >= (uint)Vehicles.Length) return;

            VehicleState veh = Vehicles[vehIdx];
            VehicleConfig vCfg = VehicleConfigs[veh.ConfigIndex];
            VehicleRenderPose pose = RenderPoses[vehIdx];

            float3 interpComPos = math.lerp(pose.PreviousPosition, pose.CurrentPosition, Alpha);
            quaternion interpRot = math.slerp(pose.PreviousRotation, pose.CurrentRotation, Alpha);
            float3 meshOriginPos = interpComPos - math.rotate(interpRot, vCfg.CenterOfMassLocalOffset);

            transform.position = (Vector3)meshOriginPos;
            transform.rotation = (Quaternion)interpRot;
        }
    }

    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct WheelTransformSyncJob : IJobParallelForTransform
    {
        [ReadOnly] public NativeArray<int2> WheelVisualMap; // .x = vehIdx, .y = localWheelIdx
        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<VehicleConfig> VehicleConfigs;
        [ReadOnly] public NativeArray<WheelConfig> WheelConfigs;
        [ReadOnly] public NativeArray<WheelState> Wheels;

        public void Execute(int visualIdx, TransformAccess transform)
        {
            if ((uint)visualIdx >= (uint)WheelVisualMap.Length) return;

            int2 map = WheelVisualMap[visualIdx];
            int vehIdx = map.x;
            int localWheelIdx = map.y;

            if ((uint)vehIdx >= (uint)Vehicles.Length) return;

            VehicleState veh = Vehicles[vehIdx];
            if ((uint)localWheelIdx >= (uint)veh.WheelCount) return;

            int wheelSlot = veh.WheelStartIndex + localWheelIdx;
            if ((uint)wheelSlot >= (uint)Wheels.Length) return;

            WheelState ws = Wheels[wheelSlot];
            if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0) return;

            VehicleConfig vCfg = VehicleConfigs[veh.ConfigIndex];
            int cfgSlot = vCfg.WheelConfigStartIndex + localWheelIdx;
            if ((uint)cfgSlot >= (uint)WheelConfigs.Length) return;

            WheelConfig wCfg = WheelConfigs[cfgSlot];

            float3 hubLocal = wCfg.MountPositionLocal
                            + wCfg.StrutAxisLocal * (wCfg.RestLengthM - ws.SuspensionCompression);
            quaternion wheelLocalRot = math.mul(
                quaternion.RotateY(ws.SteerAngleRad),
                quaternion.RotateX(ws.SpinAngle));

            transform.localPosition = (Vector3)hubLocal;
            transform.localRotation = (Quaternion)wheelLocalRot;
        }
    }
}
