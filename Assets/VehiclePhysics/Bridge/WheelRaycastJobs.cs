using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Core;

namespace VehiclePhysics.Bridge
{
    [BurstCompile]
    public struct PrepareWheelRaycastsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> ActiveWheelSlots;
        [ReadOnly] public NativeArray<int> ActiveWheelVehicleIndices;
        [ReadOnly] public NativeArray<int> ActiveWheelConfigIndices;

        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<VehicleConfig> VehicleConfigs;
        [ReadOnly] public NativeArray<WheelConfig> WheelConfigs;

        public PhysicsScene PhysicsScene;
        public QueryParameters QueryParams;
        public float LookaheadMarginM;

        [WriteOnly] public NativeArray<RaycastCommand> Commands;

        public void Execute(int cmdIdx)
        {
            int vehIdx = ActiveWheelVehicleIndices[cmdIdx];
            VehicleState veh = Vehicles[vehIdx];

            if (veh.LodTier >= 3 || (veh.TierFlags & VehicleTierFlags.IsSleeping) != 0)
            {
                QueryParameters skipParams = QueryParams;
                skipParams.layerMask = 0;
                Commands[cmdIdx] = new RaycastCommand(PhysicsScene, Vector3.zero, Vector3.down, skipParams, 0.01f);
                return;
            }

            VehicleConfig vCfg = VehicleConfigs[veh.ConfigIndex];
            WheelConfig wCfg = WheelConfigs[ActiveWheelConfigIndices[cmdIdx]];

            float3 mountRelCoM = wCfg.MountPositionLocal - vCfg.CenterOfMassLocalOffset;
            float3 worldMount = veh.Position + math.rotate(veh.Rotation, mountRelCoM);
            float3 worldStrutDir = math.normalizesafe(
                math.rotate(veh.Rotation, wCfg.StrutAxisLocal),
                new float3(0f, -1f, 0f));

            float rayLength = wCfg.RestLengthM + wCfg.UnloadedRadiusM + LookaheadMarginM;
            Commands[cmdIdx] = new RaycastCommand(PhysicsScene, worldMount, worldStrutDir, QueryParams, rayLength);
        }
    }

    [BurstCompile]
    public struct ProcessWheelHitsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> ActiveWheelSlots;
        [ReadOnly] public NativeArray<int> ActiveWheelVehicleIndices;
        [ReadOnly] public NativeArray<int> ActiveWheelConfigIndices;

        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<WheelConfig> WheelConfigs;
        [ReadOnly] public NativeArray<SurfaceMaterialData> SurfaceMaterials;
        [ReadOnly] public NativeArray<RaycastHit> Hits;

        [ReadOnly] public NativeHashMap<ColliderIdKey, ColliderPhysicsMeta>.ReadOnly ColliderMetaMap;
        [ReadOnly] public NativeArray<DynamicBodyMotion> DynamicBodies;

        public int MaxHitsPerRay;
        public float DefaultPatchLengthM;

        // Disjointness guaranteed by unique ActiveWheelSlots mapping
        [NativeDisableParallelForRestriction] public NativeArray<WheelContactSample> ContactSamples;
        [NativeDisableParallelForRestriction] public NativeArray<WheelContact> WheelContacts;
        [NativeDisableParallelForRestriction] public NativeArray<WheelInput> WheelInputs;

        public void Execute(int cmdIdx)
        {
            int wheelSlot = ActiveWheelSlots[cmdIdx];
            int vehIdx = ActiveWheelVehicleIndices[cmdIdx];
            VehicleState veh = Vehicles[vehIdx];
            WheelConfig wCfg = WheelConfigs[ActiveWheelConfigIndices[cmdIdx]];
            WheelInput wInput = WheelInputs[wheelSlot];
            int sampleSlot = wInput.ContactSampleStartIndex;

            if (veh.LodTier >= 3 || (veh.TierFlags & VehicleTierFlags.IsSleeping) != 0)
            {
                ContactSamples[sampleSlot] = default;
                WheelContacts[wheelSlot] = default;
                wInput.ContactSampleCount = 0;
                WheelInputs[wheelSlot] = wInput;
                return;
            }

            int baseHitIdx = cmdIdx * MaxHitsPerRay;
            bool hasValidHit = false;
            float bestDist = float.MaxValue;
            RaycastHit bestHit = default;
            ColliderPhysicsMeta bestMeta = new ColliderPhysicsMeta { DynamicBodyIndex = -1 };

            for (int h = 0; h < MaxHitsPerRay; h++)
            {
                RaycastHit hit = Hits[baseHitIdx + h];
                ColliderIdKey key = ColliderIdKey.FromHit(in hit);

                // First invalid hit marks the end of RaycastCommand results for this ray.
                if (!key.IsValid)
                    break;

                ColliderPhysicsMeta meta;
                if (ColliderMetaMap.TryGetValue(key, out meta))
                {
                    if ((meta.Flags & ColliderMetaFlags.IgnoreWheelRays) != 0)
                        continue;

                    if (meta.OwnerVehicleId != 0 && meta.OwnerVehicleId == veh.VehicleId)
                        continue;
                }
                else
                {
                    meta = new ColliderPhysicsMeta { DynamicBodyIndex = -1, SurfaceMaterialIndex = 0 };
                }

                if (hit.distance < bestDist)
                {
                    bestDist = hit.distance;
                    bestHit = hit;
                    bestMeta = meta;
                    hasValidHit = true;
                }
            }

            if (!hasValidHit)
            {
                ContactSamples[sampleSlot] = default;
                WheelContacts[wheelSlot] = default;
                wInput.ContactSampleCount = 0;
                WheelInputs[wheelSlot] = wInput;
                return;
            }

            int matIdx = math.clamp(bestMeta.SurfaceMaterialIndex, 0, SurfaceMaterials.Length - 1);
            SurfaceMaterialData mat = SurfaceMaterials[matIdx];

            float3 surfVel = float3.zero;
            if (bestMeta.DynamicBodyIndex >= 0 && bestMeta.DynamicBodyIndex < DynamicBodies.Length)
            {
                surfVel = DynamicBodies[bestMeta.DynamicBodyIndex].GetPointVelocity(bestHit.point);
            }

            float3 normal = math.normalizesafe((float3)bestHit.normal, new float3(0f, 1f, 0f));
            float maxReach = wCfg.RestLengthM + wCfg.UnloadedRadiusM;
            float pen = math.max(0f, maxReach - bestHit.distance);

            float effStaticMu = mat.StaticFriction * mat.TireGripMultiplier;
            float effDynamicMu = mat.DynamicFriction * mat.TireGripMultiplier;

            var sample = new WheelContactSample
            {
                Point = bestHit.point,
                PenetrationDepth = pen,
                Normal = normal,
                StaticFriction = effStaticMu,
                SurfaceVelocity = surfVel,
                DynamicFriction = effDynamicMu,
                RollingResistance = mat.RollingResistance,
                SampleWeight = 1.0f,
                SurfaceRoughness = mat.SurfaceRoughness,
                IsValid = 1,
                SurfaceMaterialType = mat.MaterialTypeId
            };
            ContactSamples[sampleSlot] = sample;

            WheelContacts[wheelSlot] = new WheelContact
            {
                PlanePoint = sample.Point,
                StaticFriction = sample.StaticFriction,
                PlaneNormal = sample.Normal,
                DynamicFriction = sample.DynamicFriction,
                SurfaceVelocity = sample.SurfaceVelocity,
                RollingResistance = sample.RollingResistance,
                ContactPatchLength = DefaultPatchLengthM,
                LateralCoPOffsetLocal = 0f,
                SurfaceRoughness = sample.SurfaceRoughness,
                HasContact = 1,
                SurfaceMaterialType = sample.SurfaceMaterialType
            };

            wInput.ContactSampleCount = 1;
            WheelInputs[wheelSlot] = wInput;
        }
    }
}
