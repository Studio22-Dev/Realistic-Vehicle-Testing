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
        [NativeDisableParallelForRestriction] public NativeArray<ContactManifold> ContactManifolds;
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
                ContactManifolds[wheelSlot] = default;
                wInput.ContactSampleCount = 0;
                WheelInputs[wheelSlot] = wInput;
                return;
            }

            int baseHitIdx = cmdIdx * MaxHitsPerRay;
            bool hasValidHit = false;
            float bestDist = float.MaxValue;
            RaycastHit bestHit = default;
            ColliderIdKey bestKey = default;
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
                    bestKey = key;
                    bestMeta = meta;
                    hasValidHit = true;
                }
            }

            if (!hasValidHit)
            {
                ContactSamples[sampleSlot] = default;
                ContactManifolds[wheelSlot] = default;
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

            bool isDynamic = bestMeta.DynamicBodyIndex >= 0;

            var c0 = new ManifoldContactPoint
            {
                WorldPoint = sample.Point,
                PenetrationM = pen,
                WorldNormal = sample.Normal,
                ContactForceN = 0f,
                SurfaceVelocity = sample.SurfaceVelocity,
                ColliderInstanceID = bestKey.GetHashCode(),
                PendingObjectImpulse = float3.zero,
                DynamicBodyIndex = bestMeta.DynamicBodyIndex,
                ContactWeight = 1.0f,
                LocalFluidOrLooseDepthM = 0f,
                FrictionScale = 1.0f,
                SurfaceMaterialIndex = (byte)matIdx,
                IsValid = 1,
                SampleCount = 1,
                ClusterFlags = isDynamic ? ManifoldClusterFlags.HitDynamicBody : ManifoldClusterFlags.None
            };

            ContactManifolds[wheelSlot] = new ContactManifold
            {
                ClusterBottom = c0,
                BlendedPoint = sample.Point,
                ContactPatchLengthM = DefaultPatchLengthM,
                BlendedNormal = sample.Normal,
                WaterDepthM = 0f,
                BlendedVelocity = sample.SurfaceVelocity,
                TerrainSinkageM = 0f,
                HydroplaningFactor = 0f,
                BulldozingDragN = 0f,
                BlendedFrictionScale = 1.0f,
                ActiveClusterMask = 1 << 0,
                PrimaryMaterialIndex = (byte)matIdx,
                IsGrounded = 1,
                Flags = isDynamic ? ManifoldFlags.TouchesDynamicBody : ManifoldFlags.None
            };

            wInput.ContactSampleCount = 1;
            WheelInputs[wheelSlot] = wInput;
        }
    }
}
