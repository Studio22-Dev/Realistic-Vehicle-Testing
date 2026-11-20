using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Core;

namespace VehiclePhysics.Bridge
{
    public sealed class VehicleRaycastPipeline : IDisposable
    {
        public readonly int MaxWheelCommands;
        public readonly int MaxHitsPerRay;

        private int _mappedVehicleCount = -1;
        private bool _wheelMapDirty = true;

        public void MarkWheelMapDirty() => _wheelMapDirty = true;

        public int ActiveCommandCount { get; private set; }
        public bool IsDisposed { get; private set; }

        public NativeArray<RaycastCommand> Commands;
        public NativeArray<RaycastHit> Hits;

        public NativeArray<int> ActiveWheelSlots;
        public NativeArray<int> ActiveWheelVehicleIndices;
        public NativeArray<int> ActiveWheelConfigIndices;

        public NativeHashMap<ColliderIdKey, ColliderPhysicsMeta> ColliderMetaMap;
        public NativeList<DynamicBodyMotion> DynamicBodies;

        private bool _topologyDirty = true;
        private int _lastTopologyVersion = -1;

        public VehicleRaycastPipeline(
            int maxWheelSlots,
            int maxHitsPerRay = 3,
            int initialColliderCapacity = 256)
        {
            MaxWheelCommands = math.max(1, maxWheelSlots);
            MaxHitsPerRay = math.max(1, maxHitsPerRay);

            Commands = new NativeArray<RaycastCommand>(MaxWheelCommands, Allocator.Persistent);
            Hits = new NativeArray<RaycastHit>(MaxWheelCommands * MaxHitsPerRay, Allocator.Persistent);

            ActiveWheelSlots = new NativeArray<int>(MaxWheelCommands, Allocator.Persistent);
            ActiveWheelVehicleIndices = new NativeArray<int>(MaxWheelCommands, Allocator.Persistent);
            ActiveWheelConfigIndices = new NativeArray<int>(MaxWheelCommands, Allocator.Persistent);

            ColliderMetaMap = new NativeHashMap<ColliderIdKey, ColliderPhysicsMeta>(
                math.max(16, initialColliderCapacity), Allocator.Persistent);
            DynamicBodies = new NativeList<DynamicBodyMotion>(64, Allocator.Persistent);
        }

        public void MarkTopologyDirty() => _topologyDirty = true;

        public void RegisterCollider(Collider collider, ColliderPhysicsMeta meta)
        {
            ColliderIdKey key = ColliderIdKey.FromObject(collider);
            if (!key.IsValid) return;
            ColliderMetaMap[key] = meta;
        }

        public void UnregisterCollider(Collider collider)
        {
            ColliderIdKey key = ColliderIdKey.FromObject(collider);
            if (!key.IsValid) return;
            ColliderMetaMap.Remove(key);
        }

        public int AddOrUpdateDynamicBody(int bodyIndex, in DynamicBodyMotion motion)
        {
            if (bodyIndex < 0)
            {
                DynamicBodies.Add(motion);
                return DynamicBodies.Length - 1;
            }
            DynamicBodies[bodyIndex] = motion;
            return bodyIndex;
        }

        public void RebuildActiveWheelMap(VehicleSimulationWorld world)
        {
            int cmdCursor = 0;
            int vehCount = world.ActiveVehicleCount;

            for (int v = 0; v < vehCount; v++)
            {
                VehicleState veh = world.Vehicles[v];
                VehicleConfig vCfg = world.VehicleConfigs[veh.ConfigIndex];

                for (int w = 0; w < veh.WheelCount; w++)
                {
                    int slot = veh.WheelStartIndex + w;

                    // Omit detached wheels from raycasts and ensure stale manifold data is cleared
                    if ((world.Wheels[slot].WheelFlags & WheelFlagBits.IsDetached) != 0)
                    {
                        world.ContactManifolds[slot] = default;
                        WheelInput wIn = world.WheelInputs[slot];
                        wIn.ContactSampleCount = 0;
                        world.WheelInputs[slot] = wIn;
                        continue;
                    }

                    if (cmdCursor >= MaxWheelCommands)
                        throw new InvalidOperationException("VehicleRaycastPipeline command buffer capacity exceeded.");

                    ActiveWheelSlots[cmdCursor] = slot;
                    ActiveWheelVehicleIndices[cmdCursor] = v;
                    ActiveWheelConfigIndices[cmdCursor] = vCfg.WheelConfigStartIndex + w;
                    cmdCursor++;
                }
            }

            ActiveCommandCount = cmdCursor;
            _lastTopologyVersion = world.TopologyVersion;
            _mappedVehicleCount = vehCount;
            _wheelMapDirty = false;
            _topologyDirty = false;
        }

        public JobHandle ScheduleQueries(
            VehicleSimulationWorld world,
            PhysicsScene physicsScene,
            LayerMask layerMask,
            JobHandle dependsOn = default,
            float lookaheadMarginM = 0.02f)
        {
            if (_wheelMapDirty
                || _topologyDirty
                || _mappedVehicleCount != world.ActiveVehicleCount
                || _lastTopologyVersion != world.TopologyVersion)
            {
                dependsOn.Complete();
                RebuildActiveWheelMap(world);
                dependsOn = default;
            }

            if (ActiveCommandCount == 0)
                return dependsOn;

            var queryParams = new QueryParameters(
                layerMask,
                hitMultipleFaces: false,
                hitTriggers: QueryTriggerInteraction.Ignore,
                hitBackfaces: false);

            var prepareJob = new PrepareWheelRaycastsJob
            {
                ActiveWheelSlots = ActiveWheelSlots,
                ActiveWheelVehicleIndices = ActiveWheelVehicleIndices,
                ActiveWheelConfigIndices = ActiveWheelConfigIndices,
                Vehicles = world.Vehicles,
                VehicleConfigs = world.VehicleConfigs,
                WheelConfigs = world.WheelConfigs,
                PhysicsScene = physicsScene,
                QueryParams = queryParams,
                LookaheadMarginM = lookaheadMarginM,
                Commands = Commands
            };

            JobHandle prepareHandle = prepareJob.Schedule(ActiveCommandCount, 4, dependsOn);

            int workerCount = math.max(1, JobsUtility.JobWorkerCount);
            int minCommandsPerJob = math.max(1, ActiveCommandCount / workerCount);

            JobHandle raycastHandle = RaycastCommand.ScheduleBatch(
                Commands.GetSubArray(0, ActiveCommandCount),
                Hits.GetSubArray(0, ActiveCommandCount * MaxHitsPerRay),
                minCommandsPerJob,
                MaxHitsPerRay,
                prepareHandle);

            var processJob = new ProcessWheelHitsJob
            {
                ActiveWheelSlots = ActiveWheelSlots,
                ActiveWheelVehicleIndices = ActiveWheelVehicleIndices,
                ActiveWheelConfigIndices = ActiveWheelConfigIndices,
                Vehicles = world.Vehicles,
                WheelConfigs = world.WheelConfigs,
                SurfaceMaterials = world.SurfaceMaterials,
                Hits = Hits,
                ColliderMetaMap = ColliderMetaMap.AsReadOnly(),
                DynamicBodies = DynamicBodies.AsArray(),
                MaxHitsPerRay = MaxHitsPerRay,
                DefaultPatchLengthM = 0.18f,
                ContactSamples = world.ContactSamples,
                ContactManifolds = world.ContactManifolds,
                WheelInputs = world.WheelInputs
            };

            return processJob.Schedule(ActiveCommandCount, 4, raycastHandle);
        }

        public void Dispose()
        {
            if (IsDisposed) return;
            IsDisposed = true;

            if (Commands.IsCreated) Commands.Dispose();
            if (Hits.IsCreated) Hits.Dispose();
            if (ActiveWheelSlots.IsCreated) ActiveWheelSlots.Dispose();
            if (ActiveWheelVehicleIndices.IsCreated) ActiveWheelVehicleIndices.Dispose();
            if (ActiveWheelConfigIndices.IsCreated) ActiveWheelConfigIndices.Dispose();
            if (ColliderMetaMap.IsCreated) ColliderMetaMap.Dispose();
            if (DynamicBodies.IsCreated) DynamicBodies.Dispose();
        }
    }
}
