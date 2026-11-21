using System;
using System.Collections.Generic;
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
        public const int MaxCapsulesPerWheel = 9;
        public const int MaxSpheresPerWheel  = 2;

        public readonly int MaxWheelCommands;
        public readonly int MaxHitsPerCast;

        private int _mappedVehicleCount = -1;
        private bool _wheelMapDirty = true;
        private bool _topologyDirty = true;
        private int _lastTopologyVersion = -1;

        public void MarkWheelMapDirty() => _wheelMapDirty = true;
        public void MarkTopologyDirty() => _topologyDirty = true;

        public int ActiveCommandCount { get; private set; }
        public bool IsDisposed { get; private set; }

        public NativeArray<CapsulecastCommand> CapsuleCommands;
        public NativeArray<RaycastHit>         CapsuleHits;

        public NativeArray<SpherecastCommand>  SphereCommands;
        public NativeArray<RaycastHit>         SphereHits;

        public NativeArray<WheelRayMeta>       WheelMeta;

        public NativeArray<int> ActiveWheelSlots;
        public NativeArray<int> ActiveWheelVehicleIndices;
        public NativeArray<int> ActiveWheelConfigIndices;

        public NativeHashMap<ColliderIdKey, ColliderPhysicsMeta> ColliderMetaMap;
        public NativeList<DynamicBodyMotion> DynamicBodies;
        private readonly List<Rigidbody> _dynamicRigidbodies = new List<Rigidbody>(64);

        public VehicleRaycastPipeline(
            int maxWheelSlots,
            int maxHitsPerRay = 1,
            int initialColliderCapacity = 256)
        {
            MaxWheelCommands = math.max(1, maxWheelSlots);
            MaxHitsPerCast = math.max(1, maxHitsPerRay);

            CapsuleCommands = new NativeArray<CapsulecastCommand>(
                MaxWheelCommands * MaxCapsulesPerWheel, Allocator.Persistent);
            CapsuleHits = new NativeArray<RaycastHit>(
                MaxWheelCommands * MaxCapsulesPerWheel * MaxHitsPerCast, Allocator.Persistent);

            SphereCommands = new NativeArray<SpherecastCommand>(
                MaxWheelCommands * MaxSpheresPerWheel, Allocator.Persistent);
            SphereHits = new NativeArray<RaycastHit>(
                MaxWheelCommands * MaxSpheresPerWheel * MaxHitsPerCast, Allocator.Persistent);

            WheelMeta = new NativeArray<WheelRayMeta>(MaxWheelCommands, Allocator.Persistent);

            ActiveWheelSlots = new NativeArray<int>(MaxWheelCommands, Allocator.Persistent);
            ActiveWheelVehicleIndices = new NativeArray<int>(MaxWheelCommands, Allocator.Persistent);
            ActiveWheelConfigIndices = new NativeArray<int>(MaxWheelCommands, Allocator.Persistent);

            ColliderMetaMap = new NativeHashMap<ColliderIdKey, ColliderPhysicsMeta>(
                math.max(16, initialColliderCapacity), Allocator.Persistent);
            DynamicBodies = new NativeList<DynamicBodyMotion>(64, Allocator.Persistent);
        }

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
            => AddOrUpdateDynamicBody(bodyIndex, in motion, null);

        public int AddOrUpdateDynamicBody(int bodyIndex, in DynamicBodyMotion motion, Rigidbody rb)
        {
            if (bodyIndex < 0)
            {
                DynamicBodies.Add(motion);
                _dynamicRigidbodies.Add(rb);
                return DynamicBodies.Length - 1;
            }

            DynamicBodies[bodyIndex] = motion;
            if (bodyIndex < _dynamicRigidbodies.Count && rb != null)
                _dynamicRigidbodies[bodyIndex] = rb;
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

                    if ((world.Wheels[slot].WheelFlags & WheelFlagBits.IsDetached) != 0)
                    {
                        world.ContactManifolds[slot] = default;
                        WheelInput wIn = world.WheelInputs[slot];
                        wIn.ContactSampleCount = 0;
                        world.WheelInputs[slot] = wIn;

                        TireCarcassState carcass = world.TireCarcassStates[slot];
                        carcass.ActiveRawProbeCount = 0;
                        world.TireCarcassStates[slot] = carcass;
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
                Wheels = world.Wheels,
                PhysicsScene = physicsScene,
                QueryParams = queryParams,
                LookaheadMarginM = lookaheadMarginM,
                CapsuleCommands = CapsuleCommands,
                SphereCommands = SphereCommands,
                WheelMeta = WheelMeta
            };

            JobHandle prepareHandle = prepareJob.Schedule(ActiveCommandCount, 4, dependsOn);

            int workerCount = math.max(1, JobsUtility.JobWorkerCount);
            int totalCapsules = ActiveCommandCount * MaxCapsulesPerWheel;
            int totalSpheres = ActiveCommandCount * MaxSpheresPerWheel;

            int minCapPerJob = math.max(1, totalCapsules / workerCount);
            int minSphPerJob = math.max(1, totalSpheres / workerCount);

            JobHandle capHandle = CapsulecastCommand.ScheduleBatch(
                CapsuleCommands.GetSubArray(0, totalCapsules),
                CapsuleHits.GetSubArray(0, totalCapsules * MaxHitsPerCast),
                minCapPerJob,
                MaxHitsPerCast,
                prepareHandle);

            JobHandle sphHandle = SpherecastCommand.ScheduleBatch(
                SphereCommands.GetSubArray(0, totalSpheres),
                SphereHits.GetSubArray(0, totalSpheres *MaxHitsPerCast),
                minSphPerJob,
                MaxHitsPerCast,
                prepareHandle);

            JobHandle castsHandle = JobHandle.CombineDependencies(capHandle, sphHandle);

            var processJob = new ProcessWheelHitsJob
            {
                ActiveWheelSlots = ActiveWheelSlots,
                ActiveWheelVehicleIndices = ActiveWheelVehicleIndices,
                ActiveWheelConfigIndices = ActiveWheelConfigIndices,
                Vehicles = world.Vehicles,
                WheelConfigs = world.WheelConfigs,
                SurfaceMaterials = world.SurfaceMaterials,
                WheelMeta = WheelMeta,
                CapsuleHits = CapsuleHits,
                SphereHits = SphereHits,
                MaxHitsPerCast = MaxHitsPerCast,
                ColliderMetaMap = ColliderMetaMap.AsReadOnly(),
                DynamicBodies = DynamicBodies.AsArray(),
                LookaheadMarginM = lookaheadMarginM,
                DefaultPatchLengthM = 0.18f,
                MaxSamplesPerWheel = world.MaxSamplesPerWheel,
                ContactSamples = world.ContactSamples,
                ContactManifolds = world.ContactManifolds,
                TireCarcassStates = world.TireCarcassStates,
                WheelInputs = world.WheelInputs
            };

            return processJob.Schedule(ActiveCommandCount, 4, castsHandle);
        }

        /// <summary>
        /// Drains post-step tire impulses onto registered dynamic Unity Rigidbodies safely on main thread.
        /// Call immediately after simulation JobHandle.Complete(). Zero GC allocations.
        /// </summary>
        public void DrainPendingObjectImpulses(VehicleSimulationWorld world, float maxDeltaVPerStep = 8f)
        {
            if (world == null || world.IsDisposed || _dynamicRigidbodies.Count == 0)
                return;

            int activeCount = world.ActiveVehicleCount;
            for (int v = 0; v < activeCount; v++)
            {
                VehicleState veh = world.Vehicles[v];
                for (int w = 0; w < veh.WheelCount; w++)
                {
                    int slot = veh.WheelStartIndex + w;
                    if ((world.Wheels[slot].WheelFlags & WheelFlagBits.IsDetached) != 0)
                        continue;

                    ContactManifold manifold = world.ContactManifolds[slot];
                    if ((manifold.Flags & ManifoldFlags.TouchesDynamicBody) == 0)
                        continue;

                    DrainCluster(ref manifold.ClusterBottom, maxDeltaVPerStep);
                    DrainCluster(ref manifold.ClusterLeading, maxDeltaVPerStep);
                    DrainCluster(ref manifold.ClusterTrailing, maxDeltaVPerStep);
                    DrainCluster(ref manifold.ClusterSidewall, maxDeltaVPerStep);
                    world.ContactManifolds[slot] = manifold;
                }
            }
        }

        private void DrainCluster(ref ManifoldContactPoint cluster, float maxDeltaV)
        {
            if (cluster.IsValid == 0 || cluster.DynamicBodyIndex < 0)
                return;

            float3 imp = cluster.PendingObjectImpulse;
            cluster.PendingObjectImpulse = float3.zero;

            float impSq = math.lengthsq(imp);
            if (impSq <= 1e-10f || cluster.DynamicBodyIndex >= _dynamicRigidbodies.Count)
                return;

            Rigidbody rb = _dynamicRigidbodies[cluster.DynamicBodyIndex];
            if (rb == null || rb.isKinematic)
                return;

            float maxImp = rb.mass * maxDeltaV;
            float impMag = math.sqrt(impSq);
            if (impMag > maxImp)
                imp *= maxImp / impMag;

            rb.AddForceAtPosition(imp, cluster.WorldPoint, ForceMode.Impulse);
        }

        public void Dispose()
        {
            if (IsDisposed) return;
            IsDisposed = true;

            if (CapsuleCommands.IsCreated) CapsuleCommands.Dispose();
            if (CapsuleHits.IsCreated) CapsuleHits.Dispose();
            if (SphereCommands.IsCreated) SphereCommands.Dispose();
            if (SphereHits.IsCreated) SphereHits.Dispose();
            if (WheelMeta.IsCreated) WheelMeta.Dispose();
            if (ActiveWheelSlots.IsCreated) ActiveWheelSlots.Dispose();
            if (ActiveWheelVehicleIndices.IsCreated) ActiveWheelVehicleIndices.Dispose();
            if (ActiveWheelConfigIndices.IsCreated) ActiveWheelConfigIndices.Dispose();
            if (ColliderMetaMap.IsCreated) ColliderMetaMap.Dispose();
            if (DynamicBodies.IsCreated) DynamicBodies.Dispose();
            _dynamicRigidbodies.Clear();
        }
    }
}
