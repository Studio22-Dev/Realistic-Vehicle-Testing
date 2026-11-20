using System;
using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public sealed class VehicleSimulationWorld : IDisposable
    {
        // 4 * 80B (WheelState) = 320B = 5 x 64B cache lines.
        // Rounding each vehicle's wheel allocation up to a multiple of 4 prevents false sharing.
        public const int WheelCacheAlignmentStride = 4;

        public readonly int MaxVehicles;
        public readonly int MaxTotalWheelSlots;
        public readonly int MaxArchetypes;
        public readonly int MaxArchetypeWheelConfigs;
        public readonly int MaxSamplesPerWheel;

        public int ActiveVehicleCount { get; private set; }
        public int ActiveWheelSlotCursor { get; private set; }
        public int ActiveConfigWheelCursor { get; private set; }
        public int TopologyVersion { get; private set; }
        public bool IsDisposed { get; private set; }

        // Mutable Per-Vehicle State
        public NativeArray<VehicleState> Vehicles;
        public NativeArray<PowertrainState> Powertrains;
        public NativeArray<VehicleDriverInput> DriverInputs;
        public NativeArray<VehicleRenderPose> RenderPoses;

        // Mutable Per-Wheel State (indexed via VehicleState.WheelStartIndex + w)
        public NativeArray<WheelState> Wheels;
        public NativeArray<WheelInput> WheelInputs;
        public NativeArray<WheelContact> WheelContacts;
        public NativeArray<WheelContactSample> ContactSamples;

        // Read-Only Archetype Configs & Surface Table
        public NativeArray<VehicleConfig> VehicleConfigs;
        public NativeArray<PowertrainConfig> PowertrainConfigs;
        public NativeArray<WheelConfig> WheelConfigs;
        public NativeArray<SurfaceMaterialData> SurfaceMaterials;

        public VehicleSimulationWorld(
            int maxVehicles,
            int maxArchetypes,
            int maxSurfaceMaterials,
            int maxSamplesPerWheel = 1)
            : this(
                maxVehicles,
                maxVehicles * WheelCacheAlignmentStride,
                maxArchetypes,
                maxArchetypes * WheelCacheAlignmentStride,
                maxSurfaceMaterials,
                maxSamplesPerWheel)
        {
        }

        public VehicleSimulationWorld(
            int maxVehicles,
            int maxTotalWheelSlots,
            int maxArchetypes,
            int maxArchetypeWheelConfigs,
            int maxSurfaceMaterials,
            int maxSamplesPerWheel = 1)
        {
            MaxVehicles = math.max(1, maxVehicles);
            MaxTotalWheelSlots = AlignWheelCount(math.max(WheelCacheAlignmentStride, maxTotalWheelSlots));
            MaxArchetypes = math.max(1, maxArchetypes);
            MaxArchetypeWheelConfigs = math.max(1, maxArchetypeWheelConfigs);
            MaxSamplesPerWheel = math.max(1, maxSamplesPerWheel);

            int surfaceCount = math.max(1, maxSurfaceMaterials);

            Vehicles = new NativeArray<VehicleState>(MaxVehicles, Allocator.Persistent);
            Powertrains = new NativeArray<PowertrainState>(MaxVehicles, Allocator.Persistent);
            DriverInputs = new NativeArray<VehicleDriverInput>(MaxVehicles, Allocator.Persistent);
            RenderPoses = new NativeArray<VehicleRenderPose>(MaxVehicles, Allocator.Persistent);

            Wheels = new NativeArray<WheelState>(MaxTotalWheelSlots, Allocator.Persistent);
            WheelInputs = new NativeArray<WheelInput>(MaxTotalWheelSlots, Allocator.Persistent);
            WheelContacts = new NativeArray<WheelContact>(MaxTotalWheelSlots, Allocator.Persistent);
            ContactSamples = new NativeArray<WheelContactSample>(
                MaxTotalWheelSlots * MaxSamplesPerWheel, Allocator.Persistent);

            VehicleConfigs = new NativeArray<VehicleConfig>(MaxArchetypes, Allocator.Persistent);
            PowertrainConfigs = new NativeArray<PowertrainConfig>(MaxArchetypes, Allocator.Persistent);
            WheelConfigs = new NativeArray<WheelConfig>(MaxArchetypeWheelConfigs, Allocator.Persistent);
            SurfaceMaterials = new NativeArray<SurfaceMaterialData>(surfaceCount, Allocator.Persistent);
        }

        public static int AlignWheelCount(int wheelCount)
        {
            return (wheelCount + (WheelCacheAlignmentStride - 1)) / WheelCacheAlignmentStride
                   * WheelCacheAlignmentStride;
        }

        public int RegisterArchetype(
            int archetypeIndex,
            VehicleConfig vehicleCfg,
            in PowertrainConfig powertrainCfg,
            ReadOnlySpan<WheelConfig> wheelConfigs)
        {
            if (archetypeIndex < 0 || archetypeIndex >= MaxArchetypes)
                throw new ArgumentOutOfRangeException(nameof(archetypeIndex));
            if (wheelConfigs.Length <= 0)
                throw new ArgumentException("Archetype must have at least 1 wheel.", nameof(wheelConfigs));
            if (ActiveConfigWheelCursor + wheelConfigs.Length > MaxArchetypeWheelConfigs)
                throw new InvalidOperationException("WheelConfigs capacity exceeded.");

            int startIdx = ActiveConfigWheelCursor;
            for (int i = 0; i < wheelConfigs.Length; i++)
            {
                WheelConfigs[startIdx + i] = wheelConfigs[i];
            }
            ActiveConfigWheelCursor += wheelConfigs.Length;

            vehicleCfg.WheelConfigStartIndex = startIdx;
            vehicleCfg.WheelCount = wheelConfigs.Length;

            VehicleConfigs[archetypeIndex] = vehicleCfg;
            PowertrainConfigs[archetypeIndex] = powertrainCfg;
            return archetypeIndex;
        }

        /// <summary>
        /// Spawns a vehicle (car, tractor cab, or trailer) into the next aligned slot.
        /// Spawn the Player vehicle first (index 0) to avoid multi-core tail latency stalls.
        /// </summary>
        public int SpawnVehicle(
            uint vehicleId,
            int configIndex,
            float3 spawnPos,
            quaternion spawnRot,
            bool isPlayerTier,
            ushort substepCount)
        {
            if (ActiveVehicleCount >= MaxVehicles)
                throw new InvalidOperationException("VehicleSimulationWorld vehicle capacity exceeded.");
            if (configIndex < 0 || configIndex >= MaxArchetypes)
                throw new ArgumentOutOfRangeException(nameof(configIndex));

            VehicleConfig cfg = VehicleConfigs[configIndex];
            PowertrainConfig ptCfg = PowertrainConfigs[configIndex];

            int wheelCount = cfg.WheelCount;
            int alignedStride = AlignWheelCount(wheelCount);
            if (ActiveWheelSlotCursor + alignedStride > MaxTotalWheelSlots)
                throw new InvalidOperationException("VehicleSimulationWorld wheel slot capacity exceeded.");

            int idx = ActiveVehicleCount++;
            int wheelStart = ActiveWheelSlotCursor;
            ActiveWheelSlotCursor += alignedStride;

            quaternion normalizedRot = math.normalize(spawnRot);

            Vehicles[idx] = new VehicleState
            {
                Rotation = normalizedRot,
                Position = spawnPos,
                InverseMass = 1.0f / math.max(cfg.MassKg, 1.0f),
                LinearVelocity = float3.zero,
                ForwardSpeedMs = 0f,
                AngularVelocity = float3.zero,
                WheelStartIndex = wheelStart,
                WheelCount = wheelCount,
                InverseInertiaLocal = 1.0f / math.max(cfg.InertiaLocal, new float3(1.0f)),
                ExternalImpulse = float3.zero,
                VehicleId = vehicleId,
                ExternalAngularImpulse = float3.zero,
                ConfigIndex = configIndex,
                LocalAcceleration = float3.zero,
                SubstepCount = substepCount,
                TierFlags = isPlayerTier ? VehicleTierFlags.IsPlayerTier : VehicleTierFlags.None,
                LodTier = isPlayerTier ? (byte)0 : (byte)2
            };

            RenderPoses[idx] = new VehicleRenderPose
            {
                PreviousRotation = normalizedRot,
                PreviousPosition = spawnPos,
                CurrentRotation = normalizedRot,
                CurrentPosition = spawnPos
            };

            bool hasPowertrain = cfg.HasPowertrain != 0;
            Powertrains[idx] = new PowertrainState
            {
                EngineOmega = hasPowertrain ? ptCfg.IdleOmega : 0f,
                GearboxInputOmega = 0f,
                DriveshaftOmega = 0f,
                EffectiveGearRatio = 0f,
                CurrentGear = 0,
                TargetGear = 0,
                ClutchEngagement = 0f,
                ClutchLockState = ClutchLockStates.Open,
                DrivelineFlags = hasPowertrain ? DrivelineFlagBits.AutoClutch : DrivelineFlagBits.None
            };

            DriverInputs[idx] = new VehicleDriverInput { RequestedGear = -128 };

            int cfgWheelBase = cfg.WheelConfigStartIndex;
            for (int w = 0; w < wheelCount; w++)
            {
                int slot = wheelStart + w;
                WheelConfig wCfg = WheelConfigs[cfgWheelBase + w];

                Wheels[slot] = new WheelState
                {
                    EffectiveRollingRadius = wCfg.UnloadedRadiusM,
                    AxleIndex = wCfg.AxleIndex,
                    SideIndex = wCfg.SideIndex
                };

                WheelInputs[slot] = new WheelInput
                {
                    ContactSampleStartIndex = slot * MaxSamplesPerWheel,
                    ContactSampleCount = 0,
                    ControlFlags = wCfg.DefaultControlFlags,
                    PatchMode = 0
                };

                WheelContacts[slot] = default;
            }

            TopologyVersion++;
            return idx;
        }

        /// <summary>
        /// Detaches or reattaches/repairs a wheel slot safely in place without shifting array indices.
        /// Automatically clears contacts, resets spin/deflection on repair, updates vehicle mass, and flags raycast topology dirty.
        /// </summary>
        public void SetWheelDetached(int vehicleIndex, int localWheelIndex, bool detached, float wheelUnsprungMassKg = 22f)
        {
            if (IsDisposed) return;
            if ((uint)vehicleIndex >= (uint)ActiveVehicleCount) return;

            VehicleState veh = Vehicles[vehicleIndex];
            if ((uint)localWheelIndex >= (uint)veh.WheelCount) return;

            int slot = veh.WheelStartIndex + localWheelIndex;
            WheelState ws = Wheels[slot];
            bool wasDetached = (ws.WheelFlags & WheelFlagBits.IsDetached) != 0;
            if (wasDetached == detached) return;

            if (detached)
            {
                ws.WheelFlags |= WheelFlagBits.IsDetached;
                ws.WheelFlags = (byte)(ws.WheelFlags & ~(WheelFlagBits.IsGrounded | WheelFlagBits.IsBrakeLocked | WheelFlagBits.AbsActive | WheelFlagBits.TcsActive));
                ws.AngularVelocity = 0f;
                ws.SuspensionCompression = 0f;
                ws.SuspensionVelocity = 0f;
                ws.GroundLoadN = 0f;
                ws.LongitudinalDeflection = 0f;
                ws.LateralDeflection = 0f;
                ws.EffectiveSlipRatio = 0f;
                ws.EffectiveSlipAngleRad = 0f;
                ws.NetDriveTorqueNm = 0f;
                ws.BrakeTorqueAppliedNm = 0f;
                ws.WarmStartBrakeImpulse = 0f;
                ws.FrictionUtilization = 0f;
                ws.LastWorldForce = float3.zero;

                WheelContacts[slot] = default;
                WheelInput wIn = WheelInputs[slot];
                wIn.ContactSampleCount = 0;
                WheelInputs[slot] = wIn;
            }
            else
            {
                WheelConfig wCfg = WheelConfigs[VehicleConfigs[veh.ConfigIndex].WheelConfigStartIndex + localWheelIndex];
                ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsDetached);
                ws.AngularVelocity = 0f;
                ws.SuspensionCompression = 0f;
                ws.SuspensionVelocity = 0f;
                ws.GroundLoadN = 0f;
                ws.LongitudinalDeflection = 0f;
                ws.LateralDeflection = 0f;
                ws.EffectiveRollingRadius = wCfg.UnloadedRadiusM;
                ws.WarmStartBrakeImpulse = 0f;
                ws.LastWorldForce = float3.zero;
            }

            Wheels[slot] = ws;

            // Drift-free mass update from VehicleConfig.MassKg based on detached wheel count
            int detachedCount = 0;
            for (int w = 0; w < veh.WheelCount; w++)
            {
                if ((Wheels[veh.WheelStartIndex + w].WheelFlags & WheelFlagBits.IsDetached) != 0)
                    detachedCount++;
            }

            VehicleConfig cfg = VehicleConfigs[veh.ConfigIndex];
            float liveMass = math.max(cfg.MassKg - detachedCount * wheelUnsprungMassKg, cfg.MassKg * 0.25f);
            veh.InverseMass = 1.0f / math.max(liveMass, 1.0f);
            Vehicles[vehicleIndex] = veh;

            TopologyVersion++;
        }

        public void ResetActiveVehicles()
        {
            ActiveVehicleCount = 0;
            ActiveWheelSlotCursor = 0;
            TopologyVersion++;
        }

        public void Dispose()
        {
            if (IsDisposed) return;
            IsDisposed = true;

            if (Vehicles.IsCreated) Vehicles.Dispose();
            if (Powertrains.IsCreated) Powertrains.Dispose();
            if (DriverInputs.IsCreated) DriverInputs.Dispose();
            if (RenderPoses.IsCreated) RenderPoses.Dispose();
            if (Wheels.IsCreated) Wheels.Dispose();
            if (WheelInputs.IsCreated) WheelInputs.Dispose();
            if (WheelContacts.IsCreated) WheelContacts.Dispose();
            if (ContactSamples.IsCreated) ContactSamples.Dispose();
            if (VehicleConfigs.IsCreated) VehicleConfigs.Dispose();
            if (PowertrainConfigs.IsCreated) PowertrainConfigs.Dispose();
            if (WheelConfigs.IsCreated) WheelConfigs.Dispose();
            if (SurfaceMaterials.IsCreated) SurfaceMaterials.Dispose();
        }
    }
}
