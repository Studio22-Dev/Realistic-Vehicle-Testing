using System.Diagnostics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Jobs;
using VehiclePhysics.Bridge;
using VehiclePhysics.Core;

namespace VehiclePhysics.Runtime
{
    public class VehicleSimulationManager : MonoBehaviour
    {
        private JobHandle _pendingSimHandle;

        [Header("Fleet & Tiered Sub-Stepping")]
        [SerializeField, Range(1, 32)] private int totalVehicles = 24;
        [SerializeField, Range(1, 12)] private ushort playerSubsteps = 9;   // 9 x 60Hz = 540Hz
        [SerializeField, Range(1, 6)] private ushort nearAiSubsteps = 2;    // 2 x 60Hz = 120Hz
        [SerializeField, Range(1, 6)] private ushort farAiSubsteps = 2;     // 2 x 60Hz = 120Hz
        [SerializeField] private float nearAiRadiusM = 60f;
        [SerializeField] private float sleepRadiusM = 350f;
        [SerializeField] private LayerMask groundLayerMask = ~0;

        [Header("Chassis Broadphase & Track")]
        [SerializeField] private bool enableVehicleCollisions = true;
        [SerializeField] private float trackSemiMajorM = 95f;
        [SerializeField] private float trackSemiMinorM = 55f;
        [SerializeField] private int waypointCount = 24;

        [Header("Camera & Diagnostics")]
        [SerializeField] private Camera followCamera;
        [SerializeField] private bool showProfilerHud = true;
        [SerializeField] private bool enableDebugDetachKey = true;

        private static readonly ProfilerMarker s_LodAndAiMarker = new("VehicleSim.LodAndAi");
        private static readonly ProfilerMarker s_RaycastMarker = new("VehicleSim.RaycastQueries");
        private static readonly ProfilerMarker s_ChassisColMarker = new("VehicleSim.ChassisCollision");
        private static readonly ProfilerMarker s_SolverMarker = new("VehicleSim.SubstepSolver");
        private static readonly ProfilerMarker s_VisualSyncMarker = new("VehicleSim.TransformSync");

        private VehicleSimulationWorld _world;
        private VehicleRaycastPipeline _rayPipeline;

        private NativeArray<float3> _waypoints;
        private NativeArray<int> _aiWaypointIndices;
        private NativeList<int2> _wheelVisualMap;

        private TransformAccessArray _chassisTransforms;
        private TransformAccessArray _wheelTransforms;
        private Transform[] _chassisRootRefs;
        private GameObject[] _wheelHubGoBySlot;
        private sbyte _pendingShiftDelta;

        private readonly Stopwatch _sw = new();
        private double _lastSimMs;
        private double _lastSyncMs;

        private void Start()
        {
            InitializeSimulation();
        }

        private void InitializeSimulation()
        {
            _world = new VehicleSimulationWorld(
                maxVehicles: totalVehicles,
                maxArchetypes: 1,
                maxSurfaceMaterials: 2);
            _rayPipeline = new VehicleRaycastPipeline(_world.MaxTotalWheelSlots, maxHitsPerRay: 3);

            _world.SurfaceMaterials[0] = new SurfaceMaterialData
            {
                StaticFriction = 1.05f,
                DynamicFriction = 0.85f,
                RollingResistance = 0.015f,
                TireGripMultiplier = 1.0f,
                MaterialTypeId = 0
            };

            RegisterDefaultSportsSedanArchetype();
            BuildOvalWaypoints();

            _chassisTransforms = new TransformAccessArray(totalVehicles);
            _wheelTransforms = new TransformAccessArray(_world.MaxTotalWheelSlots);
            _wheelVisualMap = new NativeList<int2>(_world.MaxTotalWheelSlots, Allocator.Persistent);
            _chassisRootRefs = new Transform[totalVehicles];
            _wheelHubGoBySlot = new GameObject[_world.MaxTotalWheelSlots];
            _aiWaypointIndices = new NativeArray<int>(totalVehicles, Allocator.Persistent);

            for (int i = 0; i < totalVehicles; i++)
            {
                bool isPlayer = i == 0;
                float angle = (i / (float)math.max(totalVehicles, 1)) * math.PI * 2f;
                float laneOffset = (i % 2 == 0 ? -3.0f : 3.0f);

                float3 pos = SampleOvalPoint(angle, laneOffset);
                pos.y += 0.65f;
                float3 tangent = math.normalizesafe(
                    SampleOvalPoint(angle + 0.05f, laneOffset) - SampleOvalPoint(angle, laneOffset),
                    new float3(0f, 0f, 1f));
                quaternion rot = quaternion.LookRotationSafe(tangent, new float3(0f, 1f, 0f));

                int vehIdx = _world.SpawnVehicle(
                    (uint)(i + 1),
                    0,
                    pos,
                    rot,
                    isPlayer,
                    isPlayer ? playerSubsteps : nearAiSubsteps);

                int wp = (int)math.floor((angle / (math.PI * 2f)) * waypointCount) % waypointCount;
                _aiWaypointIndices[vehIdx] = (wp + 1) % waypointCount;

                SpawnPrimitiveVisuals(vehIdx, isPlayer, pos, rot);
            }

            if (_wheelTransforms.length != _wheelVisualMap.Length)
            {
                UnityEngine.Debug.LogError(
                    $"Visual wheel sync mismatch: _wheelTransforms.length={_wheelTransforms.length}, _wheelVisualMap.Length={_wheelVisualMap.Length}");
            }
        }

        private void RegisterDefaultSportsSedanArchetype()
        {
            var vCfg = new VehicleConfig
            {
                CenterOfMassLocalOffset = new float3(0f, -0.05f, 0f),
                MassKg = 1400f,
                InertiaLocal = new float3(2200f, 2500f, 650f),
                GravityMs2 = 9.81f,
                AirDensityKgM3 = 1.225f,
                AeroDragAreaCdA = 0.65f,
                AeroFrontDownforceClA = 0.15f,
                AeroRearDownforceClA = 0.25f,
                MaxSteerAngleRad = math.radians(34f),
                WheelbaseM = 2.60f,
                TrackWidthFrontM = 1.64f,
                TrackWidthRearM = 1.64f,
                AckermannFactor = 0.85f,
                FrontAntiRollStiffnessNPerM = 18000f,
                RearAntiRollStiffnessNPerM = 12000f,
                SteeringRateRadPerS = 5.5f,
                HighSpeedSteerScale = 0.0008f,
                HasPowertrain = 1
            };

            var ptCfg = new PowertrainConfig
            {
                WotTorque0 = new float4(190f, 235f, 285f, 325f),
                WotTorque1 = new float4(350f, 360f, 360f, 355f),
                WotTorque2 = new float4(345f, 330f, 315f, 295f),
                WotTorque3 = new float4(275f, 250f, 225f, 195f),
                ForwardGears1To4 = new float4(3.60f, 2.19f, 1.41f, 1.00f),
                ForwardGears5To8 = new float4(0.83f, 0.72f, 0f, 0f),
                ReverseGearRatio = -3.40f,
                FinalDriveRatio = 3.73f,
                EngineInertia = 0.24f,
                GearboxInertia = 0.08f,
                ClutchMaxTorqueNm = 520f,
                ClutchStaticHysteresis = 1.15f,
                DrivelineEfficiency = 0.90f,
                DrivelineBaseDragNm = 12f,
                IdleOmega = 90f,
                RedlineOmega = 700f,
                RevLimiterDropOmega = 30f,
                RevLimiterCutTimeS = 0.06f,
                CurveMinOmega = 100f,
                CurveMaxOmega = 700f,
                EngineFrictionBaseNm = 18f,
                EngineFrictionViscous = 0.04f,
                FrontAxleTorqueSplit = 0f,
                ShiftDurationS = 0.14f,
                RearLsdPreloadNm = 40f,
                RearLsdLockBias = 0.35f,
                AutoShiftUpOmega = 640f,
                AutoShiftDownOmega = 240f,
                ForwardGearCount = 6,
                AutoTransmission = 1
            };

            WheelConfig MakeWheel(float x, float z, byte axle, byte side, bool steer, bool driven)
            {
                byte flags = WheelControlFlagBits.AllowAbs;
                if (steer) flags |= WheelControlFlagBits.IsSteerable;
                if (driven) flags |= WheelControlFlagBits.IsDriven | WheelControlFlagBits.AllowTcs;

                return new WheelConfig
                {
                    MountPositionLocal = new float3(x, 0f, z),
                    StrutAxisLocal = new float3(0f, -1f, 0f),
                    UnloadedRadiusM = 0.33f,
                    RimRadiusM = 0.215f,
                    RestLengthM = 0.40f,
                    MaxTravelM = 0.28f,
                    SpringStiffnessNPerM = 35000f,
                    BumpDamperNsPerM = 3200f,
                    ReboundDamperNsPerM = 4500f,
                    BumpStopStiffnessNPerM = 180000f,
                    WheelInertiaKgM2 = 1.1f,
                    RadialTireStiffnessNPerM = 220000f,
                    RadialStiffnessNPerM = 220000f,
                    MaxRadialSquishM = 0.05f,
                    RelaxationLengthX = 0.25f,
                    RelaxationLengthY = 0.40f,
                    PeakSlipRatio = 0.12f,
                    PeakSlipAngleRad = math.radians(8f),
                    MaxServiceBrakeTorqueNm = 2600f,
                    MaxHandbrakeTorqueNm = 1800f,
                    TirePeakMuScale = 1.0f,
                    TireSlideMuScale = 1.0f,
                    LowSpeedDampMaxForceScale = 1.5f,
                    NominalPressureKPa = 220f,
                    InitialTreadDepthMm = 8.0f,
                    CompoundType = TireCompoundType.Street,
                    AxleIndex = axle,
                    SideIndex = side,
                    DefaultControlFlags = flags
                };
            }

            _world.RegisterArchetype(
                0, vCfg, ptCfg,
                new[]
                {
                    MakeWheel(-0.82f,  1.30f, 0, 0, true,  false),
                    MakeWheel( 0.82f,  1.30f, 0, 1, true,  false),
                    MakeWheel(-0.82f, -1.30f, 1, 0, false, true),
                    MakeWheel( 0.82f, -1.30f, 1, 1, false, true),
                });
        }

        private void BuildOvalWaypoints()
        {
            _waypoints = new NativeArray<float3>(waypointCount, Allocator.Persistent);
            for (int i = 0; i < waypointCount; i++)
            {
                float a = (i / (float)waypointCount) * math.PI * 2f;
                _waypoints[i] = SampleOvalPoint(a, 0f);
            }
        }

        private float3 SampleOvalPoint(float angleRad, float radialOffset)
        {
            float x = (trackSemiMajorM + radialOffset) * math.cos(angleRad);
            float z = (trackSemiMinorM + radialOffset) * math.sin(angleRad);
            return new float3(x, 0f, z);
        }

        private void SpawnPrimitiveVisuals(int vehIdx, bool isPlayer, float3 pos, quaternion rot)
        {
            VehicleState veh = _world.Vehicles[vehIdx];
            VehicleConfig vCfg = _world.VehicleConfigs[veh.ConfigIndex];

            var root = new GameObject(isPlayer ? "PlayerVehicle_0" : $"AiVehicle_{vehIdx}");
            root.transform.SetPositionAndRotation(pos, rot);
            _chassisRootRefs[vehIdx] = root.transform;
            _chassisTransforms.Add(root.transform);

            var body = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(body.GetComponent<Collider>());
            body.transform.SetParent(root.transform, false);
            body.transform.localPosition = new Vector3(0f, 0.22f, 0f);
            body.transform.localScale = new Vector3(1.75f, 0.55f, 4.2f);
            if (body.TryGetComponent<Renderer>(out var rend))
                rend.material.color = isPlayer ? new Color(0.95f, 0.22f, 0.15f) : new Color(0.22f, 0.55f, 0.85f);

            // Spawns exact number of wheels from veh.WheelCount (supports 2, 4, 6, 8+ wheels)
            for (int w = 0; w < veh.WheelCount; w++)
            {
                WheelConfig wCfg = _world.WheelConfigs[vCfg.WheelConfigStartIndex + w];
                float diameter = wCfg.UnloadedRadiusM * 2.0f;

                var hub = new GameObject($"Wheel_{w}");
                hub.transform.SetParent(root.transform, false);
                hub.transform.localPosition = wCfg.MountPositionLocal + wCfg.StrutAxisLocal * wCfg.RestLengthM;

                RegisterWheelVisual(vehIdx, w, veh.WheelStartIndex + w, hub);

                var cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                Destroy(cyl.GetComponent<Collider>());
                cyl.transform.SetParent(hub.transform, false);
                cyl.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
                cyl.transform.localScale = new Vector3(diameter, 0.12f, diameter);

                var spoke = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Destroy(spoke.GetComponent<Collider>());
                spoke.transform.SetParent(hub.transform, false);
                spoke.transform.localPosition = new Vector3(wCfg.SideIndex == 0 ? -0.125f : 0.125f, 0f, 0f);
                spoke.transform.localScale = new Vector3(0.02f, diameter * 0.85f, 0.08f);
            }
        }

        private void RegisterWheelVisual(int vehIdx, int localWheelIdx, int wheelSlot, GameObject hubGo)
        {
            _wheelTransforms.Add(hubGo.transform);
            _wheelVisualMap.Add(new int2(vehIdx, localWheelIdx));
            if ((uint)wheelSlot < (uint)_wheelHubGoBySlot.Length)
                _wheelHubGoBySlot[wheelSlot] = hubGo;
        }

        /// <summary>
        /// Detaches or reattaches/repairs a specific wheel without reallocating arrays or TransformAccessArray.
        /// </summary>
        public void SetWheelDetached(int vehIdx, int localWheelIdx, bool detached)
        {
            if (_world == null || _world.IsDisposed) return;
            if ((uint)vehIdx >= (uint)_world.ActiveVehicleCount) return;

            VehicleState veh = _world.Vehicles[vehIdx];
            if ((uint)localWheelIdx >= (uint)veh.WheelCount) return;

            _world.SetWheelDetached(vehIdx, localWheelIdx, detached);

            int slot = veh.WheelStartIndex + localWheelIdx;
            if ((uint)slot < (uint)_wheelHubGoBySlot.Length && _wheelHubGoBySlot[slot] != null)
                _wheelHubGoBySlot[slot].SetActive(!detached);
        }

        private void FixedUpdate()
        {
            if (_world == null || _world.IsDisposed) return;

            int activeCount = _world.ActiveVehicleCount;
            if (activeCount <= 0) return;

            // Ensure any stray prior handle is complete before main-thread writes
            _pendingSimHandle.Complete();

            // 1. Read Player Input
            float throttle = 0f, brake = 0f, steer = 0f, handbrake = 0f;
#if ENABLE_LEGACY_INPUT_MANAGER
            throttle = (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) ? 1f : 0f;
            brake = (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) ? 1f : 0f;
            steer = Input.GetAxis("Horizontal");
            handbrake = Input.GetKey(KeyCode.Space) ? 1f : 0f;
#endif
            _world.DriverInputs[0] = new VehicleDriverInput
            {
                Throttle = throttle,
                Brake = brake,
                Steer = steer,
                Handbrake = handbrake,
                RequestedGear = -128,
                ShiftDelta = _pendingShiftDelta
            };
            _pendingShiftDelta = 0;

            _sw.Restart();
            try
            {
                // 2. Schedule Raycast Queries FIRST (no active writers on _world.Vehicles)
                using (s_RaycastMarker.Auto())
                {
                    _pendingSimHandle = _rayPipeline.ScheduleQueries(
                        _world, Physics.defaultPhysicsScene, groundLayerMask);
                }

                // 3. Schedule LOD Update + AI Waypoint Driver AFTER ray query prep
                using (s_LodAndAiMarker.Auto())
                {
                    float3 camPos = followCamera != null
                        ? (float3)followCamera.transform.position
                        : _world.Vehicles[0].Position;

                    var lodJob = new VehicleLodSchedulerJob
                    {
                        CameraOrPlayerPos = camPos,
                        PlayerSubsteps = playerSubsteps,
                        NearAiSubsteps = nearAiSubsteps,
                        FarAiSubsteps = farAiSubsteps,
                        NearAiDistSq = nearAiRadiusM * nearAiRadiusM,
                        SleepDistSq = sleepRadiusM * sleepRadiusM,
                        Vehicles = _world.Vehicles,
                        DriverInputs = _world.DriverInputs
                    };
                    _pendingSimHandle = lodJob.Schedule(activeCount, 4, _pendingSimHandle);

                    var aiJob = new AiTrackDriverJob
                    {
                        Vehicles = _world.Vehicles,
                        VehicleConfigs = _world.VehicleConfigs,
                        Waypoints = _waypoints,
                        CurrentWaypointIndices = _aiWaypointIndices,
                        DriverInputs = _world.DriverInputs,
                        MaxAiSpeedMs = 34f,
                        MinCornerSpeedMs = 15f
                    };
                    _pendingSimHandle = aiJob.Schedule(activeCount, 4, _pendingSimHandle);
                }

                // 4. Pairwise Chassis Collision Broadphase
                if (enableVehicleCollisions && activeCount > 1)
                {
                    using (s_ChassisColMarker.Auto())
                    {
                        var colJob = new VehicleChassisCollisionJob
                        {
                            ActiveVehicleCount = activeCount,
                            FixedDeltaTime = Time.fixedDeltaTime,
                            CapsuleRadius = 0.95f,
                            CapsuleHalfSpanZ = 1.15f,
                            Restitution = 0.22f,
                            BodyFriction = 0.25f,
                            Vehicles = _world.Vehicles
                        };
                        _pendingSimHandle = colJob.Schedule(_pendingSimHandle);
                    }
                }

                // 5. Execute Tiered Sub-Step Physics Solver (BatchSize = 1 so Index 0 starts immediately)
                using (s_SolverMarker.Auto())
                {
                    var simJob = new VehicleSimulationJob
                    {
                        FixedDeltaTime = Time.fixedDeltaTime,
                        VehicleConfigs = _world.VehicleConfigs,
                        PowertrainConfigs = _world.PowertrainConfigs,
                        WheelConfigs = _world.WheelConfigs,
                        DriverInputs = _world.DriverInputs,
                        SurfaceMaterials = _world.SurfaceMaterials,
                        Vehicles = _world.Vehicles,
                        Powertrains = _world.Powertrains,
                        RenderPoses = _world.RenderPoses,
                        WheelInputs = _world.WheelInputs,
                        Wheels = _world.Wheels,
                        ContactManifolds = _world.ContactManifolds,
                        TireCarcassStates = _world.TireCarcassStates,
                        TireThermalStates = _world.TireThermalStates
                    };
                    _pendingSimHandle = simJob.Schedule(activeCount, 1, _pendingSimHandle);
                }
            }
            finally
            {
                _pendingSimHandle.Complete();
                _pendingSimHandle = default;
            }
            _sw.Stop();

            _lastSimMs = _sw.Elapsed.TotalMilliseconds;
        }

        private void Update()
        {
            _pendingSimHandle.Complete();

            if (_world == null || _world.IsDisposed || _world.ActiveVehicleCount <= 0) return;

#if ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKeyDown(KeyCode.E)) _pendingShiftDelta = 1;
            else if (Input.GetKeyDown(KeyCode.Q)) _pendingShiftDelta = -1;

            if (enableDebugDetachKey && Input.GetKeyDown(KeyCode.F1))
            {
                int slot = _world.Vehicles[0].WheelStartIndex;
                bool currentlyDetached = (_world.Wheels[slot].WheelFlags & WheelFlagBits.IsDetached) != 0;
                SetWheelDetached(0, 0, !currentlyDetached);
            }
#endif
            float alpha = Time.fixedDeltaTime > 1e-6f
                ? math.saturate((Time.time - Time.fixedTime) / Time.fixedDeltaTime)
                : 1f;

            _sw.Restart();
            using (s_VisualSyncMarker.Auto())
            {
                var chassisJob = new ChassisTransformSyncJob
                {
                    Alpha = alpha,
                    Vehicles = _world.Vehicles,
                    VehicleConfigs = _world.VehicleConfigs,
                    RenderPoses = _world.RenderPoses
                };
                JobHandle cHandle = chassisJob.Schedule(_chassisTransforms);

                var wheelJob = new WheelTransformSyncJob
                {
                    WheelVisualMap = _wheelVisualMap.AsArray(),
                    Vehicles = _world.Vehicles,
                    VehicleConfigs = _world.VehicleConfigs,
                    WheelConfigs = _world.WheelConfigs,
                    Wheels = _world.Wheels
                };
                wheelJob.Schedule(_wheelTransforms, cHandle).Complete();
            }
            _sw.Stop();
            _lastSyncMs = _sw.Elapsed.TotalMilliseconds;
        }

        private void LateUpdate()
        {
            if (followCamera == null || _chassisRootRefs == null || _chassisRootRefs[0] == null) return;

            Transform target = _chassisRootRefs[0];
            Vector3 desiredPos = target.TransformPoint(new Vector3(0f, 3.2f, -7.8f));
            followCamera.transform.position = Vector3.Lerp(
                followCamera.transform.position, desiredPos, 1f - Mathf.Exp(-8f * Time.deltaTime));
            followCamera.transform.LookAt(target.position + Vector3.up * 0.8f);
        }

        private void OnGUI()
        {
            if (!showProfilerHud || _world == null || _world.IsDisposed || !Application.isPlaying) return;

            VehicleState p = _world.Vehicles[0];
            PowertrainState pt = _world.Powertrains[0];

            int totalSubsteps = 0;
            for (int i = 0; i < _world.ActiveVehicleCount; i++)
                totalSubsteps += _world.Vehicles[i].SubstepCount;

            float kmh = p.ForwardSpeedMs * 3.6f;
            float rpm = pt.EngineOmega * 9.5492966f;
            string gear = pt.CurrentGear switch { -1 => "R", 0 => "N", _ => pt.CurrentGear.ToString() };

            GUI.Box(new Rect(14, 14, 380, 145), "Multi-Vehicle Fleet Simulation (Phase 2 Step 1)");
            GUI.Label(new Rect(24, 38, 360, 22), $"Vehicles: {_world.ActiveVehicleCount} | Total Substeps/Tick: {totalSubsteps}");
            GUI.Label(new Rect(24, 58, 360, 22), $"Physics Step (Ray+Col+Solve): {_lastSimMs:F3} ms / 16.67 ms");
            GUI.Label(new Rect(24, 78, 360, 22), $"Burst Transform Sync: {_lastSyncMs:F3} ms");
            GUI.Label(new Rect(24, 100, 360, 22), $"Player: {kmh:F1} km/h | Gear: {gear} | RPM: {rpm:F0} ({p.SubstepCount * 60}Hz)");
            GUI.Label(new Rect(24, 122, 360, 22), "W/S Gas/Brk | A/D Steer | Space HB | Q/E Shift | F1 Detach FL");
        }

        private void OnDestroy()
        {
            _pendingSimHandle.Complete();

            if (_chassisTransforms.isCreated) _chassisTransforms.Dispose();
            if (_wheelTransforms.isCreated) _wheelTransforms.Dispose();
            if (_wheelVisualMap.IsCreated) _wheelVisualMap.Dispose();
            if (_waypoints.IsCreated) _waypoints.Dispose();
            if (_aiWaypointIndices.IsCreated) _aiWaypointIndices.Dispose();

            _rayPipeline?.Dispose();
            _world?.Dispose();
        }
    }
}
