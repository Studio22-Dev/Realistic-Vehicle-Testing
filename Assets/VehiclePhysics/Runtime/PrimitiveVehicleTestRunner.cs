using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Bridge;
using VehiclePhysics.Core;

namespace VehiclePhysics.Runtime
{
    public class PrimitiveVehicleTestRunner : MonoBehaviour
    {
        public enum DrivetrainType
        {
            RWD = 0,
            FWD = 1,
            AWD = 2
        }

        [Header("Simulation Tier")]
        [SerializeField, Range(1, 12)] private ushort substeps = 9; // 9 (~540Hz) or 2 (120Hz)
        [SerializeField] private LayerMask groundLayerMask = ~0;
        [SerializeField] private float initialLaunchSpeedMs = 0f;

        [Header("Powertrain & Assists")]
        [SerializeField] private DrivetrainType drivetrain = DrivetrainType.RWD;
        [SerializeField] private bool autoTransmission = true;
        [SerializeField] private bool autoClutch = true;
        [SerializeField] private bool enableAbs = true;
        [SerializeField] private bool enableTcs = true;
        [SerializeField, Range(0f, 150f)] private float rearLsdPreloadNm = 40f;
        [SerializeField, Range(0f, 1f)] private float rearLsdLockBias = 0.35f;

        [Header("Live Controls (Works With or Without Legacy Input)")]
        [SerializeField] private bool readKeyboardIfLegacyEnabled = true;
        [SerializeField, Range(0f, 1f)] private float throttleInput;
        [SerializeField, Range(0f, 1f)] private float brakeInput;
        [SerializeField, Range(-1f, 1f)] private float steerInput;
        [SerializeField, Range(0f, 1f)] private float clutchPedalInput;
        [SerializeField, Range(0f, 1f)] private float handbrakeInput;
        [SerializeField] private bool showTelemetryHud = true;

        [Header("Optional Pre-Built Visuals (Auto-Built If Null)")]
        [SerializeField] private Transform chassisVisualRoot;
        [SerializeField] private Transform[] wheelHubVisuals = new Transform[4];
        [SerializeField] private Collider[] selfCollidersToIgnore;

        private VehicleSimulationWorld _world;
        private VehicleRaycastPipeline _rayPipeline;
        private int _vehicleIndex;
        private float3 _spawnPos;
        private quaternion _spawnRot;
        private sbyte _pendingShiftDelta;
        private sbyte _manualTargetGear;

        private void Start()
        {
            _spawnPos = transform.position;
            _spawnRot = transform.rotation;

            if (chassisVisualRoot == null || wheelHubVisuals == null || wheelHubVisuals.Length < 4 || wheelHubVisuals[0] == null)
                BuildPrimitiveHierarchy();

            InitializeWorld();
        }

        private void BuildPrimitiveHierarchy()
        {
            var chassisGo = GameObject.CreatePrimitive(PrimitiveType.Cube);
            chassisGo.name = "ChassisBodyVisual";
            chassisGo.transform.SetParent(transform, false);
            chassisGo.transform.localPosition = new Vector3(0f, 0.22f, 0f);
            chassisGo.transform.localScale = new Vector3(1.75f, 0.55f, 4.2f);
            chassisVisualRoot = transform;

            var chassisCol = chassisGo.GetComponent<Collider>();
            selfCollidersToIgnore = new[] { chassisCol };

            Vector3[] mounts =
            {
                new(-0.82f, 0f,  1.30f), // FL
                new( 0.82f, 0f,  1.30f), // FR
                new(-0.82f, 0f, -1.30f), // RL
                new( 0.82f, 0f, -1.30f), // RR
            };
            string[] names = { "WheelHub_FL", "WheelHub_FR", "WheelHub_RL", "WheelHub_RR" };

            wheelHubVisuals = new Transform[4];
            for (int i = 0; i < 4; i++)
            {
                var hub = new GameObject(names[i]);
                hub.transform.SetParent(transform, false);
                hub.transform.localPosition = mounts[i];
                wheelHubVisuals[i] = hub.transform;

                var cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                cyl.name = "WheelMesh";
                Destroy(cyl.GetComponent<Collider>());
                cyl.transform.SetParent(hub.transform, false);
                cyl.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
                cyl.transform.localScale = new Vector3(0.66f, 0.12f, 0.66f);

                // Visual spoke bar on outer wheel face so spin rotation is clearly visible
                var spoke = GameObject.CreatePrimitive(PrimitiveType.Cube);
                spoke.name = "WheelSpokeMarker";
                Destroy(spoke.GetComponent<Collider>());
                spoke.transform.SetParent(hub.transform, false);
                spoke.transform.localPosition = new Vector3(i % 2 == 0 ? -0.125f : 0.125f, 0f, 0f);
                spoke.transform.localScale = new Vector3(0.02f, 0.56f, 0.08f);
            }
        }

        private void InitializeWorld()
        {
            _world = new VehicleSimulationWorld(maxVehicles: 1, maxArchetypes: 1, maxSurfaceMaterials: 2);
            _rayPipeline = new VehicleRaycastPipeline(_world.MaxTotalWheelSlots, maxHitsPerRay: 3);

            _world.SurfaceMaterials[0] = new SurfaceMaterialData
            {
                StaticFriction = 1.05f,
                DynamicFriction = 0.85f,
                RollingResistance = 0.015f,
                TireGripMultiplier = 1.0f,
                MaterialTypeId = 0
            };

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

            float frontSplit = drivetrain switch
            {
                DrivetrainType.FWD => 1.0f,
                DrivetrainType.AWD => 0.40f,
                _ => 0.0f
            };

            var ptCfg = new PowertrainConfig
            {
                // ~360 Nm peak sports sedan torque curve (100..700 rad/s / ~955..6685 RPM)
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
                FrontAxleTorqueSplit = frontSplit,
                ShiftDurationS = 0.14f,
                FrontLsdPreloadNm = drivetrain == DrivetrainType.FWD || drivetrain == DrivetrainType.AWD ? 25f : 0f,
                FrontLsdLockBias = drivetrain == DrivetrainType.FWD ? 0.30f : 0.15f,
                RearLsdPreloadNm = rearLsdPreloadNm,
                RearLsdLockBias = rearLsdLockBias,
                AutoShiftUpOmega = 640f,
                AutoShiftDownOmega = 240f,
                ForwardGearCount = 6,
                AutoTransmission = (byte)(autoTransmission ? 1 : 0)
            };

            bool frontDriven = drivetrain == DrivetrainType.FWD || drivetrain == DrivetrainType.AWD;
            bool rearDriven = drivetrain == DrivetrainType.RWD || drivetrain == DrivetrainType.AWD;

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
                    RestLengthM = 0.40f,
                    MaxTravelM = 0.28f,
                    SpringStiffnessNPerM = 35000f,
                    BumpDamperNsPerM = 3200f,
                    ReboundDamperNsPerM = 4500f,
                    BumpStopStiffnessNPerM = 180000f,
                    WheelInertiaKgM2 = 1.1f,
                    RadialTireStiffnessNPerM = 220000f,
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
                    AxleIndex = axle,
                    SideIndex = side,
                    DefaultControlFlags = flags
                };
            }

            _world.RegisterArchetype(
                0, vCfg, ptCfg,
                new[]
                {
                    MakeWheel(-0.82f,  1.30f, 0, 0, true,  frontDriven),
                    MakeWheel( 0.82f,  1.30f, 0, 1, true,  frontDriven),
                    MakeWheel(-0.82f, -1.30f, 1, 0, false, rearDriven),
                    MakeWheel( 0.82f, -1.30f, 1, 1, false, rearDriven),
                });

            _vehicleIndex = _world.SpawnVehicle(1, 0, _spawnPos, _spawnRot, true, substeps);

            if (selfCollidersToIgnore != null)
            {
                foreach (var col in selfCollidersToIgnore)
                {
                    if (col == null) continue;
                    _rayPipeline.RegisterCollider(col, new ColliderPhysicsMeta
                    {
                        OwnerVehicleId = 1,
                        DynamicBodyIndex = -1,
                        SurfaceMaterialIndex = 0
                    });
                }
            }

            RelaunchVehicle();
        }

        [ContextMenu("Relaunch / Reset Vehicle")]
        public void RelaunchVehicle()
        {
            if (_world == null || _world.IsDisposed) return;

            VehicleConfig vCfg = _world.VehicleConfigs[0];
            PowertrainConfig ptCfg = _world.PowertrainConfigs[0];

            VehicleState v = _world.Vehicles[_vehicleIndex];
            v.Rotation = math.normalize(_spawnRot);
            v.Position = _spawnPos + math.rotate(v.Rotation, vCfg.CenterOfMassLocalOffset);
            v.LinearVelocity = math.rotate(v.Rotation, new float3(0f, 0f, initialLaunchSpeedMs));
            v.AngularVelocity = float3.zero;
            v.ForwardSpeedMs = initialLaunchSpeedMs;
            v.SubstepCount = substeps;
            _world.Vehicles[_vehicleIndex] = v;

            VehicleRenderPose pose = _world.RenderPoses[_vehicleIndex];
            pose.PreviousPosition = v.Position;
            pose.CurrentPosition = v.Position;
            pose.PreviousRotation = v.Rotation;
            pose.CurrentRotation = v.Rotation;
            _world.RenderPoses[_vehicleIndex] = pose;

            _manualTargetGear = (sbyte)(initialLaunchSpeedMs > 1f ? 1 : 0);
            _pendingShiftDelta = 0;

            _world.Powertrains[_vehicleIndex] = new PowertrainState
            {
                EngineOmega = ptCfg.IdleOmega,
                CurrentGear = _manualTargetGear,
                TargetGear = _manualTargetGear,
                ClutchLockState = ClutchLockStates.Open,
                DrivelineFlags = autoClutch ? DrivelineFlagBits.AutoClutch : DrivelineFlagBits.None
            };

            for (int w = 0; w < v.WheelCount; w++)
            {
                WheelState ws = _world.Wheels[v.WheelStartIndex + w];
                ws.AngularVelocity = initialLaunchSpeedMs / math.max(ws.EffectiveRollingRadius, 0.1f);
                ws.LongitudinalDeflection = 0f;
                ws.LateralDeflection = 0f;
                ws.WarmStartBrakeImpulse = 0f;
                _world.Wheels[v.WheelStartIndex + w] = ws;
            }
        }

        [ContextMenu("Shift Up")]
        public void ShiftUp() => _pendingShiftDelta = 1;

        [ContextMenu("Shift Down")]
        public void ShiftDown() => _pendingShiftDelta = -1;

        private void Update()
        {
            if (_world == null || _world.IsDisposed) return;

#if ENABLE_LEGACY_INPUT_MANAGER
            if (readKeyboardIfLegacyEnabled)
            {
                throttleInput = (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) ? 1f : 0f;
                brakeInput = (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) ? 1f : 0f;
                steerInput = Input.GetAxis("Horizontal");
                handbrakeInput = Input.GetKey(KeyCode.Space) ? 1f : 0f;
                clutchPedalInput = Input.GetKey(KeyCode.LeftShift) ? 1f : 0f;

                if (Input.GetKeyDown(KeyCode.E)) _pendingShiftDelta = 1;
                else if (Input.GetKeyDown(KeyCode.Q)) _pendingShiftDelta = -1;

                if (Input.GetKeyDown(KeyCode.R)) RelaunchVehicle();
            }
#endif
            float alpha = Time.fixedDeltaTime > 1e-6f
                ? math.saturate((Time.time - Time.fixedTime) / Time.fixedDeltaTime)
                : 1f;

            VehicleState veh = _world.Vehicles[_vehicleIndex];
            VehicleConfig vCfg = _world.VehicleConfigs[veh.ConfigIndex];
            VehicleRenderPose pose = _world.RenderPoses[_vehicleIndex];

            float3 interpComPos = math.lerp(pose.PreviousPosition, pose.CurrentPosition, alpha);
            quaternion interpRot = math.slerp(pose.PreviousRotation, pose.CurrentRotation, alpha);
            float3 meshOriginPos = interpComPos - math.rotate(interpRot, vCfg.CenterOfMassLocalOffset);

            chassisVisualRoot.SetPositionAndRotation(meshOriginPos, interpRot);

            for (int w = 0; w < 4; w++)
            {
                if (wheelHubVisuals[w] == null) continue;
                WheelConfig wCfg = _world.WheelConfigs[vCfg.WheelConfigStartIndex + w];
                WheelState ws = _world.Wheels[veh.WheelStartIndex + w];

                float3 hubLocal = wCfg.MountPositionLocal
                                + wCfg.StrutAxisLocal * (wCfg.RestLengthM - ws.SuspensionCompression);

                quaternion wheelLocalRot = math.mul(
                    quaternion.RotateY(ws.SteerAngleRad),
                    quaternion.RotateX(ws.SpinAngle));

                wheelHubVisuals[w].SetLocalPositionAndRotation(hubLocal, wheelLocalRot);
            }
        }

        private void FixedUpdate()
        {
            if (_world == null || _world.IsDisposed) return;

            VehicleState v = _world.Vehicles[_vehicleIndex];
            v.SubstepCount = substeps;
            _world.Vehicles[_vehicleIndex] = v;

            // Allow live toggling of AutoTransmission, AutoClutch, and LSD settings from Inspector
            PowertrainConfig ptCfg = _world.PowertrainConfigs[0];
            ptCfg.AutoTransmission = (byte)(autoTransmission ? 1 : 0);
            ptCfg.RearLsdPreloadNm = rearLsdPreloadNm;
            ptCfg.RearLsdLockBias = rearLsdLockBias;
            _world.PowertrainConfigs[0] = ptCfg;

            PowertrainState pt = _world.Powertrains[_vehicleIndex];
            if (autoClutch) pt.DrivelineFlags |= DrivelineFlagBits.AutoClutch;
            else pt.DrivelineFlags = (byte)(pt.DrivelineFlags & ~DrivelineFlagBits.AutoClutch);
            _world.Powertrains[_vehicleIndex] = pt;

            if (!autoTransmission && _pendingShiftDelta != 0)
            {
                _manualTargetGear = (sbyte)math.clamp(pt.CurrentGear + _pendingShiftDelta, -1, ptCfg.ForwardGearCount);
            }

            byte inputFlags = 0;
            if (!enableAbs) inputFlags |= 1 << 0;
            if (!enableTcs) inputFlags |= 1 << 1;

            _world.DriverInputs[_vehicleIndex] = new VehicleDriverInput
            {
                Throttle = throttleInput,
                Brake = brakeInput,
                Steer = steerInput,
                ClutchPedal = clutchPedalInput,
                Handbrake = handbrakeInput,
                RequestedGear = autoTransmission ? (sbyte)-128 : _manualTargetGear,
                ShiftDelta = autoTransmission ? _pendingShiftDelta : (sbyte)0,
                InputFlags = inputFlags
            };
            _pendingShiftDelta = 0;

            JobHandle queryHandle = _rayPipeline.ScheduleQueries(
                _world, Physics.defaultPhysicsScene, groundLayerMask);

            var simJob = new VehicleSimulationJob
            {
                FixedDeltaTime = Time.fixedDeltaTime,
                VehicleConfigs = _world.VehicleConfigs,
                PowertrainConfigs = _world.PowertrainConfigs,
                WheelConfigs = _world.WheelConfigs,
                DriverInputs = _world.DriverInputs,
                WheelContacts = _world.WheelContacts,
                Vehicles = _world.Vehicles,
                Powertrains = _world.Powertrains,
                RenderPoses = _world.RenderPoses,
                WheelInputs = _world.WheelInputs,
                Wheels = _world.Wheels
            };

            simJob.Schedule(_world.ActiveVehicleCount, 1, queryHandle).Complete();
        }

        private void OnGUI()
        {
            if (!showTelemetryHud || _world == null || _world.IsDisposed || !Application.isPlaying) return;

            VehicleState v = _world.Vehicles[_vehicleIndex];
            PowertrainState pt = _world.Powertrains[_vehicleIndex];

            float kmh = v.ForwardSpeedMs * 3.6f;
            float rpm = pt.EngineOmega * 9.5492966f; // rad/s -> RPM
            string gearStr = pt.CurrentGear switch { -1 => "R", 0 => "N", _ => pt.CurrentGear.ToString() };
            string clutchStr = pt.ClutchLockState switch
            {
                ClutchLockStates.Open => "OPEN",
                ClutchLockStates.Slipping => "SLIP",
                _ => "LOCKED"
            };

            bool anyAbs = false, anyTcs = false;
            for (int w = 0; w < v.WheelCount; w++)
            {
                byte wf = _world.Wheels[v.WheelStartIndex + w].WheelFlags;
                if ((wf & WheelFlagBits.AbsActive) != 0) anyAbs = true;
                if ((wf & WheelFlagBits.TcsActive) != 0) anyTcs = true;
            }
            bool stalled = (pt.DrivelineFlags & DrivelineFlagBits.EngineStalled) != 0;

            GUI.Box(new Rect(14, 14, 320, 175), "Vehicle Physics Telemetry (Phase 1)");
            GUI.Label(new Rect(24, 38, 300, 22), $"Speed: {kmh:F1} km/h  |  Gear: {gearStr}  |  RPM: {rpm:F0}");
            GUI.Label(new Rect(24, 58, 300, 22), $"Clutch: {clutchStr} ({pt.ClutchEngagement:P0}) | Slip: {pt.ClutchSlipOmega:F1} rad/s");
            GUI.Label(new Rect(24, 78, 300, 22), $"Engine: {pt.EngineTorqueNm:F0} Nm  |  Clutch Out: {pt.ClutchTorqueNm:F0} Nm");
            GUI.Label(new Rect(24, 98, 300, 22), $"ABS: {(anyAbs ? "ACTIVE" : (enableAbs ? "Ready" : "Off"))}  |  TCS: {(anyTcs ? "ACTIVE" : (enableTcs ? "Ready" : "Off"))}  |  {(stalled ? "STALLED" : "OK")}");
            GUI.Label(new Rect(24, 120, 300, 22), $"Substeps: {v.SubstepCount} (~{v.SubstepCount * 60} Hz) | Layout: {drivetrain}");
            GUI.Label(new Rect(24, 145, 300, 36), "W/S: Gas/Brake | A/D: Steer | Space: Handbrake\nQ/E: Shift Down/Up | Shift: Clutch | R: Reset");
        }

        private void OnDrawGizmos()
        {
            if (_world == null || _world.IsDisposed || !Application.isPlaying) return;

            VehicleState veh = _world.Vehicles[_vehicleIndex];
            VehicleConfig vCfg = _world.VehicleConfigs[veh.ConfigIndex];

            for (int w = 0; w < veh.WheelCount; w++)
            {
                WheelConfig wCfg = _world.WheelConfigs[vCfg.WheelConfigStartIndex + w];
                WheelState ws = _world.Wheels[veh.WheelStartIndex + w];
                WheelContact contact = _world.WheelContacts[veh.WheelStartIndex + w];

                float3 mountWorld = veh.Position
                    + math.rotate(veh.Rotation, wCfg.MountPositionLocal - vCfg.CenterOfMassLocalOffset);
                float3 strutDir = math.rotate(veh.Rotation, wCfg.StrutAxisLocal);
                float3 hubWorld = mountWorld + strutDir * (wCfg.RestLengthM - ws.SuspensionCompression);

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(mountWorld, hubWorld);
                Gizmos.DrawWireSphere(hubWorld, 0.04f);

                if (contact.HasContact != 0)
                {
                    Gizmos.color = (ws.WheelFlags & WheelFlagBits.IsGrounded) != 0 ? Color.green : Color.gray;
                    Gizmos.DrawWireSphere(contact.PlanePoint, 0.05f);
                    Gizmos.DrawRay(contact.PlanePoint, contact.PlaneNormal * 0.35f);

                    Gizmos.color = Color.cyan;
                    Gizmos.DrawRay(contact.PlanePoint, ws.LastWorldForce * 0.00025f);
                }
            }
        }

        private void OnDestroy()
        {
            _rayPipeline?.Dispose();
            _world?.Dispose();
        }
    }
}
