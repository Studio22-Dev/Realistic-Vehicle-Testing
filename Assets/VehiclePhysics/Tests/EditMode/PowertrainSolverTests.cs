using NUnit.Framework;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Bridge;
using VehiclePhysics.Core;

namespace VehiclePhysics.Tests
{
    public class PowertrainSolverTests
    {
        private static void CreatePoweredCarWorld(
            out VehicleSimulationWorld world,
            out VehicleRaycastPipeline pipeline,
            float rearLsdPreloadNm,
            float rearLsdBias,
            bool autoClutch = true)
        {
            world = new VehicleSimulationWorld(maxVehicles: 2, maxArchetypes: 1, maxSurfaceMaterials: 2);
            pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

            world.SurfaceMaterials[0] = new SurfaceMaterialData
            {
                StaticFriction = 1.05f, DynamicFriction = 0.85f, RollingResistance = 0.015f,
                TireGripMultiplier = 1.0f, MaterialTypeId = 0
            };
            world.SurfaceMaterials[1] = new SurfaceMaterialData
            {
                StaticFriction = 0.12f, DynamicFriction = 0.10f, RollingResistance = 0.015f,
                TireGripMultiplier = 1.0f, MaterialTypeId = 1 // Ice
            };

            var vCfg = new VehicleConfig
            {
                MassKg = 1400f,
                InertiaLocal = new float3(2200f, 2500f, 650f),
                GravityMs2 = 9.81f,
                AirDensityKgM3 = 1.225f,
                AeroDragAreaCdA = 0.65f,
                MaxSteerAngleRad = math.radians(34f),
                WheelbaseM = 2.6f,
                TrackWidthFrontM = 1.6f,
                TrackWidthRearM = 1.6f,
                AckermannFactor = 0.8f,
                FrontAntiRollStiffnessNPerM = 18000f,
                RearAntiRollStiffnessNPerM = 12000f,
                SteeringRateRadPerS = 6.0f,
                HasPowertrain = 1
            };

            // ~360 Nm peak sports sedan torque curve from 100..700 rad/s (~955..6685 RPM)
            var ptCfg = new PowertrainConfig
            {
                WotTorque0 = new float4(190, 235, 285, 325),
                WotTorque1 = new float4(350, 360, 360, 355),
                WotTorque2 = new float4(345, 330, 315, 295),
                WotTorque3 = new float4(275, 250, 225, 195),
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
                FrontAxleTorqueSplit = 0.0f, // RWD
                ShiftDurationS = 0.14f,
                RearLsdPreloadNm = rearLsdPreloadNm,
                RearLsdLockBias = rearLsdBias,
                RearLsdViscous = 0f,
                AutoShiftUpOmega = 650f,
                AutoShiftDownOmega = 240f,
                ForwardGearCount = 6,
                AutoTransmission = 1
            };

            WheelConfig MakeWheel(float x, float z, byte axle, byte side, bool steer) => new WheelConfig
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
                DefaultControlFlags = (byte)(steer
                    ? WheelControlFlagBits.IsSteerable | WheelControlFlagBits.AllowAbs
                    : WheelControlFlagBits.IsDriven | WheelControlFlagBits.AllowAbs)
            };

            world.RegisterArchetype(
                0, vCfg, ptCfg,
                new[]
                {
                    MakeWheel(-0.8f,  1.3f, 0, 0, true),
                    MakeWheel( 0.8f,  1.3f, 0, 1, true),
                    MakeWheel(-0.8f, -1.3f, 1, 0, false),
                    MakeWheel( 0.8f, -1.3f, 1, 1, false),
                });

            world.SpawnVehicle(1, 0, new float3(0f, 0.60f, 0f), quaternion.identity, true, 9);

            if (!autoClutch)
            {
                PowertrainState pt = world.Powertrains[0];
                pt.DrivelineFlags = (byte)(pt.DrivelineFlags & ~DrivelineFlagBits.AutoClutch);
                world.Powertrains[0] = pt;
            }
        }

        private static void Step(VehicleSimulationWorld world, VehicleRaycastPipeline pipeline, int steps, bool logProgress = false)
        {
            for (int i = 0; i < steps; i++)
            {
                var q = pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0);
                var sim = new VehicleSimulationJob
                {
                    FixedDeltaTime = 1f / 60f,
                    VehicleConfigs = world.VehicleConfigs,
                    PowertrainConfigs = world.PowertrainConfigs,
                    WheelConfigs = world.WheelConfigs,
                    DriverInputs = world.DriverInputs,
                    SurfaceMaterials = world.SurfaceMaterials,
                    Vehicles = world.Vehicles,
                    Powertrains = world.Powertrains,
                    RenderPoses = world.RenderPoses,
                    WheelInputs = world.WheelInputs,
                    Wheels = world.Wheels,
                    ContactManifolds = world.ContactManifolds,
                    TireCarcassStates = world.TireCarcassStates,
                    TireThermalStates = world.TireThermalStates
                };
                sim.Schedule(world.ActiveVehicleCount, 1, q).Complete();

                if (logProgress && ((i + 1) % 60 == 0))
                {
                    VehicleState vDiag = world.Vehicles[0];
                    PowertrainState ptDiag = world.Powertrains[0];
                    WheelState rlDiag = world.Wheels[vDiag.WheelStartIndex + 2];

                    Debug.Log(
                        $"[Step {i + 1}] Speed={vDiag.ForwardSpeedMs:F2} m/s | Gear={ptDiag.CurrentGear} (Target={ptDiag.TargetGear}) | " +
                        $"EngOmega={ptDiag.EngineOmega:F1} | ThrottleFilt={ptDiag.ThrottleFiltered:F2} | " +
                        $"Clutch={ptDiag.ClutchLockState} (Eng={ptDiag.ClutchEngagement:F2}) | " +
                        $"RL_Slip={rlDiag.EffectiveSlipRatio:F3} | TCS={(rlDiag.WheelFlags & WheelFlagBits.TcsActive) != 0}");
                }
            }
        }

        [Test]
        public void Powertrain_NeutralRevHitsLimiter_WithZeroCarCreep()
        {
            var ground = new GameObject("Ground");
            try
            {
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(500f, 1f, 500f);
                Physics.SyncTransforms();

                CreatePoweredCarWorld(out var world, out var pipeline, 40f, 0.35f);
                using (world) using (pipeline)
                {
                    Step(world, pipeline, 30);

                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 1.0f, RequestedGear = 0 };
                    Step(world, pipeline, 120);

                    PowertrainState pt = world.Powertrains[0];
                    VehicleState v = world.Vehicles[0];
                    Assert.AreEqual(0, pt.CurrentGear);
                    Assert.AreEqual(ClutchLockStates.Open, pt.ClutchLockState);
                    Assert.Greater(pt.EngineOmega, 600f);
                    Assert.LessOrEqual(pt.EngineOmega, 715f);
                    Assert.Less(math.abs(v.ForwardSpeedMs), 0.05f);
                }
            }
            finally { Object.DestroyImmediate(ground); }
        }

        [Test]
        public void Powertrain_LaunchesAcceleratesAndLocksClutch_ThroughGears()
        {
            var ground = new GameObject("Track");
            try
            {
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(1000f, 1f, 2000f);
                Physics.SyncTransforms();

                CreatePoweredCarWorld(out var world, out var pipeline, 40f, 0.35f);
                using (world) using (pipeline)
                {
                    Step(world, pipeline, 30);

                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 1.0f, RequestedGear = -128 };
                    Step(world, pipeline, 300);

                    VehicleState v = world.Vehicles[0];
                    PowertrainState pt = world.Powertrains[0];

                    Assert.IsFalse(math.any(math.isnan(v.Position)));
                    Assert.Greater(v.ForwardSpeedMs, 22f);
                    Assert.GreaterOrEqual(pt.CurrentGear, 2);
                    Assert.AreEqual(ClutchLockStates.Locked, pt.ClutchLockState);
                    Assert.Less(math.abs(pt.ClutchSlipOmega), 0.5f);
                }
            }
            finally { Object.DestroyImmediate(ground); }
        }

        [Test]
        public void Powertrain_ReverseGearAcceleratesBackward()
        {
            var ground = new GameObject("Ground");
            try
            {
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(500f, 1f, 500f);
                Physics.SyncTransforms();

                CreatePoweredCarWorld(out var world, out var pipeline, 40f, 0.35f);
                using (world) using (pipeline)
                {
                    Step(world, pipeline, 30);

                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 0.8f, RequestedGear = -1 };
                    Step(world, pipeline, 120);

                    Assert.AreEqual(-1, world.Powertrains[0].CurrentGear);
                    Assert.Less(world.Vehicles[0].ForwardSpeedMs, -6.0f);
                }
            }
            finally { Object.DestroyImmediate(ground); }
        }

        [Test]
        public void Powertrain_ManualClutchStallsEngine_WhenBrakedToStopInGear()
        {
            var ground = new GameObject("Ground");
            try
            {
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(500f, 1f, 500f);
                Physics.SyncTransforms();

                CreatePoweredCarWorld(out var world, out var pipeline, 40f, 0.35f, autoClutch: false);
                using (world) using (pipeline)
                {
                    Step(world, pipeline, 20);
                    PowertrainState pt = world.Powertrains[0];
                    pt.CurrentGear = 1;
                    pt.TargetGear = 1;
                    world.Powertrains[0] = pt;

                    world.DriverInputs[0] = new VehicleDriverInput
                    {
                        Throttle = 0f, Brake = 1.0f, ClutchPedal = 0f, RequestedGear = 1
                    };
                    Step(world, pipeline, 60);

                    Assert.AreNotEqual(0, world.Powertrains[0].DrivelineFlags & DrivelineFlagBits.EngineStalled);
                }
            }
            finally { Object.DestroyImmediate(ground); }
        }

        [Test]
        public void Audit_BrakeRelease_ImmediatelyClearsWarmStartAndTorque()
        {
            var ground = new GameObject("Ground");
            try
            {
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(500f, 1f, 500f);
                Physics.SyncTransforms();

                CreatePoweredCarWorld(out var world, out var pipeline, 40f, 0.35f);
                using (world) using (pipeline)
                {
                    Step(world, pipeline, 20);

                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 0.8f, RequestedGear = -128 };
                    Step(world, pipeline, 30);

                    world.DriverInputs[0] = new VehicleDriverInput { Brake = 1.0f, RequestedGear = -128 };
                    Step(world, pipeline, 15);

                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 0f, Brake = 0f, Handbrake = 0f, RequestedGear = -128 };
                    Step(world, pipeline, 1);

                    int start = world.Vehicles[0].WheelStartIndex;
                    for (int w = 0; w < 4; w++)
                    {
                        WheelState ws = world.Wheels[start + w];
                        Assert.AreEqual(0f, ws.WarmStartBrakeImpulse, 1e-5f);
                        Assert.AreEqual(0f, ws.BrakeTorqueAppliedNm, 1e-4f);
                    }
                }
            }
            finally { Object.DestroyImmediate(ground); }
        }

        [Test]
        public void Audit_ParkedCarWithNoBrake_HasZeroCreep()
        {
            var ground = new GameObject("Ground");
            try
            {
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(500f, 1f, 500f);
                Physics.SyncTransforms();

                CreatePoweredCarWorld(out var world, out var pipeline, 40f, 0.35f);
                using (world) using (pipeline)
                {
                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 0f, Brake = 0f, RequestedGear = 1 };
                    Step(world, pipeline, 180);

                    Assert.Less(math.abs(world.Vehicles[0].ForwardSpeedMs), 1e-3f);
                    int start = world.Vehicles[0].WheelStartIndex;
                    for (int w = 0; w < 4; w++)
                        Assert.Less(math.abs(world.Wheels[start + w].AngularVelocity), 1e-3f);
                }
            }
            finally { Object.DestroyImmediate(ground); }
        }
    }
}
