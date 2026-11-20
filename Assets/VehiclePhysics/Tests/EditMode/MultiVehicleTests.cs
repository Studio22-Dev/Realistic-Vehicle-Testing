using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Bridge;
using VehiclePhysics.Core;

namespace VehiclePhysics.Tests
{
    public class MultiVehicleTests
    {
        [Test]
        public void Fleet24Vehicles_Runs120TicksCleanly_WithTieredSubstepsAndCollisions()
        {
            const int fleetSize = 24;
            const int wpCount = 16;

            GameObject ground = null;
            VehicleSimulationWorld world = null;
            VehicleRaycastPipeline pipeline = null;
            NativeArray<float3> waypoints = default;
            NativeArray<int> wpIndices = default;

            try
            {
                ground = new GameObject("TestOvalGround");
                ground.transform.position = new Vector3(0f, -0.5f, 0f);
                ground.AddComponent<BoxCollider>().size = new Vector3(800f, 1f, 800f);
                Physics.SyncTransforms();

                world = new VehicleSimulationWorld(fleetSize, 1, 2);
                pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);
                waypoints = new NativeArray<float3>(wpCount, Allocator.TempJob);
                wpIndices = new NativeArray<int>(fleetSize, Allocator.TempJob);

                world.SurfaceMaterials[0] = new SurfaceMaterialData
                {
                    StaticFriction = 1.05f, DynamicFriction = 0.85f, RollingResistance = 0.015f,
                    TireGripMultiplier = 1.0f
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
                    AckermannFactor = 0.85f,
                    FrontAntiRollStiffnessNPerM = 18000f,
                    RearAntiRollStiffnessNPerM = 12000f,
                    SteeringRateRadPerS = 5.5f,
                    HasPowertrain = 1
                };

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
                    ShiftDurationS = 0.14f,
                    RearLsdPreloadNm = 40f,
                    RearLsdLockBias = 0.35f,
                    AutoShiftUpOmega = 640f,
                    AutoShiftDownOmega = 240f,
                    ForwardGearCount = 6,
                    AutoTransmission = 1
                };

                WheelConfig MakeWheel(float x, float z, byte axle, byte side, bool steer, bool driven) => new WheelConfig
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
                    DefaultControlFlags = (byte)(
                        WheelControlFlagBits.AllowAbs |
                        (steer ? WheelControlFlagBits.IsSteerable : 0) |
                        (driven ? WheelControlFlagBits.IsDriven | WheelControlFlagBits.AllowTcs : 0))
                };

                world.RegisterArchetype(0, vCfg, ptCfg, new[]
                {
                    MakeWheel(-0.8f,  1.3f, 0, 0, true,  false),
                    MakeWheel( 0.8f,  1.3f, 0, 1, true,  false),
                    MakeWheel(-0.8f, -1.3f, 1, 0, false, true),
                    MakeWheel( 0.8f, -1.3f, 1, 1, false, true),
                });

                for (int w = 0; w < wpCount; w++)
                {
                    float a = w / (float)wpCount * math.PI * 2f;
                    waypoints[w] = new float3(80f * math.cos(a), 0f, 50f * math.sin(a));
                }

                for (int i = 0; i < fleetSize; i++)
                {
                    bool isPlayer = i == 0;
                    float a = i / (float)fleetSize * math.PI * 2f;
                    float3 pos = new float3(80f * math.cos(a), 0.65f, 50f * math.sin(a));
                    float3 fwd = math.normalizesafe(new float3(-80f * math.sin(a), 0f, 50f * math.cos(a)));
                    world.SpawnVehicle((uint)(i + 1), 0, pos, quaternion.LookRotationSafe(fwd, math.up()), isPlayer, (ushort)(isPlayer ? 9 : 2));
                    wpIndices[i] = ((int)math.floor(a / (math.PI * 2f) * wpCount) + 1) % wpCount;
                }

                for (int tick = 0; tick < 120; tick++)
                {
                    world.DriverInputs[0] = new VehicleDriverInput { Throttle = 0.7f, RequestedGear = -128 };

                    var rayH = pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0);

                    var lodH = new VehicleLodSchedulerJob
                    {
                        CameraOrPlayerPos = world.Vehicles[0].Position,
                        PlayerSubsteps = 9, NearAiSubsteps = 2, FarAiSubsteps = 2,
                        NearAiDistSq = 3600f, SleepDistSq = 122500f,
                        Vehicles = world.Vehicles, DriverInputs = world.DriverInputs
                    }.Schedule(fleetSize, 4, rayH);

                    var aiH = new AiTrackDriverJob
                    {
                        Vehicles = world.Vehicles, VehicleConfigs = world.VehicleConfigs,
                        Waypoints = waypoints, CurrentWaypointIndices = wpIndices,
                        DriverInputs = world.DriverInputs, MaxAiSpeedMs = 30f, MinCornerSpeedMs = 14f
                    }.Schedule(fleetSize, 4, lodH);

                    var colH = new VehicleChassisCollisionJob
                    {
                        ActiveVehicleCount = fleetSize, FixedDeltaTime = 1f / 60f,
                        CapsuleRadius = 0.95f, CapsuleHalfSpanZ = 1.15f,
                        Restitution = 0.22f, BodyFriction = 0.25f, Vehicles = world.Vehicles
                    }.Schedule(aiH);

                    new VehicleSimulationJob
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
                    }.Schedule(fleetSize, 1, colH).Complete();
                }

                Assert.AreEqual(9, world.Vehicles[0].SubstepCount);
                Assert.AreEqual(2, world.Vehicles[12].SubstepCount);

                for (int i = 0; i < fleetSize; i++)
                {
                    VehicleState v = world.Vehicles[i];
                    Assert.IsFalse(math.any(math.isnan(v.Position)), $"Vehicle {i} produced NaN position.");
                    Assert.Greater(v.Position.y, 0.2f);
                    Assert.Less(v.Position.y, 2.0f);
                    Assert.Greater(math.length(v.LinearVelocity), 5.0f, $"Vehicle {i} failed to accelerate.");
                }
            }
            finally
            {
                if (waypoints.IsCreated) waypoints.Dispose();
                if (wpIndices.IsCreated) wpIndices.Dispose();
                pipeline?.Dispose();
                world?.Dispose();
                if (ground != null) Object.DestroyImmediate(ground);
            }
        }
    }
}
