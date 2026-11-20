using NUnit.Framework;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Bridge;
using VehiclePhysics.Core;

namespace VehiclePhysics.Tests
{
    public class CorePhysicsSolverTests
    {
        private static void CreateStandardCarWorld(
            out VehicleSimulationWorld world,
            out VehicleRaycastPipeline pipeline,
            float3 spawnPos,
            quaternion spawnRot,
            ushort substeps)
        {
            world = new VehicleSimulationWorld(maxVehicles: 2, maxArchetypes: 1, maxSurfaceMaterials: 1);
            pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

            world.SurfaceMaterials[0] = new SurfaceMaterialData
            {
                StaticFriction = 1.05f,
                DynamicFriction = 0.85f,
                RollingResistance = 0.015f,
                TireGripMultiplier = 1.0f,
                MaterialTypeId = 0
            };

            var vCfg = new VehicleConfig
            {
                MassKg = 1400f,
                InertiaLocal = new float3(2200f, 2500f, 650f),
                GravityMs2 = 9.81f,
                AirDensityKgM3 = 1.225f,
                AeroDragAreaCdA = 0.65f,
                MaxSteerAngleRad = math.radians(35f),
                WheelbaseM = 2.6f,
                TrackWidthFrontM = 1.6f,
                TrackWidthRearM = 1.6f,
                AckermannFactor = 0.8f,
                FrontAntiRollStiffnessNPerM = 18000f,
                RearAntiRollStiffnessNPerM = 12000f,
                SteeringRateRadPerS = 6.0f,
                HighSpeedSteerScale = 0.001f,
                HasPowertrain = 1
            };

            WheelConfig MakeWheel(float x, float z, byte axle, byte side, bool steer) => new WheelConfig
            {
                MountPositionLocal = new float3(x, 0.0f, z),
                StrutAxisLocal = new float3(0f, -1f, 0f),
                UnloadedRadiusM = 0.33f,
                RestLengthM = 0.40f,
                MaxTravelM = 0.30f,
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
                MaxServiceBrakeTorqueNm = 2400f,
                MaxHandbrakeTorqueNm = 1800f,
                TirePeakMuScale = 1.0f,
                TireSlideMuScale = 1.0f,
                LowSpeedDampMaxForceScale = 1.5f,
                AxleIndex = axle,
                SideIndex = side,
                DefaultControlFlags = steer
                    ? WheelControlFlagBits.IsSteerable
                    : WheelControlFlagBits.IsDriven
            };

            world.RegisterArchetype(
                0, vCfg, new PowertrainConfig { IdleOmega = 85f },
                new[]
                {
                    MakeWheel(-0.8f,  1.3f, 0, 0, true),
                    MakeWheel( 0.8f,  1.3f, 0, 1, true),
                    MakeWheel(-0.8f, -1.3f, 1, 0, false),
                    MakeWheel( 0.8f, -1.3f, 1, 1, false),
                });

            world.SpawnVehicle(1, 0, spawnPos, spawnRot, substeps > 4, substepCount: substeps);
        }

        private static void StepSimulation(
            VehicleSimulationWorld world,
            VehicleRaycastPipeline pipeline,
            float fixedDt,
            int steps)
        {
            for (int i = 0; i < steps; i++)
            {
                var qHandle = pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0);
                var simJob = new VehicleSimulationJob
                {
                    FixedDeltaTime = fixedDt,
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
                simJob.Schedule(world.ActiveVehicleCount, 1, qHandle).Complete();
            }
        }

        [Test]
        public void Solver_SettlesOnFlatGround_WithExpectedStaticWeight()
        {
            var groundGo = new GameObject("FlatGround");
            try
            {
                groundGo.transform.position = new Vector3(0f, -0.5f, 0f);
                groundGo.AddComponent<BoxCollider>().size = new Vector3(200f, 1f, 200f);
                Physics.SyncTransforms();

                CreateStandardCarWorld(out var world, out var pipeline, new float3(0f, 0.8f, 0f), quaternion.identity, 9);
                using (world)
                using (pipeline)
                {
                    StepSimulation(world, pipeline, 1f / 60f, 120);

                    VehicleState v = world.Vehicles[0];
                    Assert.IsFalse(math.any(math.isnan(v.Position)));
                    Assert.Less(math.length(v.LinearVelocity), 0.02f);

                    float totalFz = 0f;
                    for (int w = 0; w < 4; w++)
                        totalFz += world.Wheels[v.WheelStartIndex + w].GroundLoadN;

                    // 1400 kg * 9.81 = 13,734 N
                    Assert.AreEqual(1400f * 9.81f, totalFz, 100f);
                }
            }
            finally { Object.DestroyImmediate(groundGo); }
        }

        [TestCase((ushort)9)] // ~540 Hz Player Tier
        [TestCase((ushort)2)] //  120 Hz AI Tier
        public void Solver_HoldsStationaryOn20DegSlope_WithoutJitterOrExplosion(ushort substeps)
        {
            var slopeGo = new GameObject("Slope20Deg");
            try
            {
                slopeGo.transform.position = new Vector3(0f, -0.5f, 0f);
                slopeGo.transform.rotation = Quaternion.Euler(20f, 0f, 0f);
                slopeGo.AddComponent<BoxCollider>().size = new Vector3(200f, 1f, 200f);
                Physics.SyncTransforms();

                quaternion carRot = quaternion.RotateX(math.radians(20f));
                float3 startPos = new float3(0f, 0.65f, 0f);

                CreateStandardCarWorld(out var world, out var pipeline, startPos, carRot, substeps);
                using (world)
                using (pipeline)
                {
                    // Apply brakes so wheels hold on the 20-degree incline
                    world.DriverInputs[0] = new VehicleDriverInput { Brake = 1.0f, RequestedGear = -128 };
                    StepSimulation(world, pipeline, 1f / 60f, 60); // Settle 1 sec

                    float3 settledPos = world.Vehicles[0].Position;
                    StepSimulation(world, pipeline, 1f / 60f, 180); // Hold 3 more sec

                    VehicleState finalState = world.Vehicles[0];
                    Assert.IsFalse(math.any(math.isnan(finalState.Position)));
                    Assert.Less(math.distance(settledPos, finalState.Position), 0.03f);
                    Assert.Less(math.length(finalState.LinearVelocity), 0.01f);
                }
            }
            finally { Object.DestroyImmediate(slopeGo); }
        }

        [Test]
        public void Solver_SteersRightSmoothly_AtHighwaySpeed()
        {
            var groundGo = new GameObject("Skidpad");
            try
            {
                groundGo.transform.position = new Vector3(0f, -0.5f, 0f);
                groundGo.AddComponent<BoxCollider>().size = new Vector3(500f, 1f, 500f);
                Physics.SyncTransforms();

                CreateStandardCarWorld(out var world, out var pipeline, new float3(0f, 0.65f, 0f), quaternion.identity, 9);
                using (world)
                using (pipeline)
                {
                    StepSimulation(world, pipeline, 1f / 60f, 30); // Settle

                    VehicleState v = world.Vehicles[0];
                    v.LinearVelocity = new float3(0f, 0f, 20f); // 72 km/h forward
                    world.Vehicles[0] = v;

                    world.DriverInputs[0] = new VehicleDriverInput { Steer = 0.6f, RequestedGear = -128 };
                    StepSimulation(world, pipeline, 1f / 60f, 60);

                    VehicleState after = world.Vehicles[0];
                    Assert.IsFalse(math.any(math.isnan(after.Position)));
                    Assert.Greater(after.Position.x, 2.0f); // Turned right toward +X
                    Assert.Greater(after.AngularVelocity.y, 0.05f);
                }
            }
            finally { Object.DestroyImmediate(groundGo); }
        }
    }
}
