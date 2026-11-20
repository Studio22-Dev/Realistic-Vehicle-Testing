using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Bridge;
using VehiclePhysics.Core;

namespace VehiclePhysics.Tests
{
    public class RaycastBridgeTests
    {
        private static VehicleSimulationWorld CreateTestWorld(out int vehIdx)
        {
            var world = new VehicleSimulationWorld(maxVehicles: 2, maxArchetypes: 1, maxSurfaceMaterials: 2);

            world.SurfaceMaterials[0] = new SurfaceMaterialData
            {
                StaticFriction = 1.0f,
                DynamicFriction = 0.85f,
                RollingResistance = 0.015f,
                TireGripMultiplier = 1.0f,
                MaterialTypeId = 0
            };
            world.SurfaceMaterials[1] = new SurfaceMaterialData
            {
                StaticFriction = 0.65f,
                DynamicFriction = 0.55f,
                RollingResistance = 0.045f,
                TireGripMultiplier = 1.0f,
                MaterialTypeId = 1
            };

            var wheels = new WheelConfig[]
            {
                new WheelConfig { MountPositionLocal = new float3(-0.8f, 0f,  1.3f), StrutAxisLocal = new float3(0, -1, 0), RestLengthM = 0.4f, UnloadedRadiusM = 0.33f, AxleIndex = 0, SideIndex = 0 },
                new WheelConfig { MountPositionLocal = new float3( 0.8f, 0f,  1.3f), StrutAxisLocal = new float3(0, -1, 0), RestLengthM = 0.4f, UnloadedRadiusM = 0.33f, AxleIndex = 0, SideIndex = 1 },
                new WheelConfig { MountPositionLocal = new float3(-0.8f, 0f, -1.3f), StrutAxisLocal = new float3(0, -1, 0), RestLengthM = 0.4f, UnloadedRadiusM = 0.33f, AxleIndex = 1, SideIndex = 0 },
                new WheelConfig { MountPositionLocal = new float3( 0.8f, 0f, -1.3f), StrutAxisLocal = new float3(0, -1, 0), RestLengthM = 0.4f, UnloadedRadiusM = 0.33f, AxleIndex = 1, SideIndex = 1 },
            };

            world.RegisterArchetype(
                0,
                new VehicleConfig { MassKg = 1400f, InertiaLocal = new float3(600, 2400, 2200), HasPowertrain = 1 },
                new PowertrainConfig { IdleOmega = 85f },
                wheels);

            vehIdx = world.SpawnVehicle(101, 0, new float3(0f, 0.55f, 0f), quaternion.identity, true, 9);
            return world;
        }

        [Test]
        public void RaycastBridge_DetectsGroundAndResolvesSurfaceMaterial()
        {
            var groundGo = new GameObject("TestGround");
            try
            {
                groundGo.transform.position = new Vector3(0f, -0.5f, 0f);
                var groundCol = groundGo.AddComponent<BoxCollider>();
                groundCol.size = new Vector3(50f, 1f, 50f); // Top plane at y = 0
                Physics.SyncTransforms();

                using var world = CreateTestWorld(out int vehIdx);
                using var pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

                pipeline.RegisterCollider(groundCol, new ColliderPhysicsMeta
                {
                    OwnerVehicleId = 0,
                    DynamicBodyIndex = -1,
                    SurfaceMaterialIndex = 1
                });

                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();

                int start = world.Vehicles[vehIdx].WheelStartIndex;
                for (int w = 0; w < 4; w++)
                {
                    ContactManifold manifold = world.ContactManifolds[start + w];
                    SurfaceMaterialData mat = world.SurfaceMaterials[manifold.PrimaryMaterialIndex];

                    Assert.AreEqual(1, manifold.IsGrounded);
                    Assert.AreEqual(1, manifold.ClusterBottom.IsValid);
                    Assert.AreEqual(1, manifold.ActiveClusterMask & 1);
                    Assert.AreEqual(0f, manifold.BlendedPoint.y, 1e-3f);
                    Assert.AreEqual(1f, manifold.BlendedNormal.y, 1e-3f);
                    Assert.AreEqual(0.65f, mat.StaticFriction, 1e-4f);
                    Assert.AreEqual(1, mat.MaterialTypeId);
                    Assert.AreEqual(1, world.WheelInputs[start + w].ContactSampleCount);
                }
            }
            finally
            {
                Object.DestroyImmediate(groundGo);
            }
        }

        [Test]
        public void RaycastBridge_IgnoresSelfChassisAndHitsGroundBelow()
        {
            var groundGo = new GameObject("TestGround");
            var chassisGo = new GameObject("SelfChassis");
            try
            {
                groundGo.transform.position = new Vector3(0f, -0.5f, 0f);
                var groundCol = groundGo.AddComponent<BoxCollider>();
                groundCol.size = new Vector3(50f, 1f, 50f);

                // Deliberately blocks the wheel rays at y = 0.25 (below mount y=0.55, above ground y=0)
                chassisGo.transform.position = new Vector3(0f, 0.25f, 0f);
                var selfCol = chassisGo.AddComponent<BoxCollider>();
                selfCol.size = new Vector3(3f, 0.1f, 4f);

                Physics.SyncTransforms();

                using var world = CreateTestWorld(out int vehIdx);
                using var pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

                pipeline.RegisterCollider(groundCol, new ColliderPhysicsMeta { OwnerVehicleId = 0, DynamicBodyIndex = -1, SurfaceMaterialIndex = 1 });
                pipeline.RegisterCollider(selfCol, new ColliderPhysicsMeta { OwnerVehicleId = 101, DynamicBodyIndex = -1, SurfaceMaterialIndex = 0 });

                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();

                int start = world.Vehicles[vehIdx].WheelStartIndex;
                for (int w = 0; w < 4; w++)
                {
                    ContactManifold manifold = world.ContactManifolds[start + w];
                    Assert.AreEqual(1, manifold.IsGrounded);
                    Assert.AreEqual(0f, manifold.BlendedPoint.y, 1e-3f);
                    Assert.AreEqual(1, world.SurfaceMaterials[manifold.PrimaryMaterialIndex].MaterialTypeId);
                }
            }
            finally
            {
                Object.DestroyImmediate(chassisGo);
                Object.DestroyImmediate(groundGo);
            }
        }

        [Test]
        public void RaycastBridge_ComputesMovingPlatformSurfaceVelocity()
        {
            var platformGo = new GameObject("MovingPlatform");
            try
            {
                platformGo.transform.position = new Vector3(0f, -0.5f, 0f);
                var platformCol = platformGo.AddComponent<BoxCollider>();
                platformCol.size = new Vector3(50f, 1f, 50f);
                Physics.SyncTransforms();

                using var world = CreateTestWorld(out int vehIdx);
                using var pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

                int dynIdx = pipeline.AddOrUpdateDynamicBody(-1, new DynamicBodyMotion
                {
                    WorldCenterOfMass = float3.zero,
                    LinearVelocity = new float3(10f, 0f, 0f),
                    AngularVelocity = new float3(0f, 2f, 0f)
                });

                pipeline.RegisterCollider(platformCol, new ColliderPhysicsMeta
                {
                    OwnerVehicleId = 0,
                    DynamicBodyIndex = dynIdx,
                    SurfaceMaterialIndex = 0
                });

                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();

                int start = world.Vehicles[vehIdx].WheelStartIndex;
                ContactManifold fl = world.ContactManifolds[start + 0]; // Contact around (-0.8, 0, +1.3)
                float3 expected = new float3(10f, 0f, 0f) + math.cross(new float3(0f, 2f, 0f), fl.BlendedPoint);

                Assert.AreNotEqual(0, fl.Flags & ManifoldFlags.TouchesDynamicBody);
                Assert.AreEqual(expected.x, fl.BlendedVelocity.x, 1e-3f);
                Assert.AreEqual(expected.y, fl.BlendedVelocity.y, 1e-3f);
                Assert.AreEqual(expected.z, fl.BlendedVelocity.z, 1e-3f);
                Assert.AreEqual(expected.x, fl.ClusterBottom.SurfaceVelocity.x, 1e-3f);
            }
            finally
            {
                Object.DestroyImmediate(platformGo);
            }
        }

        [Test]
        public void RaycastBridge_ClearsAirborneContactCleanly()
        {
            var groundGo = new GameObject("TestGround");
            try
            {
                groundGo.transform.position = new Vector3(0f, -0.5f, 0f);
                var groundCol = groundGo.AddComponent<BoxCollider>();
                groundCol.size = new Vector3(50f, 1f, 50f);
                Physics.SyncTransforms();

                using var world = CreateTestWorld(out int vehIdx);
                using var pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

                // Grounded tick
                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();
                int start = world.Vehicles[vehIdx].WheelStartIndex;
                Assert.AreEqual(1, world.ContactManifolds[start].IsGrounded);

                // Teleport high into the air
                VehicleState veh = world.Vehicles[vehIdx];
                veh.Position = new float3(0f, 50f, 0f);
                world.Vehicles[vehIdx] = veh;

                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();
                for (int w = 0; w < 4; w++)
                {
                    Assert.AreEqual(0, world.ContactManifolds[start + w].IsGrounded);
                    Assert.AreEqual(0, world.ContactManifolds[start + w].ActiveClusterMask);
                    Assert.AreEqual(0, world.WheelInputs[start + w].ContactSampleCount);
                }
            }
            finally
            {
                Object.DestroyImmediate(groundGo);
            }
        }

        [Test]
        public void RaycastBridge_DetachWheelClearsManifoldWithoutChangingVehicleCount()
        {
            var groundGo = new GameObject("TestGround");
            try
            {
                groundGo.transform.position = new Vector3(0f, -0.5f, 0f);
                var groundCol = groundGo.AddComponent<BoxCollider>();
                groundCol.size = new Vector3(50f, 1f, 50f);
                Physics.SyncTransforms();

                using var world = CreateTestWorld(out int vehIdx);
                using var pipeline = new VehicleRaycastPipeline(world.MaxTotalWheelSlots, maxHitsPerRay: 3);

                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();
                int start = world.Vehicles[vehIdx].WheelStartIndex;
                Assert.AreEqual(1, world.ContactManifolds[start + 2].IsGrounded);

                world.SetWheelDetached(vehIdx, 2, true);
                pipeline.ScheduleQueries(world, Physics.defaultPhysicsScene, ~0).Complete();

                Assert.AreEqual(0, world.ContactManifolds[start + 2].IsGrounded);
                Assert.AreEqual(0, world.WheelInputs[start + 2].ContactSampleCount);
                Assert.AreEqual(1, world.ContactManifolds[start + 0].IsGrounded);
            }
            finally
            {
                Object.DestroyImmediate(groundGo);
            }
        }
    }
}
