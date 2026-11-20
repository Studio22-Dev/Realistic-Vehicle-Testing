using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using VehiclePhysics.Core;

namespace VehiclePhysics.Tests
{
    public class VehicleLayoutValidationTests
    {
        [BurstCompile(CompileSynchronously = true)]
        public struct TorqueLutBurstSmokeJob : IJob
        {
            public PowertrainConfig Config;
            public float TestOmega;
            public NativeArray<float> OutTorque;

            public void Execute()
            {
                OutTorque[0] = Config.EvaluateNetEngineTorque(TestOmega, 1.0f);
            }
        }

        [Test]
        public void StructSizes_MatchExactCacheAndSimdLayouts()
        {
            Assert.AreEqual(128, UnsafeUtility.SizeOf<VehicleState>());
            Assert.AreEqual(64, UnsafeUtility.SizeOf<PowertrainState>());
            Assert.AreEqual(80, UnsafeUtility.SizeOf<WheelState>());
            Assert.AreEqual(32, UnsafeUtility.SizeOf<WheelInput>());
            Assert.AreEqual(64, UnsafeUtility.SizeOf<WheelContactSample>());
            Assert.AreEqual(64, UnsafeUtility.SizeOf<WheelContact>());
            Assert.AreEqual(32, UnsafeUtility.SizeOf<VehicleDriverInput>());
            Assert.AreEqual(64, UnsafeUtility.SizeOf<VehicleRenderPose>());
            Assert.AreEqual(112, UnsafeUtility.SizeOf<VehicleConfig>());
            Assert.AreEqual(208, UnsafeUtility.SizeOf<PowertrainConfig>());
            Assert.AreEqual(112, UnsafeUtility.SizeOf<WheelConfig>());
            Assert.AreEqual(32, UnsafeUtility.SizeOf<SurfaceMaterialData>());
        }

        [Test]
        public void AllStructs_AreBlittable()
        {
            Assert.IsTrue(UnsafeUtility.IsBlittable<VehicleState>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<PowertrainState>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<WheelState>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<WheelInput>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<WheelContactSample>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<WheelContact>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<VehicleDriverInput>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<VehicleRenderPose>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<VehicleConfig>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<PowertrainConfig>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<WheelConfig>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<SurfaceMaterialData>());
        }

        [Test]
        public void VectorFields_Are16ByteAligned()
        {
            AssertAligned16<VehicleState>(nameof(VehicleState.Rotation));
            AssertAligned16<VehicleState>(nameof(VehicleState.Position));
            AssertAligned16<VehicleState>(nameof(VehicleState.LinearVelocity));
            AssertAligned16<VehicleState>(nameof(VehicleState.AngularVelocity));
            AssertAligned16<VehicleState>(nameof(VehicleState.InverseInertiaLocal));
            AssertAligned16<VehicleState>(nameof(VehicleState.ExternalImpulse));
            AssertAligned16<VehicleState>(nameof(VehicleState.ExternalAngularImpulse));
            AssertAligned16<VehicleState>(nameof(VehicleState.LocalAcceleration));

            AssertAligned16<WheelState>(nameof(WheelState.LastWorldForce));
            AssertAligned16<WheelContact>(nameof(WheelContact.PlanePoint));
            AssertAligned16<WheelContact>(nameof(WheelContact.PlaneNormal));
            AssertAligned16<WheelContact>(nameof(WheelContact.SurfaceVelocity));
            AssertAligned16<VehicleRenderPose>(nameof(VehicleRenderPose.PreviousRotation));
            AssertAligned16<VehicleRenderPose>(nameof(VehicleRenderPose.CurrentRotation));
        }

        [Test]
        public void SimulationWorld_SupportsMixedCarAnd18WheelerCabTrailer()
        {
            // Capacity for 1 car (4) + 1 cab (10->12 aligned) + 1 trailer (8) = 24 wheel slots
            using var world = new VehicleSimulationWorld(
                maxVehicles: 4,
                maxTotalWheelSlots: 28,
                maxArchetypes: 3,
                maxArchetypeWheelConfigs: 22,
                maxSurfaceMaterials: 4);

            var carWheels = new WheelConfig[4];
            var cabWheels = new WheelConfig[10];
            var trailerWheels = new WheelConfig[8];
            for (int i = 0; i < 4; i++) carWheels[i] = new WheelConfig { UnloadedRadiusM = 0.33f };
            for (int i = 0; i < 10; i++) cabWheels[i] = new WheelConfig { UnloadedRadiusM = 0.52f };
            for (int i = 0; i < 8; i++) trailerWheels[i] = new WheelConfig { UnloadedRadiusM = 0.52f };

            world.RegisterArchetype(
                0,
                new VehicleConfig { MassKg = 1400f, InertiaLocal = new float3(600, 2400, 2200), HasPowertrain = 1 },
                new PowertrainConfig { IdleOmega = 85f },
                carWheels);

            world.RegisterArchetype(
                1,
                new VehicleConfig { MassKg = 8500f, InertiaLocal = new float3(8000, 28000, 26000), HasPowertrain = 1 },
                new PowertrainConfig { IdleOmega = 65f },
                cabWheels);

            world.RegisterArchetype(
                2,
                new VehicleConfig { MassKg = 24000f, InertiaLocal = new float3(22000, 95000, 90000), HasPowertrain = 0 },
                default,
                trailerWheels);

            int carIdx = world.SpawnVehicle(1, 0, float3.zero, quaternion.identity, true, 9);
            int cabIdx = world.SpawnVehicle(2, 1, new float3(10, 0, 0), quaternion.identity, false, 2);
            int trlIdx = world.SpawnVehicle(3, 2, new float3(10, 0, -10), quaternion.identity, false, 2);

            Assert.AreEqual(0, world.Vehicles[carIdx].WheelStartIndex);
            Assert.AreEqual(4, world.Vehicles[carIdx].WheelCount);

            Assert.AreEqual(4, world.Vehicles[cabIdx].WheelStartIndex);
            Assert.AreEqual(10, world.Vehicles[cabIdx].WheelCount);

            // 10 wheels aligned up to 12, so trailer starts at 4 + 12 = 16
            Assert.AreEqual(16, world.Vehicles[trlIdx].WheelStartIndex);
            Assert.AreEqual(8, world.Vehicles[trlIdx].WheelCount);
            Assert.AreEqual(24, world.ActiveWheelSlotCursor);

            // Verify every vehicle's wheel block starts on an exact 64-byte cache-line boundary
            int wheelSize = UnsafeUtility.SizeOf<WheelState>();
            Assert.AreEqual(0, world.Vehicles[carIdx].WheelStartIndex * wheelSize % 64);
            Assert.AreEqual(0, world.Vehicles[cabIdx].WheelStartIndex * wheelSize % 64);
            Assert.AreEqual(0, world.Vehicles[trlIdx].WheelStartIndex * wheelSize % 64);
        }

        [Test]
        public void TorqueLutJob_CompilesWithBurst()
        {
            using var outTorque = new NativeArray<float>(1, Allocator.TempJob);

            var job = new TorqueLutBurstSmokeJob
            {
                Config = new PowertrainConfig
                {
                    CurveMinOmega = 100f,
                    CurveMaxOmega = 700f,
                    WotTorque0 = new float4(200, 250, 300, 340),
                    WotTorque1 = new float4(360, 370, 375, 375),
                    WotTorque2 = new float4(370, 360, 345, 330),
                    WotTorque3 = new float4(310, 290, 265, 240)
                },
                TestOmega = 340f,
                OutTorque = outTorque
            };

            job.Run();
            Assert.Greater(outTorque[0], 300f);
        }

        private static void AssertAligned16<T>(string fieldName) where T : struct
        {
            int offset = UnsafeUtility.GetFieldOffset(typeof(T).GetField(fieldName));
            Assert.AreEqual(0, offset % 16, $"{typeof(T).Name}.{fieldName} offset {offset} is not 16-byte aligned.");
        }
    }
}
