using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Core;

namespace VehiclePhysics.Bridge
{
    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct PrepareWheelRaycastsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> ActiveWheelSlots;
        [ReadOnly] public NativeArray<int> ActiveWheelVehicleIndices;
        [ReadOnly] public NativeArray<int> ActiveWheelConfigIndices;

        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<VehicleConfig> VehicleConfigs;
        [ReadOnly] public NativeArray<WheelConfig> WheelConfigs;
        [ReadOnly] public NativeArray<WheelState> Wheels;

        public PhysicsScene PhysicsScene;
        public QueryParameters QueryParams;
        public float LookaheadMarginM;

        [NativeDisableParallelForRestriction] public NativeArray<CapsulecastCommand> CapsuleCommands;
        [NativeDisableParallelForRestriction] public NativeArray<SpherecastCommand> SphereCommands;
        public NativeArray<WheelRayMeta> WheelMeta;

        public void Execute(int idx)
        {
            int slot = ActiveWheelSlots[idx];
            int vehIdx = ActiveWheelVehicleIndices[idx];
            int cfgIdx = ActiveWheelConfigIndices[idx];

            VehicleState veh = Vehicles[vehIdx];
            VehicleConfig vCfg = VehicleConfigs[veh.ConfigIndex];
            WheelConfig wCfg = WheelConfigs[cfgIdx];
            WheelState ws = Wheels[slot];

            int capBase = idx * VehicleRaycastPipeline.MaxCapsulesPerWheel;
            int sphBase = idx * VehicleRaycastPipeline.MaxSpheresPerWheel;

            // Sleeping / LOD3 vehicles skip all PhysX casts
            if (veh.LodTier >= 3 || (veh.TierFlags & VehicleTierFlags.IsSleeping) != 0)
            {
                for (int i = 0; i < VehicleRaycastPipeline.MaxCapsulesPerWheel; i++)
                    CapsuleCommands[capBase + i] = default;
                for (int i = 0; i < VehicleRaycastPipeline.MaxSpheresPerWheel; i++)
                    SphereCommands[sphBase + i] = default;

                WheelMeta[idx] = default;
                return;
            }

            float3 mountRelCoM = wCfg.MountPositionLocal - vCfg.CenterOfMassLocalOffset;
            float3 worldMount = veh.Position + math.rotate(veh.Rotation, mountRelCoM);
            float3 downDir = math.normalizesafe(
                math.rotate(veh.Rotation, wCfg.StrutAxisLocal),
                new float3(0f, -1f, 0f));
            float3 upDir = -downDir;

            // Align probes to live wheel steering angle (matches TireSolver sign: +steer -> Right/+X)
            math.sincos(ws.SteerAngleRad, out float sinSteer, out float cosSteer);
            float3 steerFwdHint = math.rotate(veh.Rotation, new float3(sinSteer, 0f, cosSteer));
            float3 steerRightHint = math.rotate(veh.Rotation, new float3(cosSteer, 0f, -sinSteer));

            float3 rightDir = math.normalizesafe(math.cross(upDir, steerFwdHint), steerRightHint);
            float3 fwdDir = math.normalizesafe(math.cross(rightDir, upDir), steerFwdHint);
            float outSign = wCfg.SideIndex == 0 ? -1f : 1f;
            float3 outDir = rightDir * outSign;

            float R = math.max(wCfg.UnloadedRadiusM, 0.10f);
            float halfW = (wCfg.TireWidthM > 0f ? wCfg.TireWidthM : 0.235f) * 0.5f;

            byte lod = veh.LodTier;
            float targetCapR = lod == 0 ? 0.065f : (lod == 1 ? 0.085f : 0.110f);
            float rCap = math.min(targetCapR, math.min(R * 0.42f, halfW * 0.90f));
            float capHalfSpan = math.max(0.005f, halfW - rCap);

            float centerRayDist = wCfg.RestLengthM + R + LookaheadMarginM;
            float hubDropCurrent = math.max(0f, wCfg.RestLengthM - ws.SuspensionCompression);
            float3 hubEst = worldMount + downDir * hubDropCurrent;

            // --- Capsule 0 (All LODs): Center vertical sweep from strut mount ---
            // Offset start upward by rCap so outer bottom skin starts at worldMount (no initial overlap).
            float3 c0Origin = worldMount - downDir * rCap;
            CapsuleCommands[capBase + 0] = new CapsulecastCommand(
                PhysicsScene,
                c0Origin - outDir * capHalfSpan,
                c0Origin + outDir * capHalfSpan,
                rCap,
                downDir,
                QueryParams,
                centerRayDist);

            // --- Arc Capsules (C1 Leading / C2 Trailing) ---
            float arcBackoff = math.max(rCap + 0.06f, 0.45f * R);
            float arcSweepDist = arcBackoff + (R - rCap) + LookaheadMarginM;

            byte capCount;
            if (lod == 0)
            {
                capCount = 9;
                WriteArcCapsule(capBase + 1, hubEst, fwdDir, downDir, outDir,  18f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 2, hubEst, fwdDir, downDir, outDir,  36f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 3, hubEst, fwdDir, downDir, outDir,  54f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 4, hubEst, fwdDir, downDir, outDir,  72f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 5, hubEst, fwdDir, downDir, outDir, -18f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 6, hubEst, fwdDir, downDir, outDir, -36f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 7, hubEst, fwdDir, downDir, outDir, -54f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 8, hubEst, fwdDir, downDir, outDir, -72f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
            }
            else if (lod == 1)
            {
                capCount = 5;
                WriteArcCapsule(capBase + 1, hubEst, fwdDir, downDir, outDir,  24f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 2, hubEst, fwdDir, downDir, outDir,  48f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 3, hubEst, fwdDir, downDir, outDir, -24f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 4, hubEst, fwdDir, downDir, outDir, -48f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                for (int i = 5; i < VehicleRaycastPipeline.MaxCapsulesPerWheel; i++)
                    CapsuleCommands[capBase + i] = default;
            }
            else
            {
                capCount = 3;
                WriteArcCapsule(capBase + 1, hubEst, fwdDir, downDir, outDir,  36f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                WriteArcCapsule(capBase + 2, hubEst, fwdDir, downDir, outDir, -36f, rCap, capHalfSpan, arcBackoff, arcSweepDist);
                for (int i = 3; i < VehicleRaycastPipeline.MaxCapsulesPerWheel; i++)
                    CapsuleCommands[capBase + i] = default;
            }

            // --- Sidewall / Shoulder Spheres (C3) ---
            float rSide = math.min(R * 0.28f, halfW * 0.85f);
            float sideBackoff = rSide + 0.05f;
            float sideSweep = sideBackoff + (halfW - rSide) + LookaheadMarginM;
            byte sphCount = 0;

            if (lod <= 1)
            {
                float3 sideOutDir = math.normalizesafe(0.95f * outDir + 0.31f * downDir, outDir);
                SphereCommands[sphBase + 0] = new SpherecastCommand(
                    PhysicsScene, hubEst - sideOutDir * sideBackoff, rSide, sideOutDir, QueryParams, sideSweep);
                sphCount = 1;
            }
            else
            {
                SphereCommands[sphBase + 0] = default;
            }

            if (lod == 0)
            {
                float3 sideInDir = math.normalizesafe(-0.95f * outDir + 0.31f * downDir, -outDir);
                SphereCommands[sphBase + 1] = new SpherecastCommand(
                    PhysicsScene, hubEst - sideInDir * sideBackoff, rSide, sideInDir, QueryParams, sideSweep);
                sphCount = 2;
            }
            else
            {
                SphereCommands[sphBase + 1] = default;
            }

            WheelMeta[idx] = new WheelRayMeta
            {
                HubEstimate = hubEst,
                CapsuleRadiusM = rCap,
                WheelUp = upDir,
                ArcBackoffM = arcBackoff,
                WheelOutward = outDir,
                SideRadiusM = rSide,
                WheelForward = fwdDir,
                SideBackoffM = sideBackoff,
                HubDropCurrentM = hubDropCurrent,
                HalfWidthM = halfW,
                CenterRayDistM = centerRayDist,
                CapsuleCount = capCount,
                SphereCount = sphCount,
                LodTier = lod
            };
        }

        private void WriteArcCapsule(
            int cmdIdx,
            float3 hubEst,
            float3 fwdDir,
            float3 downDir,
            float3 outDir,
            float thetaDeg,
            float rCap,
            float halfSpan,
            float backoff,
            float sweepDist)
        {
            math.sincos(math.radians(thetaDeg), out float s, out float c);
            float3 dir = math.normalizesafe(fwdDir * s + downDir * c, downDir);
            float3 origin = hubEst - dir * backoff;
            CapsuleCommands[cmdIdx] = new CapsulecastCommand(
                PhysicsScene,
                origin - outDir * halfSpan,
                origin + outDir * halfSpan,
                rCap,
                dir,
                QueryParams,
                sweepDist);
        }
    }
}
