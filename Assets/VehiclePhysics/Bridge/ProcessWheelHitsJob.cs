using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Core;

namespace VehiclePhysics.Bridge
{
    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct ProcessWheelHitsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> ActiveWheelSlots;
        [ReadOnly] public NativeArray<int> ActiveWheelVehicleIndices;
        [ReadOnly] public NativeArray<int> ActiveWheelConfigIndices;

        [ReadOnly] public NativeArray<VehicleState> Vehicles;
        [ReadOnly] public NativeArray<WheelConfig> WheelConfigs;
        [ReadOnly] public NativeArray<SurfaceMaterialData> SurfaceMaterials;
        [ReadOnly] public NativeArray<WheelRayMeta> WheelMeta;

        [ReadOnly] public NativeArray<RaycastHit> CapsuleHits;
        [ReadOnly] public NativeArray<RaycastHit> SphereHits;
        public int MaxHitsPerCast;

        [ReadOnly] public NativeHashMap<ColliderIdKey, ColliderPhysicsMeta>.ReadOnly ColliderMetaMap;
        [ReadOnly] public NativeArray<DynamicBodyMotion> DynamicBodies;

        public float LookaheadMarginM;
        public float DefaultPatchLengthM;
        public int MaxSamplesPerWheel;

        [NativeDisableParallelForRestriction] public NativeArray<WheelContactSample> ContactSamples;
        [NativeDisableParallelForRestriction] public NativeArray<ContactManifold> ContactManifolds;
        [NativeDisableParallelForRestriction] public NativeArray<TireCarcassState> TireCarcassStates;
        [NativeDisableParallelForRestriction] public NativeArray<WheelInput> WheelInputs;

        public void Execute(int idx)
        {
            int slot = ActiveWheelSlots[idx];
            int vehIdx = ActiveWheelVehicleIndices[idx];
            int cfgIdx = ActiveWheelConfigIndices[idx];

            VehicleState veh = Vehicles[vehIdx];
            WheelConfig wCfg = WheelConfigs[cfgIdx];
            WheelRayMeta meta = WheelMeta[idx];

            WheelInput wIn = WheelInputs[slot];
            int sampleBase = wIn.ContactSampleStartIndex;

            if (veh.LodTier >= 3 || (veh.TierFlags & VehicleTierFlags.IsSleeping) != 0 || meta.CapsuleCount == 0)
            {
                ContactManifolds[slot] = default;
                wIn.ContactSampleCount = 0;
                WheelInputs[slot] = wIn;

                TireCarcassState cSleep = TireCarcassStates[slot];
                cSleep.ActiveRawProbeCount = 0;
                TireCarcassStates[slot] = cSleep;
                return;
            }

            int capBase = idx * VehicleRaycastPipeline.MaxCapsulesPerWheel;
            int sphBase = idx * VehicleRaycastPipeline.MaxSpheresPerWheel;

            float3 downDir = -meta.WheelUp;
            float R = math.max(wCfg.UnloadedRadiusM, 0.10f);

            ContactManifold manifold = default;
            int rawWriteCount = 0;

            // Track climbable support accumulators (C1/C2) vs non-climbable horizontal wall impacts
            float3 leadPosSum = float3.zero, leadNormSum = float3.zero, leadVelSum = float3.zero;
            float leadWSum = 0f, leadMaxPen = -1e6f;
            float leadFricSum = 0f;
            int leadCount = 0;
            byte leadMat = 0, leadFlags = 0;
            int leadDynIdx = -1, leadColId = 0;

            float3 trailPosSum = float3.zero, trailNormSum = float3.zero, trailVelSum = float3.zero;
            float trailWSum = 0f, trailMaxPen = -1e6f;
            float trailFricSum = 0f;
            int trailCount = 0;
            byte trailMat = 0, trailFlags = 0;
            int trailDynIdx = -1, trailColId = 0;

            bool hasWallImpact = false;
            float wallMaxPen = -1f;
            float3 wallPoint = float3.zero, wallNormal = float3.zero, wallVel = float3.zero;
            byte wallZone = 1, wallMat = 0;
            float wallFric = 1f;
            int wallDynIdx = -1, wallColId = 0;

            // --- 1. Process Capsule 0 (Bottom Patch C0) ---
            int capCmd0 = capBase + 0;
            float3 c0Origin = meta.HubEstimate + meta.WheelUp * meta.HubDropCurrentM - downDir * meta.CapsuleRadiusM;
            if (TryFindClosestValidHit(CapsuleHits, capCmd0, c0Origin, downDir, meta.CapsuleRadiusM, meta.WheelUp, veh.VehicleId,
                                    out float3 p0, out float3 n0, out float dist0, out bool overlap0,
                                    out byte mat0, out float fric0, out int dyn0, out float3 vel0, out int colId0))
            {
                float dHub0 = overlap0 ? 0f : (dist0 - meta.HubDropCurrentM);
                float pen0 = R - dHub0;

                if (pen0 >= -LookaheadMarginM)
                {
                    manifold.ClusterBottom = MakeCluster(p0, pen0, n0, vel0, colId0, dyn0, fric0, mat0, 1,
                        dyn0 >= 0 ? ManifoldClusterFlags.HitDynamicBody : ManifoldClusterFlags.None);
                    manifold.ActiveClusterMask |= (1 << 0);
                    if (dyn0 >= 0) manifold.Flags |= ManifoldFlags.TouchesDynamicBody;

                    AppendRawSample(sampleBase, ref rawWriteCount, p0, pen0, n0, vel0, 0f,
                        mat0, fric0, WheelProbeZones.Bottom);
                }
            }

            // --- 2. Process Angled Arc Capsules (Leading C1 & Trailing C2) ---
            for (int c = 1; c < meta.CapsuleCount; c++)
            {
                float thetaDeg = GetArcAngleDeg(meta.LodTier, c);
                float thetaRad = math.radians(thetaDeg);
                math.sincos(thetaRad, out float sinT, out float cosT);
                float3 probeDir = math.normalize(meta.WheelForward * sinT + downDir * cosT);
                float3 probeOrigin = meta.HubEstimate - probeDir * meta.ArcBackoffM;

                if (!TryFindClosestValidHit(CapsuleHits, capBase + c, probeOrigin, probeDir, meta.CapsuleRadiusM, meta.WheelUp, veh.VehicleId,
                                    out float3 pc, out float3 nc, out float distc, out bool overlapc,
                                    out byte matc, out float fricc, out int dync, out float3 velc, out int colIdc))
                    continue;

                float dHub = overlapc ? 0f : (distc - meta.ArcBackoffM + meta.CapsuleRadiusM);
                float pen = R - dHub;
                if (pen <= 0.001f)
                    continue;

                // Separate climbable curb/slope contacts from solid vertical walls
                bool isClimbable = math.dot(nc, meta.WheelUp) >= 0.35f;
                if (!isClimbable)
                {
                    float3 radialDir = math.normalizesafe(meta.HubEstimate - pc, nc);
                    if (math.dot(radialDir, meta.WheelUp) > 0.18f && pc.y < meta.HubEstimate.y - 0.08f * R)
                    {
                        nc = math.normalize(math.lerp(nc, radialDir, 0.75f));
                        isClimbable = math.dot(nc, meta.WheelUp) >= 0.30f;
                    }
                }

                byte zone = thetaDeg >= 0f ? WheelProbeZones.Leading : WheelProbeZones.Trailing;
                AppendRawSample(sampleBase, ref rawWriteCount, pc, pen, nc, velc, thetaRad,
                    matc, fricc, zone);

                if (!isClimbable && pen > 0f)
                {
                    hasWallImpact = true;
                    if (pen > wallMaxPen)
                    {
                        wallMaxPen = pen;
                        wallPoint = pc;
                        wallNormal = nc;
                        wallVel = velc;
                        wallZone = zone;
                        wallMat = matc;
                        wallFric = fricc;
                        wallDynIdx = dync;
                        wallColId = colIdc;
                    }
                    continue;
                }

                float wProbe = 1.0f + math.max(0f, pen) * 35.0f;
                if (thetaDeg >= 0f)
                {
                    leadPosSum += pc * wProbe;
                    leadNormSum += nc * wProbe;
                    leadVelSum += velc * wProbe;
                    leadFricSum += fricc * wProbe;
                    leadWSum += wProbe;
                    leadCount++;
                    if (pen > leadMaxPen)
                    {
                        leadMaxPen = pen;
                        leadMat = matc;
                        leadDynIdx = dync;
                        leadColId = colIdc;
                    }
                    if (dync >= 0) leadFlags |= ManifoldClusterFlags.HitDynamicBody;
                }
                else
                {
                    trailPosSum += pc * wProbe;
                    trailNormSum += nc * wProbe;
                    trailVelSum += velc * wProbe;
                    trailFricSum += fricc * wProbe;
                    trailWSum += wProbe;
                    trailCount++;
                    if (pen > trailMaxPen)
                    {
                        trailMaxPen = pen;
                        trailMat = matc;
                        trailDynIdx = dync;
                        trailColId = colIdc;
                    }
                    if (dync >= 0) trailFlags |= ManifoldClusterFlags.HitDynamicBody;
                }
            }

            if (leadCount > 0 && leadWSum > 1e-5f)
            {
                float invL = 1.0f / leadWSum;
                manifold.ClusterLeading = MakeCluster(
                    leadPosSum * invL, leadMaxPen, math.normalizesafe(leadNormSum, meta.WheelUp),
                    leadVelSum * invL, leadColId, leadDynIdx, leadFricSum * invL, leadMat,
                    (byte)math.min(leadCount, 255), leadFlags);
                manifold.ActiveClusterMask |= (1 << 1);
                if ((leadFlags & ManifoldClusterFlags.HitDynamicBody) != 0)
                    manifold.Flags |= ManifoldFlags.TouchesDynamicBody;
            }

            if (trailCount > 0 && trailWSum > 1e-5f)
            {
                float invT = 1.0f / trailWSum;
                manifold.ClusterTrailing = MakeCluster(
                    trailPosSum * invT, trailMaxPen, math.normalizesafe(trailNormSum, meta.WheelUp),
                    trailVelSum * invT, trailColId, trailDynIdx, trailFricSum * invT, trailMat,
                    (byte)math.min(trailCount, 255), trailFlags);
                manifold.ActiveClusterMask |= (1 << 2);
                if ((trailFlags & ManifoldClusterFlags.HitDynamicBody) != 0)
                    manifold.Flags |= ManifoldFlags.TouchesDynamicBody;
            }

            if (hasWallImpact)
            {
                manifold.Flags |= ManifoldFlags.HasWallImpact;
                byte wallFlags = wallDynIdx >= 0 ? ManifoldClusterFlags.HitDynamicBody : ManifoldClusterFlags.None;
                if (wallDynIdx >= 0) manifold.Flags |= ManifoldFlags.TouchesDynamicBody;

                if (wallZone == WheelProbeZones.Leading && manifold.ClusterLeading.IsValid == 0)
                {
                    manifold.ClusterLeading = MakeCluster(
                        wallPoint, wallMaxPen, wallNormal, wallVel, wallColId, wallDynIdx, wallFric, wallMat, 1, wallFlags);
                    manifold.ActiveClusterMask |= (1 << 1);
                }
                else if (wallZone == WheelProbeZones.Trailing && manifold.ClusterTrailing.IsValid == 0)
                {
                    manifold.ClusterTrailing = MakeCluster(
                        wallPoint, wallMaxPen, wallNormal, wallVel, wallColId, wallDynIdx, wallFric, wallMat, 1, wallFlags);
                    manifold.ActiveClusterMask |= (1 << 2);
                }
            }

            // --- 3. Process Sidewall Spheres (C3) ---
            float bestSidePen = -1f;
            ManifoldContactPoint bestC3 = default;

            for (int sIdx = 0; sIdx < meta.SphereCount; sIdx++)
            {
                float sign = (sIdx == 0) ? 1f : -1f; // 0 = outer sidewall, 1 = inner sidewall
                float3 sideDir = math.normalize(sign * 0.95f * meta.WheelOutward + 0.31f * downDir);
                float3 sideOrigin = meta.HubEstimate - sideDir * meta.SideBackoffM;

                if (!TryFindClosestValidHit(SphereHits, sphBase + sIdx, sideOrigin, sideDir, meta.SideRadiusM, meta.WheelUp, veh.VehicleId,
                                    out float3 ps, out float3 ns, out float dists, out bool overlaps,
                                    out byte mats, out float frics, out int dyns, out float3 vels, out int colIds))
                    continue;

                if (math.dot(ns, meta.WheelOutward) * sign > -0.25f)
                    continue;

                float dLat = overlaps ? 0f : (dists - meta.SideBackoffM + meta.SideRadiusM);
                float penSide = meta.HalfWidthM - dLat;
                if (penSide <= 0f)
                    continue;

                AppendRawSample(sampleBase, ref rawWriteCount, ps, penSide, ns, vels,
                    sign * (math.PI * 0.5f), mats, frics, WheelProbeZones.Sidewall);

                if (penSide > bestSidePen)
                {
                    bestSidePen = penSide;
                    byte c3Flags = ManifoldClusterFlags.HitRimSidewall;
                    if (dyns >= 0) c3Flags |= ManifoldClusterFlags.HitDynamicBody;
                    bestC3 = MakeCluster(ps, penSide, ns, vels, colIds, dyns, frics, mats, 1, c3Flags);
                }
            }

            if (bestC3.IsValid != 0)
            {
                manifold.ClusterSidewall = bestC3;
                manifold.ActiveClusterMask |= (1 << 3);
                if (bestC3.DynamicBodyIndex >= 0) manifold.Flags |= ManifoldFlags.TouchesDynamicBody;
            }

            // --- 4. Aggregate Support Patch Summary (C0 + climbable C1/C2 ONLY; never C3 or horizontal walls) ---
            float w0 = manifold.ClusterBottom.IsValid != 0
                ? (1.0f + math.max(0f, manifold.ClusterBottom.PenetrationM) * 20.0f) : 0f;
            float w1 = (manifold.ClusterLeading.IsValid != 0 && math.dot(manifold.ClusterLeading.WorldNormal, meta.WheelUp) >= 0.30f)
                ? math.max(0.05f, math.max(0f, manifold.ClusterLeading.PenetrationM) * 30.0f) : 0f;
            float w2 = (manifold.ClusterTrailing.IsValid != 0 && math.dot(manifold.ClusterTrailing.WorldNormal, meta.WheelUp) >= 0.30f)
                ? math.max(0.05f, math.max(0f, manifold.ClusterTrailing.PenetrationM) * 30.0f) : 0f;

            float wSum = w0 + w1 + w2;
            if (wSum > 1e-5f)
            {
                float invSum = 1.0f / wSum;
                w0 *= invSum;
                w1 *= invSum;
                w2 *= invSum;

                manifold.ClusterBottom.ContactWeight = w0;
                manifold.ClusterLeading.ContactWeight = w1;
                manifold.ClusterTrailing.ContactWeight = w2;

                manifold.BlendedPoint =
                    manifold.ClusterBottom.WorldPoint   * w0 +
                    manifold.ClusterLeading.WorldPoint  * w1 +
                    manifold.ClusterTrailing.WorldPoint * w2;

                manifold.BlendedNormal = math.normalizesafe(
                    manifold.ClusterBottom.WorldNormal   * w0 +
                    manifold.ClusterLeading.WorldNormal  * w1 +
                    manifold.ClusterTrailing.WorldNormal * w2,
                    meta.WheelUp);

                manifold.BlendedVelocity =
                    manifold.ClusterBottom.SurfaceVelocity   * w0 +
                    manifold.ClusterLeading.SurfaceVelocity  * w1 +
                    manifold.ClusterTrailing.SurfaceVelocity * w2;

                manifold.BlendedFrictionScale =
                    manifold.ClusterBottom.FrictionScale   * w0 +
                    manifold.ClusterLeading.FrictionScale  * w1 +
                    manifold.ClusterTrailing.FrictionScale * w2;

                manifold.PrimaryMaterialIndex = w0 >= math.max(w1, w2)
                    ? manifold.ClusterBottom.SurfaceMaterialIndex
                    : (w1 >= w2 ? manifold.ClusterLeading.SurfaceMaterialIndex : manifold.ClusterTrailing.SurfaceMaterialIndex);

                float activeSpan = (w0 > 0f ? 1f : 0f) + (w1 > 0f ? 0.5f : 0f) + (w2 > 0f ? 0.5f : 0f);
                manifold.ContactPatchLengthM = DefaultPatchLengthM * math.max(1f, activeSpan);
                manifold.IsGrounded = 1;
            }
            else
            {
                manifold.IsGrounded = 0;
            }

            ContactManifolds[slot] = manifold;

            wIn.ContactSampleCount = (byte)rawWriteCount;
            WheelInputs[slot] = wIn;

            TireCarcassState carcass = TireCarcassStates[slot];
            carcass.ActiveRawProbeCount = (byte)rawWriteCount;
            TireCarcassStates[slot] = carcass;
        }

        private static float GetArcAngleDeg(byte lod, int c)
        {
            if (lod == 0)
            {
                return c switch
                {
                    1 => 18f, 2 => 36f, 3 => 54f, 4 => 72f,
                    5 => -18f, 6 => -36f, 7 => -54f, _ => -72f
                };
            }
            if (lod == 1)
            {
                return c switch { 1 => 24f, 2 => 48f, 3 => -24f, _ => -48f };
            }
            return c == 1 ? 36f : -36f;
        }

        private bool TryFindClosestValidHit(
            NativeArray<RaycastHit> hitsBuffer,
            int commandIndex,
            float3 probeOrigin,
            float3 probeDir,
            float probeRadius,
            float3 fallbackNormal,
            uint vehicleId,
            out float3 bestPoint,
            out float3 bestNormal,
            out float bestDistance,
            out bool bestIsInitialOverlap,
            out byte bestMatIndex,
            out float bestFrictionScale,
            out int bestDynBodyIdx,
            out float3 bestSurfaceVel,
            out int bestColliderId)
        {
            bool found = false;
            bestDistance = float.MaxValue;
            bestPoint = float3.zero;
            bestNormal = fallbackNormal;
            bestIsInitialOverlap = false;
            bestMatIndex = 0;
            bestFrictionScale = 1.0f;
            bestDynBodyIdx = -1;
            bestSurfaceVel = float3.zero;
            bestColliderId = 0;

            int baseHitIdx = commandIndex * MaxHitsPerCast;
            for (int h = 0; h < MaxHitsPerCast; h++)
            {
                RaycastHit hit = hitsBuffer[baseHitIdx + h];
                ColliderIdKey key = ColliderIdKey.FromHit(in hit);
                if (!key.IsValid)
                    break; // First invalid collider terminates hits for this command

                if (math.lengthsq((float3)hit.normal) < 0.25f)
                    continue;

                float dist = hit.distance;
                bool isOverlap = (dist <= 1e-5f) && (math.lengthsq((float3)hit.point) < 1e-8f);
                float3 pt = isOverlap ? (probeOrigin + probeDir * probeRadius) : (float3)hit.point;
                float3 nrm = isOverlap ? math.normalizesafe(-probeDir, fallbackNormal) : math.normalize((float3)hit.normal);

                if (!ResolveHitMetadata(key, pt, vehicleId, out byte mat, out float fric, out int dyn, out float3 vel, out int colId))
                    continue;

                if (!found || dist < bestDistance)
                {
                    found = true;
                    bestDistance = dist;
                    bestPoint = pt;
                    bestNormal = nrm;
                    bestIsInitialOverlap = isOverlap;
                    bestMatIndex = mat;
                    bestFrictionScale = fric;
                    bestDynBodyIdx = dyn;
                    bestSurfaceVel = vel;
                    bestColliderId = colId;
                }
            }

            return found;
        }

        private bool ResolveHitMetadata(
            ColliderIdKey key,
            float3 worldPoint,
            uint vehicleId,
            out byte matIndex,
            out float frictionScale,
            out int dynBodyIdx,
            out float3 surfaceVel,
            out int colliderId)
        {
            matIndex = 0;
            frictionScale = 1.0f;
            dynBodyIdx = -1;
            surfaceVel = float3.zero;
            colliderId = key.GetHashCode();

            if (ColliderMetaMap.TryGetValue(key, out ColliderPhysicsMeta meta))
            {
                if ((meta.Flags & ColliderMetaFlags.IgnoreWheelRays) != 0)
                    return false;
                if (meta.OwnerVehicleId != 0 && meta.OwnerVehicleId == vehicleId)
                    return false;

                matIndex = meta.SurfaceMaterialIndex;
                dynBodyIdx = meta.DynamicBodyIndex;
                if ((uint)dynBodyIdx < (uint)DynamicBodies.Length)
                {
                    DynamicBodyMotion dyn = DynamicBodies[dynBodyIdx];
                    surfaceVel = dyn.GetPointVelocity(worldPoint);
                }
            }

            return true;
        }

        private void AppendRawSample(
            int sampleBase,
            ref int rawCount,
            float3 pt,
            float pen,
            float3 nrm,
            float3 vel,
            float angleRad,
            byte matIdx,
            float fricScale,
            byte zoneId)
        {
            if (rawCount >= MaxSamplesPerWheel || sampleBase + rawCount >= ContactSamples.Length)
                return;

            int mIdx = math.clamp(matIdx, 0, SurfaceMaterials.Length - 1);
            SurfaceMaterialData mat = SurfaceMaterials[mIdx];

            ContactSamples[sampleBase + rawCount] = new WheelContactSample
            {
                Point = pt,
                PenetrationDepth = pen,
                Normal = nrm,
                StaticFriction = mat.StaticFriction * fricScale,
                SurfaceVelocity = vel,
                DynamicFriction = mat.DynamicFriction * fricScale,
                RollingResistance = mat.RollingResistance,
                SampleWeight = 1.0f,
                ProbeAngleRad = angleRad,
                IsValid = 1,
                SurfaceMaterialType = matIdx,
                ZoneId = zoneId
            };
            rawCount++;
        }

        private static ManifoldContactPoint MakeCluster(
            float3 pt, float pen, float3 nrm, float3 vel,
            int colId, int dynIdx, float fricScale, byte matIdx, byte count, byte flags)
        {
            return new ManifoldContactPoint
            {
                WorldPoint = pt,
                PenetrationM = pen,
                WorldNormal = nrm,
                ContactForceN = 0f,
                SurfaceVelocity = vel,
                ColliderInstanceID = colId,
                PendingObjectImpulse = float3.zero,
                DynamicBodyIndex = dynIdx,
                ContactWeight = 1.0f,
                LocalFluidOrLooseDepthM = 0f,
                FrictionScale = fricScale,
                SurfaceMaterialIndex = matIdx,
                IsValid = 1,
                SampleCount = count,
                ClusterFlags = flags
            };
        }
    }
}
