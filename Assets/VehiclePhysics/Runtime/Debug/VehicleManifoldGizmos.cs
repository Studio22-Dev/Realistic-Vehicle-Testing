using System.Text;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using VehiclePhysics.Core;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace VehiclePhysics.Runtime
{
    /// <summary>
    /// Scene-view & HUD diagnostics for Phase 2A Step 2 volumetric Capsulecast + Spherecast
    /// ContactManifold clustering (C0..C3) and Step 3 raw WheelContactSample planes.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class VehicleManifoldGizmos : MonoBehaviour
    {
        [Header("Target Selection")]
        [SerializeField] private VehicleSimulationManager simulationManager;
        [Tooltip("-1 draws all active vehicles up to MaxVehiclesToDraw; 0 draws Player (LOD0).")]
        [SerializeField] private int targetVehicleIndex = 0;
        [SerializeField] private int maxVehiclesToDraw = 4;

        [Header("Geometry & Probe Layers")]
        [SerializeField] private bool drawSuspensionAndRim = true;
        [SerializeField] private bool drawCapsuleSweeps = true;
        [SerializeField] private bool drawSidewallSpheres = true;
        [SerializeField] private bool drawOnlyHitSweeps = false;

        [Header("ContactManifold & Raw XPBD Planes")]
        [SerializeField] private bool drawClusterNormals = true;
        [SerializeField] private bool drawBlendedSummary = true;
        [SerializeField] private bool drawRawXpbdPlanes = true;
        [SerializeField] private bool drawDynamicImpulseVectors = true;

        [Header("Telemetry")]
        [SerializeField] private bool drawSceneLabels = true;
        [SerializeField] private bool drawScreenHud = true;
        [SerializeField] private float normalDisplayScale = 0.35f;
        [SerializeField] private float lookaheadMarginM = 0.02f;

        private static readonly float[] Lod0AnglesDeg = { 0f, 18f, 36f, 54f, 72f, -18f, -36f, -54f, -72f };
        private static readonly float[] Lod1AnglesDeg = { 0f, 24f, 48f, -24f, -48f };
        private static readonly float[] Lod2AnglesDeg = { 0f, 36f, -36f };

        private readonly StringBuilder _sb = new StringBuilder(256);

        private void Reset()
        {
            if (simulationManager == null)
                simulationManager = GetComponent<VehicleSimulationManager>();
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
                return;

            if (!TryAcquireWorld(out VehicleSimulationWorld world))
                return;

            int activeVehicles = world.ActiveVehicleCount;
            if (activeVehicles <= 0)
                return;

            int startVeh = targetVehicleIndex >= 0 ? math.min(targetVehicleIndex, activeVehicles - 1) : 0;
            int endVeh = targetVehicleIndex >= 0
                ? startVeh + 1
                : math.min(activeVehicles, startVeh + math.max(1, maxVehiclesToDraw));

            for (int v = startVeh; v < endVeh; v++)
            {
                DrawVehicleManifolds(in world, v);
            }
        }

        private void DrawVehicleManifolds(in VehicleSimulationWorld world, int vehIdx)
        {
            VehicleState veh = world.Vehicles[vehIdx];
            VehicleConfig vCfg = world.VehicleConfigs[veh.ConfigIndex];
            int cfgWheelBase = vCfg.WheelConfigStartIndex;

            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = veh.WheelStartIndex + w;
                WheelState ws = world.Wheels[slot];
                if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0)
                    continue;

                WheelConfig wCfg = world.WheelConfigs[cfgWheelBase + w];
                WheelInput wi = world.WheelInputs[slot];
                ContactManifold manifold = world.ContactManifolds[slot];
                TireCarcassState carcass = world.TireCarcassStates[slot];

                GetWheelFrameWorld(
                    in veh, in wCfg, wi.TargetSteerAngleRad,
                    out float3 mountWorld, out float3 strutDownWorld,
                    out float3 wheelFwd, out float3 wheelUp, out float3 wheelOut);

                float3 downDir = strutDownWorld;
                float r = wCfg.UnloadedRadiusM;
                float halfW = (wCfg.TireWidthM > 0f ? wCfg.TireWidthM : 0.235f) * 0.5f;
                float restLength = wCfg.RestLengthM;
                float maxTravel = wCfg.MaxTravelM;
                float centerRayDist = restLength + r + lookaheadMarginM;

                float hubDropDroop = math.max(0f, centerRayDist - lookaheadMarginM - r);
                float hubDropCurrent = math.max(0f, hubDropDroop - ws.SuspensionCompression);
                float3 hubEstimate = mountWorld + downDir * hubDropCurrent;

                float mountContactDist = ComputeMountContactDistance(in manifold, mountWorld, downDir, centerRayDist);

                if (drawSuspensionAndRim)
                {
                    Handles.color = new Color(0.75f, 0.75f, 0.75f, 0.65f);
                    Handles.DrawLine(mountWorld, hubEstimate);
                    Handles.DrawWireDisc(hubEstimate, wheelOut, r);

                    Handles.color = new Color(0.45f, 0.75f, 1.0f, 0.65f);
                    float rimR = carcass.RimRadiusM > 0f ? carcass.RimRadiusM : (wCfg.RimRadiusM > 0f ? wCfg.RimRadiusM : r * 0.62f);
                    Handles.DrawWireDisc(hubEstimate, wheelOut, rimR);

                    if (manifold.IsGrounded != 0)
                    {
                        float3 mountContactPt = mountWorld + downDir * mountContactDist;
                        Handles.color = Color.magenta;
                        Handles.DrawWireDisc(mountContactPt, wheelUp, 0.045f);
                    }
                }

                GetRawProbeSlice(in world, in wi, in carcass, slot, out int rawBase, out int rawCount);

                if (drawCapsuleSweeps)
                {
                    DrawWheelCapsuleArc(
                        in world, veh.LodTier, rawBase, rawCount,
                        mountWorld, hubEstimate, wheelFwd, downDir, wheelOut,
                        r, halfW, centerRayDist, maxTravel);
                }

                if (drawSidewallSpheres)
                {
                    DrawWheelSidewallSpheres(
                        in world, veh.LodTier, rawBase, rawCount, in manifold,
                        hubEstimate, downDir, wheelOut, r, halfW);
                }

                if (drawRawXpbdPlanes && rawCount > 0 && world.ContactSamples.IsCreated)
                {
                    for (int p = 0; p < rawCount; p++)
                    {
                        WheelContactSample probe = world.ContactSamples[rawBase + p];
                        if (probe.IsValid == 0)
                            continue;

                        Color col = GetPenetrationColor(probe.PenetrationDepth, isOverlap: false);
                        Handles.color = col;
                        Handles.DrawWireDisc(probe.Point, probe.Normal, 0.04f);
                        Handles.DrawLine(probe.Point, probe.Point + probe.Normal * (normalDisplayScale * 0.65f));
                    }
                }

                if (drawClusterNormals)
                {
                    DrawCluster(in manifold.ClusterBottom,   new Color(0.20f, 1.00f, 0.30f), "C0");
                    DrawCluster(in manifold.ClusterLeading,  new Color(1.00f, 0.55f, 0.10f), "C1");
                    DrawCluster(in manifold.ClusterTrailing, new Color(0.25f, 0.65f, 1.00f), "C2");
                    DrawCluster(in manifold.ClusterSidewall, new Color(1.00f, 0.20f, 0.90f), "C3");
                }

                if (drawBlendedSummary && manifold.IsGrounded != 0)
                {
                    Handles.color = Color.white;
                    Handles.DrawSolidDisc(manifold.BlendedPoint, manifold.BlendedNormal, 0.025f);
                    Handles.DrawAAPolyLine(3f, manifold.BlendedPoint, manifold.BlendedPoint + manifold.BlendedNormal * normalDisplayScale);
                }

                if (drawSceneLabels)
                {
                    _sb.Clear();
                    _sb.Append("V").Append(vehIdx).Append(" W").Append(w)
                       .Append(" [LOD").Append(veh.LodTier).Append("]\n")
                       .Append("Mask: 0x").Append(manifold.ActiveClusterMask.ToString("X"))
                       .Append("  Raw: ").Append(rawCount).Append('\n')
                       .Append("Dist: ").Append(mountContactDist.ToString("F3"))
                       .Append("  Comp: ").Append(ws.SuspensionCompression.ToString("F3"));

                    if (manifold.ClusterSidewall.IsValid != 0)
                        _sb.Append("\nC3 Pen: ").Append(manifold.ClusterSidewall.PenetrationM.ToString("F3"));

                    Handles.Label(hubEstimate + wheelUp * (r + 0.14f), _sb.ToString());
                }
            }
        }

        private void DrawWheelCapsuleArc(
            in VehicleSimulationWorld world,
            byte lod,
            int rawBase,
            int rawCount,
            float3 mountWorld,
            float3 hubEstimate,
            float3 wheelFwd,
            float3 downDir,
            float3 wheelOut,
            float r,
            float halfW,
            float centerRayDist,
            float maxTravel)
        {
            float targetCapRadius = lod == 0 ? 0.065f : (lod == 1 ? 0.085f : 0.110f);
            float rCap = math.min(targetCapRadius, math.min(r * 0.42f, halfW * 0.90f));
            float halfSpan = math.max(0.005f, halfW - rCap);
            float backoff = math.max(rCap + 0.06f, 0.45f * r);
            float arcSweep = backoff + (r - rCap) + lookaheadMarginM;

            float[] angles = lod == 0 ? Lod0AnglesDeg : (lod == 1 ? Lod1AnglesDeg : Lod2AnglesDeg);

            for (int i = 0; i < angles.Length; i++)
            {
                float thetaRad = math.radians(angles[i]);
                bool hasMatch = TryFindMatchingProbe(
                    in world, rawBase, rawCount, thetaRad, angleTolRad: 0.09f, out WheelContactSample matched);

                if (drawOnlyHitSweeps && !hasMatch)
                    continue;

                bool isOverlap = hasMatch && matched.PenetrationDepth > r * 0.95f;
                Color col = !hasMatch
                    ? new Color(0.5f, 0.5f, 0.5f, 0.30f)
                    : GetPenetrationColor(matched.PenetrationDepth, isOverlap);

                float3 dir;
                float3 startCenter;
                float drawEndDist;

                if (i == 0)
                {
                    dir = downDir;
                    startCenter = mountWorld - dir * rCap;
                    drawEndDist = hasMatch
                        ? math.clamp(math.dot(matched.Point - mountWorld, dir), 0f, centerRayDist)
                        : math.min(centerRayDist, maxTravel + r + lookaheadMarginM);
                }
                else
                {
                    math.sincos(thetaRad, out float s, out float c);
                    dir = math.normalize(wheelFwd * s + downDir * c);
                    startCenter = hubEstimate - dir * backoff;
                    drawEndDist = hasMatch
                        ? math.max(0f, (r - matched.PenetrationDepth) + backoff - rCap)
                        : arcSweep;
                }

                float3 endCenter = startCenter + dir * drawEndDist;

                Handles.color = new Color(col.r, col.g, col.b, col.a * 0.45f);
                Handles.DrawLine(startCenter, endCenter);

                Handles.color = col;
                DrawWireCapsule(endCenter - wheelOut * halfSpan, endCenter + wheelOut * halfSpan, rCap);
            }
        }

        private void DrawWheelSidewallSpheres(
            in VehicleSimulationWorld world,
            byte lod,
            int rawBase,
            int rawCount,
            in ContactManifold manifold,
            float3 hubEstimate,
            float3 downDir,
            float3 wheelOut,
            float r,
            float halfW)
        {
            if (lod > 1)
                return;

            float rSide = math.min(r * 0.28f, halfW * 0.85f);
            float sideBackoff = rSide + 0.05f;
            float sideSweep = sideBackoff + (halfW - rSide) + lookaheadMarginM;

            DrawSingleSideSphere(
                in world, rawBase, rawCount, in manifold.ClusterSidewall,
                hubEstimate, math.normalize(0.95f * wheelOut + 0.31f * downDir),
                +math.PI * 0.5f, rSide, sideBackoff, sideSweep, halfW);

            if (lod == 0)
            {
                DrawSingleSideSphere(
                    in world, rawBase, rawCount, in manifold.ClusterSidewall,
                    hubEstimate, math.normalize(-0.95f * wheelOut + 0.31f * downDir),
                    -math.PI * 0.5f, rSide, sideBackoff, sideSweep, halfW);
            }
        }

        private void DrawSingleSideSphere(
            in VehicleSimulationWorld world,
            int rawBase,
            int rawCount,
            in ManifoldContactPoint c3,
            float3 hubEstimate,
            float3 sideDir,
            float targetAngleRad,
            float rSide,
            float sideBackoff,
            float sideSweep,
            float halfW)
        {
            bool hasHit = TryFindMatchingProbe(
                in world, rawBase, rawCount, targetAngleRad, angleTolRad: 0.12f, out WheelContactSample matched);

            if (drawOnlyHitSweeps && !hasHit && c3.IsValid == 0)
                return;

            float pen = hasHit ? matched.PenetrationDepth : (c3.IsValid != 0 ? c3.PenetrationM : -1f);
            Color col = hasHit
                ? GetPenetrationColor(pen, isOverlap: false)
                : new Color(0.8f, 0.35f, 0.9f, 0.25f);

            float3 origin = hubEstimate - sideDir * sideBackoff;
            float drawDist = hasHit
                ? math.max(0f, (halfW - matched.PenetrationDepth) + sideBackoff - rSide)
                : sideSweep;

            float3 endCenter = origin + sideDir * drawDist;
            Handles.color = col;
            Handles.DrawLine(origin, endCenter);
            Handles.DrawWireDisc(endCenter, sideDir, rSide);
            Handles.DrawWireDisc(endCenter, math.up(), rSide);
        }

        private void DrawCluster(in ManifoldContactPoint c, Color color, string tag)
        {
            if (c.IsValid == 0)
                return;

            Handles.color = color;
            Handles.DrawWireDisc(c.WorldPoint, c.WorldNormal, 0.055f);
            Handles.DrawLine(c.WorldPoint, c.WorldPoint + c.WorldNormal * normalDisplayScale);

            if (drawDynamicImpulseVectors && c.DynamicBodyIndex >= 0 && math.lengthsq(c.PendingObjectImpulse) > 1e-8f)
            {
                Handles.color = Color.red;
                Handles.DrawLine(c.WorldPoint, c.WorldPoint + c.PendingObjectImpulse * 0.05f);
            }

            if (drawSceneLabels)
                Handles.Label(c.WorldPoint + c.WorldNormal * (normalDisplayScale + 0.03f), tag);
        }

        private static Color GetPenetrationColor(float penetrationM, bool isOverlap)
        {
            if (isOverlap)
                return Color.cyan;
            if (penetrationM <= 0f)
                return new Color(0.25f, 0.95f, 0.40f, 0.85f);

            float t = math.saturate(penetrationM / 0.08f);
            return Color.Lerp(Color.yellow, Color.red, t);
        }

        private static void DrawWireCapsule(float3 p1, float3 p2, float radius)
        {
            float3 axis = p2 - p1;
            float lenSq = math.lengthsq(axis);
            if (lenSq < 1e-8f)
            {
                Handles.DrawWireDisc(p1, math.right(), radius);
                Handles.DrawWireDisc(p1, math.up(), radius);
                Handles.DrawWireDisc(p1, math.forward(), radius);
                return;
            }

            float3 dir = axis / math.sqrt(lenSq);
            float3 refVec = math.abs(dir.y) < 0.99f ? new float3(0f, 1f, 0f) : new float3(1f, 0f, 0f);
            float3 u = math.normalize(math.cross(dir, refVec));
            float3 v = math.cross(dir, u);

            Handles.DrawWireDisc(p1, dir, radius);
            Handles.DrawWireDisc(p2, dir, radius);

            Handles.DrawLine(p1 + u * radius, p2 + u * radius);
            Handles.DrawLine(p1 - u * radius, p2 - u * radius);
            Handles.DrawLine(p1 + v * radius, p2 + v * radius);
            Handles.DrawLine(p1 - v * radius, p2 - v * radius);

            Handles.DrawWireArc(p1, v, u, -180f, radius);
            Handles.DrawWireArc(p1, u, v,  180f, radius);
            Handles.DrawWireArc(p2, v, u,  180f, radius);
            Handles.DrawWireArc(p2, u, v, -180f, radius);
        }
#endif

        private void OnGUI()
        {
            if (!drawScreenHud || !Application.isPlaying)
                return;

            if (!TryAcquireWorld(out VehicleSimulationWorld world))
                return;

            int activeVehicles = world.ActiveVehicleCount;
            if (activeVehicles <= 0)
                return;

            int vIdx = targetVehicleIndex >= 0 ? math.min(targetVehicleIndex, activeVehicles - 1) : 0;
            VehicleState veh = world.Vehicles[vIdx];
            VehicleConfig vCfg = world.VehicleConfigs[veh.ConfigIndex];
            int cfgWheelBase = vCfg.WheelConfigStartIndex;

            _sb.Clear();
            _sb.Append("Manifold Telemetry | Vehicle ").Append(vIdx)
               .Append(" | LOD").Append(veh.LodTier)
               .Append(" | Substeps: ").Append(veh.SubstepCount)
               .Append(" | Speed: ").Append((veh.ForwardSpeedMs * 3.6f).ToString("F1")).Append(" km/h\n");

            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = veh.WheelStartIndex + w;
                WheelConfig wCfg = world.WheelConfigs[cfgWheelBase + w];
                WheelInput wi = world.WheelInputs[slot];
                ContactManifold m = world.ContactManifolds[slot];
                TireCarcassState c = world.TireCarcassStates[slot];
                WheelState ws = world.Wheels[slot];

                float3 mountWorld = veh.Position + math.rotate(veh.Rotation, wCfg.MountPositionLocal);
                float3 downWorld = math.rotate(veh.Rotation, math.normalizesafe(wCfg.StrutAxisLocal, new float3(0f, -1f, 0f)));
                float fallbackDist = wCfg.RestLengthM + wCfg.UnloadedRadiusM + lookaheadMarginM;
                float dist = ComputeMountContactDistance(in m, mountWorld, downWorld, fallbackDist);
                GetRawProbeSlice(in world, in wi, in c, slot, out _, out int rawCount);

                _sb.Append("W").Append(w)
                   .Append(" G:").Append(m.IsGrounded)
                   .Append(" Mask:0x").Append(m.ActiveClusterMask.ToString("X"))
                   .Append(" C0:").Append(m.ClusterBottom.PenetrationM.ToString("F3"))
                   .Append(" C1:").Append(m.ClusterLeading.PenetrationM.ToString("F3"))
                   .Append(" C2:").Append(m.ClusterTrailing.PenetrationM.ToString("F3"))
                   .Append(" C3:").Append(m.ClusterSidewall.PenetrationM.ToString("F3"))
                   .Append(" Dist:").Append(dist.ToString("F3"))
                   .Append(" Comp:").Append(ws.SuspensionCompression.ToString("F3"))
                   .Append(" Raw:").Append(rawCount)
                   .Append('\n');
            }

            GUI.Box(new Rect(14f, 170f, 700f, 34f + veh.WheelCount * 20f), _sb.ToString());
        }

        private static float ComputeMountContactDistance(
            in ContactManifold manifold,
            float3 mountWorld,
            float3 downDir,
            float fallbackDist)
        {
            if (manifold.IsGrounded == 0)
                return fallbackDist;

            float3 refPt = manifold.ClusterBottom.IsValid != 0
                ? manifold.ClusterBottom.WorldPoint
                : manifold.BlendedPoint;

            return math.max(0f, math.dot(refPt - mountWorld, downDir));
        }

        private static bool TryFindMatchingProbe(
            in VehicleSimulationWorld world,
            int rawBase,
            int rawCount,
            float targetAngleRad,
            float angleTolRad,
            out WheelContactSample best)
        {
            best = default;
            if (!world.ContactSamples.IsCreated || rawCount <= 0)
                return false;

            bool found = false;
            float bestDiff = angleTolRad;

            for (int i = 0; i < rawCount; i++)
            {
                WheelContactSample probe = world.ContactSamples[rawBase + i];
                if (probe.IsValid == 0)
                    continue;

                float diff = math.abs(probe.ProbeAngleRad - targetAngleRad);
                if (diff <= bestDiff)
                {
                    bestDiff = diff;
                    best = probe;
                    found = true;
                }
            }
            return found;
        }

        private bool TryAcquireWorld(out VehicleSimulationWorld world)
        {
            world = null;
            if (simulationManager == null)
                return false;

            if (!simulationManager.TryGetWorldForDebug(out world))
                return false;

            return world != null
                && world.ContactManifolds.IsCreated
                && world.Wheels.IsCreated
                && world.Vehicles.IsCreated;
        }

        private static void GetWheelFrameWorld(
            in VehicleState veh,
            in WheelConfig wCfg,
            float steerRad,
            out float3 mountWorld,
            out float3 strutDownWorld,
            out float3 wheelFwd,
            out float3 wheelUp,
            out float3 wheelOut)
        {
            mountWorld = veh.Position + math.rotate(veh.Rotation, wCfg.MountPositionLocal);
            float3 strutDownLocal = math.normalizesafe(wCfg.StrutAxisLocal, new float3(0f, -1f, 0f));
            strutDownWorld = math.rotate(veh.Rotation, strutDownLocal);

            quaternion steerRot = quaternion.AxisAngle(new float3(0f, 1f, 0f), steerRad);
            quaternion wheelWorldRot = math.mul(veh.Rotation, steerRot);

            wheelFwd = math.rotate(wheelWorldRot, new float3(0f, 0f, 1f));
            wheelUp  = math.rotate(wheelWorldRot, new float3(0f, 1f, 0f));
            float sideSign = wCfg.SideIndex == 0 ? -1f : 1f;
            wheelOut = math.rotate(wheelWorldRot, new float3(sideSign, 0f, 0f));
        }

        private static void GetRawProbeSlice(
            in VehicleSimulationWorld world,
            in WheelInput wi,
            in TireCarcassState carcass,
            int slot,
            out int rawBase,
            out int rawCount)
        {
            int maxSamples = world.MaxSamplesPerWheel;
            if (wi.ContactSampleCount > 0)
            {
                rawBase = wi.ContactSampleStartIndex;
                rawCount = math.min(wi.ContactSampleCount, maxSamples);
            }
            else
            {
                rawBase = slot * maxSamples;
                rawCount = math.clamp(carcass.ActiveRawProbeCount, 0, maxSamples);
            }
        }
    }
}
