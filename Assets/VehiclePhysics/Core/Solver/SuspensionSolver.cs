using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class SuspensionSolver
    {
        private struct AxleCompressionTracker
        {
            public float4 LeftSum;
            public float4 RightSum;
            public float4 LeftCount;
            public float4 RightCount;

            public void Accumulate(byte axleIdx, byte sideIdx, float compression)
            {
                if (axleIdx >= 4) return;
                int a = axleIdx;
                if (sideIdx == 0)
                {
                    AddToComp(ref LeftSum, a, compression);
                    AddToComp(ref LeftCount, a, 1f);
                }
                else
                {
                    AddToComp(ref RightSum, a, compression);
                    AddToComp(ref RightCount, a, 1f);
                }
            }

            public float GetLeftMinusRight(byte axleIdx)
            {
                if (axleIdx >= 4) return 0f;
                int a = axleIdx;

                float lCnt = GetComp(LeftCount, a);
                float rCnt = GetComp(RightCount, a);

                // Anti-roll bar cannot transfer load if either side of the axle has no attached wheels
                if (lCnt < 0.5f || rCnt < 0.5f) return 0f;

                float lAvg = GetComp(LeftSum, a) / lCnt;
                float rAvg = GetComp(RightSum, a) / rCnt;
                return lAvg - rAvg;
            }

            private static void AddToComp(ref float4 v, int idx, float val)
            {
                switch (idx)
                {
                    case 0: v.x += val; break;
                    case 1: v.y += val; break;
                    case 2: v.z += val; break;
                    default: v.w += val; break;
                }
            }

            private static float GetComp(in float4 v, int idx)
            {
                return idx switch { 0 => v.x, 1 => v.y, 2 => v.z, _ => v.w };
            }
        }

        public static void SolveSuspension(
            ref VehicleState veh,
            in VehicleConfig vCfg,
            NativeArray<WheelConfig> wheelConfigs,
            NativeArray<WheelContact> wheelContacts,
            NativeArray<WheelState> wheels)
        {
            int start = veh.WheelStartIndex;
            int cfgStart = vCfg.WheelConfigStartIndex;
            var arbTracker = new AxleCompressionTracker();
            bool anyGrounded = false;

            // Pass 1: Analytical contact-plane projection, bump stops, and asymmetric dampers
            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                WheelContact contact = wheelContacts[slot];
                WheelState ws = wheels[slot];

                if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0)
                {
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsGrounded);
                    ws.SuspensionCompression = 0f;
                    ws.SuspensionVelocity = 0f;
                    ws.GroundLoadN = 0f;
                    ws.LastWorldForce = float3.zero;
                    wheels[slot] = ws;
                    continue;
                }

                if (contact.HasContact == 0)
                {
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsGrounded);
                    ws.SuspensionCompression = 0f;
                    ws.SuspensionVelocity = 0f;
                    ws.GroundLoadN = 0f;
                    ws.EffectiveRollingRadius = wCfg.UnloadedRadiusM;
                    wheels[slot] = ws;
                    arbTracker.Accumulate(wCfg.AxleIndex, wCfg.SideIndex, 0f);
                    continue;
                }

                float3 mountRelCoM = wCfg.MountPositionLocal - vCfg.CenterOfMassLocalOffset;
                float3 worldMount = veh.Position + math.rotate(veh.Rotation, mountRelCoM);
                float3 worldStrut = math.normalizesafe(
                    math.rotate(veh.Rotation, wCfg.StrutAxisLocal),
                    new float3(0f, -1f, 0f));

                float3 n = contact.PlaneNormal;
                float cosAngle = -math.dot(worldStrut, n);
                if (cosAngle <= 0.1f)
                {
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsGrounded);
                    ws.SuspensionCompression = 0f;
                    ws.SuspensionVelocity = 0f;
                    ws.GroundLoadN = 0f;
                    ws.EffectiveRollingRadius = wCfg.UnloadedRadiusM;
                    wheels[slot] = ws;
                    arbTracker.Accumulate(wCfg.AxleIndex, wCfg.SideIndex, 0f);
                    continue;
                }

                float denom = math.max(cosAngle, 0.25f);
                float distToPlane = math.dot(worldMount - contact.PlanePoint, n) / denom;
                float reqStrutLength = distToPlane - wCfg.UnloadedRadiusM;
                float rawCompression = wCfg.RestLengthM - reqStrutLength;

                if (rawCompression <= 0f)
                {
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsGrounded);
                    ws.SuspensionCompression = 0f;
                    ws.SuspensionVelocity = 0f;
                    ws.GroundLoadN = 0f;
                    ws.EffectiveRollingRadius = wCfg.UnloadedRadiusM;
                    wheels[slot] = ws;
                    arbTracker.Accumulate(wCfg.AxleIndex, wCfg.SideIndex, 0f);
                    continue;
                }

                anyGrounded = true;
                ws.WheelFlags |= WheelFlagBits.IsGrounded;

                float clampedComp = math.min(rawCompression, wCfg.MaxTravelM);
                float bumpStopPen = math.max(0f, rawCompression - wCfg.MaxTravelM);

                float3 vMount = veh.LinearVelocity
                              + math.cross(veh.AngularVelocity, worldMount - veh.Position)
                              - contact.SurfaceVelocity;
                float compVel = -math.dot(vMount, n) / math.max(cosAngle, 0.5f);

                float springForce = wCfg.SpringStiffnessNPerM * clampedComp
                                  + wCfg.BumpStopStiffnessNPerM * bumpStopPen;
                float damperRate = compVel >= 0f ? wCfg.BumpDamperNsPerM : wCfg.ReboundDamperNsPerM;
                float rawFz = springForce + damperRate * compVel;

                ws.SuspensionCompression = clampedComp;
                ws.SuspensionVelocity = compVel;
                ws.GroundLoadN = rawFz;
                wheels[slot] = ws;

                arbTracker.Accumulate(wCfg.AxleIndex, wCfg.SideIndex, clampedComp);
            }

            if (anyGrounded) veh.TierFlags |= VehicleTierFlags.IsGroundedAny;
            else veh.TierFlags = (byte)(veh.TierFlags & ~VehicleTierFlags.IsGroundedAny);

            // Pass 2: Apply Anti-Roll Bar load transfer and enforce unilateral constraint Fz >= 0
            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];
                if ((ws.WheelFlags & (WheelFlagBits.IsGrounded | WheelFlagBits.IsDetached)) != WheelFlagBits.IsGrounded)
                    continue;

                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                float kArb = wCfg.AxleIndex == 0
                    ? vCfg.FrontAntiRollStiffnessNPerM
                    : vCfg.RearAntiRollStiffnessNPerM;

                float deltaComp = arbTracker.GetLeftMinusRight(wCfg.AxleIndex);
                float arbForce = (wCfg.SideIndex == 0 ? 1f : -1f) * kArb * deltaComp;

                float fz = math.max(0f, ws.GroundLoadN + arbForce);
                ws.GroundLoadN = fz;

                float tireSquish = math.min(wCfg.MaxRadialSquishM,
                    fz / math.max(wCfg.RadialTireStiffnessNPerM, 1e4f));
                ws.EffectiveRollingRadius = math.max(0.05f, wCfg.UnloadedRadiusM - tireSquish);

                wheels[slot] = ws;
            }
        }
    }
}
