using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public struct TireStepResult
    {
        public float3 WorldApplicationPoint;
        public float3 LongDir;
        public float3 LatDir;
        public float Fx;
        public float Fy;
        public float dFx_dOmega;
    }

    public static class TireSolver
    {
        public static TireStepResult EvaluateTireForces(
            in VehicleState veh,
            in VehicleConfig vCfg,
            in WheelConfig wCfg,
            in WheelContact contact,
            ref WheelState ws,
            float dtSub)
        {
            if ((ws.WheelFlags & WheelFlagBits.IsGrounded) == 0 || ws.GroundLoadN <= 1e-3f)
            {
                // Decay carcass deflection rapidly when airborne.
                float airDecay = 1.0f / (1.0f + 25.0f * dtSub);
                ws.LongitudinalDeflection *= airDecay;
                ws.LateralDeflection *= airDecay;
                ws.EffectiveSlipRatio = 0f;
                ws.EffectiveSlipAngleRad = 0f;
                ws.FrictionUtilization = 0f;
                ws.LastWorldForce = float3.zero;
                return default;
            }

            float3 mountRelCoM = wCfg.MountPositionLocal - vCfg.CenterOfMassLocalOffset;
            float3 worldMount = veh.Position + math.rotate(veh.Rotation, mountRelCoM);
            float3 worldStrut = math.normalizesafe(
                math.rotate(veh.Rotation, wCfg.StrutAxisLocal),
                new float3(0f, -1f, 0f));

            // Instantaneous hub & contact-patch point projected directly under the wheel
            // (eliminates fixed-step horizontal lever-arm lag at high speeds).
            float3 hubPos = worldMount + worldStrut * (wCfg.RestLengthM - ws.SuspensionCompression);
            float3 n = contact.PlaneNormal;
            float hubDistToPlane = math.dot(hubPos - contact.PlanePoint, n);
            float3 patchPos = hubPos - n * hubDistToPlane;

            // Steered wheel forward axis (+Z rotated around local +Y; +steer turns Right/+X)
            float3 localForward = new float3(math.sin(ws.SteerAngleRad), 0f, math.cos(ws.SteerAngleRad));
            float3 worldForward = math.rotate(veh.Rotation, localForward);
            float3 fallbackRight = math.rotate(veh.Rotation, new float3(1f, 0f, 0f));

            float3 tLat = math.normalizesafe(math.cross(n, worldForward), fallbackRight);
            float3 tLong = math.normalizesafe(math.cross(tLat, n), worldForward);

            float3 vRel = veh.LinearVelocity
                        + math.cross(veh.AngularVelocity, patchPos - veh.Position)
                        - contact.SurfaceVelocity;

            float vx = math.dot(vRel, tLong);
            float vy = math.dot(vRel, tLat);
            float rEff = ws.EffectiveRollingRadius;

            float vSlipX = ws.AngularVelocity * rEff - vx;
            float vSlipY = -vy;

            float absVx = math.abs(vx);
            float sigX = math.max(wCfg.RelaxationLengthX, 0.02f);
            float sigY = math.max(wCfg.RelaxationLengthY, 0.02f);
            float peakKappa = math.max(wCfg.PeakSlipRatio, 0.02f);
            float peakTanAlpha = math.max(math.tan(math.max(wCfg.PeakSlipAngleRad, 0.02f)), 0.02f);

            // Unconditionally stable Backward-Euler 1st-order carcass relaxation
            float dx = absVx / sigX * dtSub;
            float dy = absVx / sigY * dtSub;
            float ux = (ws.LongitudinalDeflection + vSlipX * dtSub) / (1.0f + dx);
            float uy = (ws.LateralDeflection + vSlipY * dtSub) / (1.0f + dy);

            float sx = ux / (sigX * peakKappa);
            float sy = uy / (sigY * peakTanAlpha);
            float s = math.sqrt(sx * sx + sy * sy);

            // Prevent carcass spring windup during locked slides or parked-slope transitions
            const float windupCap = 3.0f;
            if (s > windupCap)
            {
                float invScale = windupCap / s;
                ux *= invScale;
                uy *= invScale;
                sx *= invScale;
                sy *= invScale;
                s = windupCap;
            }

            ws.LongitudinalDeflection = ux;
            ws.LateralDeflection = uy;
            ws.EffectiveSlipRatio = sx * peakKappa;
            ws.EffectiveSlipAngleRad = math.atan(sy * peakTanAlpha);
            ws.FrictionUtilization = s;

            float fz = ws.GroundLoadN;
            float muPeakX = contact.StaticFriction * wCfg.TirePeakMuScale;
            float muSlideX = contact.DynamicFriction * wCfg.TireSlideMuScale;
            float muPeakY = contact.StaticFriction * wCfg.TirePeakMuScale;
            float muSlideY = contact.DynamicFriction * wCfg.TireSlideMuScale;

            // C1-continuous, zero-singularity combined slip curve (exact peak=1.0 at s=1, finite slope at s=0)
            float normMagOverS;
            if (s <= 1.0f)
            {
                normMagOverS = 2.0f / (1.0f + s * s);
            }
            else
            {
                float dropT = math.saturate((s - 1.0f) / 2.5f);
                float smoothDrop = dropT * dropT * (3.0f - 2.0f * dropT);
                float avgPeak = 0.5f * (muPeakX + muPeakY);
                float avgSlide = 0.5f * (muSlideX + muSlideY);
                float ratioAtS = math.lerp(1.0f, avgSlide / math.max(avgPeak, 1e-4f), smoothDrop);
                normMagOverS = ratioAtS / s;
            }

            float fxElastic = sx * normMagOverS * (muPeakX * fz);
            float fyElastic = sy * normMagOverS * (muPeakY * fz);

            // Stability-bounded low-speed carcass damping (kills slope/rest jitter at both 120Hz and 500Hz)
            float vHubMag = math.length(vRel);
            float vBlend = 1.0f - math.smoothstep(0.0f, 1.5f, vHubMag);
            float cornerMass = fz / math.max(vCfg.GravityMs2, 1.0f);
            float cLowMax = 0.4f * cornerMass / math.max(dtSub, 1e-5f);
            float dampScale = wCfg.LowSpeedDampMaxForceScale > 0f ? wCfg.LowSpeedDampMaxForceScale : 1.5f;
            float cLow = math.min(cLowMax, fz * dampScale) * vBlend;

            float fx = fxElastic + cLow * vSlipX;
            float fy = fyElastic + cLow * vSlipY;

            // Friction ellipse clamp
            float maxFx = math.max(muPeakX * fz, 1e-3f);
            float maxFy = math.max(muPeakY * fz, 1e-3f);
            float ex = fx / maxFx;
            float ey = fy / maxFy;
            float eMag = math.sqrt(ex * ex + ey * ey);
            if (eMag > 1.0f)
            {
                float invE = 1.0f / eMag;
                fx *= invE;
                fy *= invE;
            }

            // Implicit wheel-spin tire damping derivative for Step 3/4 spin stability
            float kTreadX = 2.0f * maxFx / (sigX * peakKappa);
            float dFx_dOmega = rEff * (kTreadX * dtSub / (1.0f + dx) + cLow);

            return new TireStepResult
            {
                WorldApplicationPoint = patchPos,
                LongDir = tLong,
                LatDir = tLat,
                Fx = fx,
                Fy = fy,
                dFx_dOmega = dFx_dOmega
            };
        }
    }
}
