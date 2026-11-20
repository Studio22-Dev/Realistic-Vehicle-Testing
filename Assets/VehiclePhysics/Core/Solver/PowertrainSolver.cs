using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class PowertrainSolver
    {
        private struct FixedFloat16
        {
            private float4 _v0, _v1, _v2, _v3;

            public float Get(int i)
            {
                float4 v = (i >> 2) switch { 0 => _v0, 1 => _v1, 2 => _v2, _ => _v3 };
                return (i & 3) switch { 0 => v.x, 1 => v.y, 2 => v.z, _ => v.w };
            }

            public void Set(int i, float val)
            {
                switch (i >> 2)
                {
                    case 0: SetComp(ref _v0, i & 3, val); break;
                    case 1: SetComp(ref _v1, i & 3, val); break;
                    case 2: SetComp(ref _v2, i & 3, val); break;
                    default: SetComp(ref _v3, i & 3, val); break;
                }
            }

            private static void SetComp(ref float4 v, int c, float val)
            {
                switch (c)
                {
                    case 0: v.x = val; break;
                    case 1: v.y = val; break;
                    case 2: v.z = val; break;
                    default: v.w = val; break;
                }
            }
        }

        public static void SolveSubstep(
            in VehicleState veh,
            in VehicleConfig vCfg,
            in PowertrainConfig ptCfg,
            in VehicleDriverInput driver,
            ref PowertrainState pt,
            NativeArray<WheelConfig> wheelConfigs,
            NativeArray<WheelInput> wheelInputs,
            NativeArray<ContactManifold> contactManifolds,
            NativeArray<SurfaceMaterialData> surfaceMaterials,
            NativeArray<WheelState> wheels,
            in FixedStepTireCache tireCache,
            float dtSub)
        {
            int wheelCount = math.min(veh.WheelCount, 16);
            int start = veh.WheelStartIndex;
            int cfgStart = vCfg.WheelConfigStartIndex;

            // -----------------------------------------------------------------
            // 1. Transmission, Shift Timer, Rev Limiter, TCS & Clutch Control
            // -----------------------------------------------------------------
            float rawThrottle = math.saturate(driver.Throttle);
            bool autoClutchEnabled = (pt.DrivelineFlags & DrivelineFlagBits.AutoClutch) != 0;

            if ((pt.DrivelineFlags & DrivelineFlagBits.EngineStalled) != 0)
            {
                bool starterPressed = (driver.InputFlags & (1 << 2)) != 0;
                if (starterPressed || (autoClutchEnabled && rawThrottle > 0.05f))
                {
                    pt.DrivelineFlags = (byte)(pt.DrivelineFlags & ~DrivelineFlagBits.EngineStalled);
                    pt.EngineOmega = math.max(pt.EngineOmega, ptCfg.IdleOmega * 0.6f);
                }
            }

            UpdateGearboxState(in veh, in ptCfg, in driver, rawThrottle, ref pt, dtSub);

            float gearRatio = ptCfg.GetGearRatio(pt.CurrentGear);
            float effectiveRatio = gearRatio * ptCfg.FinalDriveRatio;
            pt.EffectiveGearRatio = effectiveRatio;

            var driveShare = new FixedFloat16();
            float attachedShareSum = 0f;
            for (int w = 0; w < wheelCount; w++)
            {
                int slot = start + w;
                bool detached = (wheels[slot].WheelFlags & WheelFlagBits.IsDetached) != 0;
                float s = detached ? 0f : math.max(0f, wheelInputs[slot].DriveTorqueShare);
                driveShare.Set(w, s);
                attachedShareSum += s;
            }

            if (attachedShareSum > 1e-5f)
            {
                float invShareSum = 1.0f / attachedShareSum;
                for (int w = 0; w < wheelCount; w++)
                    driveShare.Set(w, driveShare.Get(w) * invShareSum);
            }

            float omegaCarrier = 0f;
            for (int w = 0; w < wheelCount; w++)
                omegaCarrier += driveShare.Get(w) * wheels[start + w].AngularVelocity;
            float kinematicGearboxOmega = effectiveRatio * omegaCarrier;

            // Traction Control (TCS)
            float tcsScale = 1.0f;
            bool tcsAllowed = (driver.InputFlags & (1 << 1)) == 0;
            if (tcsAllowed && rawThrottle > 0.05f)
            {
                float maxExcessRatio = 0f;
                for (int w = 0; w < wheelCount; w++)
                {
                    int slot = start + w;
                    WheelInput wIn = wheelInputs[slot];
                    WheelState ws = wheels[slot];
                    if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0) continue;

                    bool driven = (wIn.ControlFlags & WheelControlFlagBits.IsDriven) != 0;
                    bool allowTcs = (wIn.ControlFlags & WheelControlFlagBits.AllowTcs) != 0;
                    bool grounded = (ws.WheelFlags & WheelFlagBits.IsGrounded) != 0;

                    if (driven && allowTcs && grounded)
                    {
                        float peakK = math.max(wheelConfigs[cfgStart + w].PeakSlipRatio, 0.05f);
                        float excess = (ws.EffectiveSlipRatio - peakK * 1.25f) / peakK;
                        if (excess > 0f)
                        {
                            ws.WheelFlags |= WheelFlagBits.TcsActive;
                            maxExcessRatio = math.max(maxExcessRatio, excess);
                        }
                        else
                        {
                            ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.TcsActive);
                        }
                        wheels[slot] = ws;
                    }
                }
                if (maxExcessRatio > 0f)
                    tcsScale = math.clamp(1.0f - maxExcessRatio * 0.65f, 0.1f, 1.0f);
            }

            // Rev Limiter Hysteresis
            if (pt.EngineOmega >= ptCfg.RedlineOmega)
            {
                pt.DrivelineFlags |= DrivelineFlagBits.RevLimiterCut;
                pt.RevLimiterTimer = math.max(pt.RevLimiterTimer, ptCfg.RevLimiterCutTimeS);
            }
            if ((pt.DrivelineFlags & DrivelineFlagBits.RevLimiterCut) != 0)
            {
                pt.RevLimiterTimer -= dtSub;
                if (pt.RevLimiterTimer <= 0f &&
                    pt.EngineOmega <= ptCfg.RedlineOmega - ptCfg.RevLimiterDropOmega)
                {
                    pt.DrivelineFlags = (byte)(pt.DrivelineFlags & ~DrivelineFlagBits.RevLimiterCut);
                    pt.RevLimiterTimer = 0f;
                }
            }

            bool shifting = (pt.DrivelineFlags & DrivelineFlagBits.IsShifting) != 0;
            bool revCut = (pt.DrivelineFlags & DrivelineFlagBits.RevLimiterCut) != 0;
            bool stalled = (pt.DrivelineFlags & DrivelineFlagBits.EngineStalled) != 0;

            float targetThrottle = rawThrottle * tcsScale;
            if (revCut || stalled)
            {
                targetThrottle = 0f;
            }
            else if (shifting)
            {
                targetThrottle = pt.TargetGear < pt.CurrentGear ? math.max(0.25f, targetThrottle * 0.3f) : 0f;
            }
            else
            {
                float idleAssist = math.saturate((ptCfg.IdleOmega - pt.EngineOmega) / math.max(ptCfg.IdleOmega * 0.25f, 10f)) * 0.45f;
                targetThrottle = math.max(targetThrottle, idleAssist);
            }

            float throttleRate = 1.0f - math.exp(-35.0f * dtSub);
            pt.ThrottleFiltered = math.lerp(pt.ThrottleFiltered, targetThrottle, throttleRate);

            float manualEngage = 1.0f - math.saturate(driver.ClutchPedal);
            float clutchEngage;
            if (pt.CurrentGear == 0)
            {
                clutchEngage = 0f;
            }
            else if (shifting)
            {
                float halfShift = 0.5f * math.max(ptCfg.ShiftDurationS, 0.02f);
                clutchEngage = pt.ShiftTimer > halfShift
                    ? 0f
                    : math.saturate(1.0f - pt.ShiftTimer / halfShift) * manualEngage;
            }
            else if (autoClutchEnabled)
            {
                float absGbx = math.abs(kinematicGearboxOmega);
                if (absGbx >= ptCfg.IdleOmega * 0.9f)
                {
                    clutchEngage = manualEngage;
                }
                else
                {
                    float launchRamp = math.saturate(
                        (pt.EngineOmega - ptCfg.IdleOmega * 0.92f) / math.max(ptCfg.IdleOmega * 0.95f, 20f));
                    if (rawThrottle < 0.03f && absGbx < ptCfg.IdleOmega * 0.5f)
                        launchRamp = 0f;
                    clutchEngage = math.min(manualEngage, launchRamp);
                }
            }
            else
            {
                clutchEngage = manualEngage;
            }
            pt.ClutchEngagement = clutchEngage;

            // -----------------------------------------------------------------
            // 2. Gather Wheel Inertias, Road Impulses & Brake Limits
            // -----------------------------------------------------------------
            var iEff = new FixedFloat16();
            var invIEff = new FixedFloat16();
            var wsInitOmega = new FixedFloat16();
            var jRoad = new FixedFloat16();
            var jExtRoadRr = new FixedFloat16();
            var jRrMaxArr = new FixedFloat16();
            var jBrkMax = new FixedFloat16();
            var jBrkAccum = new FixedFloat16();

            bool absAllowed = (driver.InputFlags & (1 << 0)) == 0;
            float invDt = 1.0f / math.max(dtSub, 1e-6f);

            for (int w = 0; w < wheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];
                wsInitOmega.Set(w, ws.AngularVelocity);

                if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0)
                {
                    ws.AngularVelocity = 0f;
                    ws.NetDriveTorqueNm = 0f;
                    ws.BrakeTorqueAppliedNm = 0f;
                    ws.WarmStartBrakeImpulse = 0f;
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~(WheelFlagBits.AbsActive | WheelFlagBits.IsBrakeLocked));
                    wheels[slot] = ws;

                    iEff.Set(w, 1f);
                    invIEff.Set(w, 0f);
                    jRoad.Set(w, 0f);
                    jRrMaxArr.Set(w, 0f);
                    jBrkMax.Set(w, 0f);
                    continue;
                }

                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                WheelInput wIn = wheelInputs[slot];
                ContactManifold manifold = contactManifolds[slot];

                bool grounded = (ws.WheelFlags & WheelFlagBits.IsGrounded) != 0 && manifold.IsGrounded != 0;
                TireStepResult tr = tireCache.Get(w);

                float inertiaW = math.max(wCfg.WheelInertiaKgM2, 0.2f)
                               + (grounded ? dtSub * ws.EffectiveRollingRadius * tr.dFx_dOmega : 0f);
                float invIW = 1.0f / inertiaW;

                float jRoadW = grounded ? (-tr.Fx * ws.EffectiveRollingRadius * dtSub) : 0f;

                float jRrMax = 0f;
                if (grounded)
                {
                    int matIdx = math.clamp(manifold.PrimaryMaterialIndex, 0, surfaceMaterials.Length - 1);
                    float rrCoeff = surfaceMaterials[matIdx].RollingResistance;
                    jRrMax = math.max(0f, rrCoeff * ws.GroundLoadN * ws.EffectiveRollingRadius) * dtSub;
                }

                // ABS modulation
                float serviceBrk = wIn.ServiceBrakeTorqueNm;
                if (absAllowed && (wIn.ControlFlags & WheelControlFlagBits.AllowAbs) != 0 &&
                    grounded && serviceBrk > 1f && math.abs(veh.ForwardSpeedMs) > 2.5f)
                {
                    float peakK = math.max(wCfg.PeakSlipRatio, 0.05f);
                    if (ws.EffectiveSlipRatio < -peakK * 1.25f)
                    {
                        float excess = (-ws.EffectiveSlipRatio - peakK * 1.25f) / peakK;
                        serviceBrk *= math.clamp(1.0f - excess * 0.75f, 0.15f, 1.0f);
                        ws.WheelFlags |= WheelFlagBits.AbsActive;
                    }
                    else
                    {
                        ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.AbsActive);
                    }
                }
                else
                {
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.AbsActive);
                }

                float maxBrakeTorque = math.max(0f, serviceBrk + wIn.HandbrakeTorqueNm);

                iEff.Set(w, inertiaW);
                invIEff.Set(w, invIW);
                jRoad.Set(w, jRoadW);
                jRrMaxArr.Set(w, jRrMax);
                jBrkMax.Set(w, maxBrakeTorque * dtSub);

                wheels[slot] = ws;
            }

            // Engine Free-Velocity Prediction
            float ie = math.max(ptCfg.EngineInertia, 0.05f);
            float invIe = 1.0f / ie;
            float netEngineTorque = stalled
                ? -(ptCfg.EngineFrictionBaseNm + ptCfg.EngineFrictionViscous * math.max(pt.EngineOmega, 0f))
                : ptCfg.EvaluateNetEngineTorque(pt.EngineOmega, pt.ThrottleFiltered);

            pt.EngineTorqueNm = netEngineTorque;
            float omegaE = math.max(0f, pt.EngineOmega + netEngineTorque * invIe * dtSub);

            // -----------------------------------------------------------------
            // 3. Coupled [Wheels + Gearbox] Sherman-Morrison Inverse Mass M^-1
            // -----------------------------------------------------------------
            bool gearEngaged = (effectiveRatio != 0f) && (attachedShareSum > 1e-5f);
            bool clutchActive = gearEngaged && (clutchEngage > 0.001f);

            float cInv = 0f;
            float betaGbx = 1.0f;
            float alphaSm = 0f; // iGbx * effectiveRatio^2 * betaGbx
            float invIDrive = 0f;

            if (gearEngaged)
            {
                for (int w = 0; w < wheelCount; w++)
                {
                    float s = driveShare.Get(w);
                    float invW = invIEff.Get(w);
                    if (s > 0f && invW > 0f)
                        cInv += (effectiveRatio * effectiveRatio) * (s * s) * invW;
                }
                float iGbx = math.max(ptCfg.GearboxInertia, 0f);
                betaGbx = 1.0f / (1.0f + iGbx * cInv);
                alphaSm = iGbx * effectiveRatio * effectiveRatio * betaGbx;
                invIDrive = cInv * betaGbx;
            }

            var qVec = new FixedFloat16();
            var invMDiag = new FixedFloat16();
            for (int w = 0; w < wheelCount; w++)
            {
                float invW = invIEff.Get(w);
                float q = gearEngaged ? (invW * driveShare.Get(w)) : 0f;
                qVec.Set(w, q);
                invMDiag.Set(w, invW > 0f ? math.max(invW - alphaSm * q * q, 1e-9f) : 0f);
            }

            // Apply Road Torque via exact M^-1 so gearbox inertia couples symmetrically to road & clutch
            var omegaWork = new FixedFloat16();
            float qDotRoad = 0f;
            for (int w = 0; w < wheelCount; w++)
                qDotRoad += qVec.Get(w) * jRoad.Get(w);

            for (int w = 0; w < wheelCount; w++)
            {
                float invW = invIEff.Get(w);
                float wAfter = wsInitOmega.Get(w) + jRoad.Get(w) * invW - alphaSm * qDotRoad * qVec.Get(w);
                omegaWork.Set(w, wAfter);
            }

            // Audit Fix #4: Passive zero-clamped rolling resistance via exact M^-1
            for (int w = 0; w < wheelCount; w++)
            {
                float dJRr = 0f;
                float jRrMax = jRrMaxArr.Get(w);
                if (jRrMax > 0f && invMDiag.Get(w) > 0f)
                {
                    dJRr = math.clamp(-omegaWork.Get(w) / invMDiag.Get(w), -jRrMax, jRrMax);
                    ApplyWheelImpulseExact(
                        w, dJRr, wheelCount, alphaSm, in invIEff, in qVec, ref omegaWork);
                }
                jExtRoadRr.Set(w, jRoad.Get(w) + dJRr);
            }

            float hysteresis = pt.ClutchLockState == ClutchLockStates.Locked
                ? math.max(ptCfg.ClutchStaticHysteresis, 1.0f)
                : 1.0f;
            float jClutchMax = (clutchActive && invIDrive > 0f)
                ? clutchEngage * hysteresis * ptCfg.ClutchMaxTorqueNm * dtSub
                : 0f;

            // Audit Fix #2: Keep PGS kinematic clutch-to-wheel transfer exact (eta_d = 1.0),
            // and apply (1 - eta_d) + axle churning as an explicit passive carrier drag via M^-1.
            float absG = math.abs(effectiveRatio);
            float effEta = ptCfg.DrivelineEfficiency > 0f
                ? math.clamp(ptCfg.DrivelineEfficiency, 0.5f, 1.0f)
                : 1.0f;
            float baseDragAtWheelsNm = attachedShareSum > 1e-5f
                ? math.max(0f, ptCfg.DrivelineBaseDragNm) * math.max(1.0f, math.abs(ptCfg.FinalDriveRatio))
                : 0f;

            float carrierAfterRoad = 0f;
            for (int w = 0; w < wheelCount; w++)
                carrierAfterRoad += driveShare.Get(w) * omegaWork.Get(w);

            float jMeshLossTotal = 0f;
            if (clutchActive && jClutchMax > 0f)
            {
                float jClutchEst = math.clamp(
                    (omegaE - effectiveRatio * carrierAfterRoad) / (invIe + invIDrive),
                    -jClutchMax,
                    jClutchMax);
                jMeshLossTotal = (1.0f - effEta) * math.abs(jClutchEst) * absG;
            }

            float jTotalDrivelineLoss = jMeshLossTotal + baseDragAtWheelsNm * dtSub;
            if (gearEngaged && invIDrive > 1e-9f && jTotalDrivelineLoss > 0f)
            {
                float iCarrierEff = (effectiveRatio * effectiveRatio) / invIDrive;
                float dJCarrierLoss = math.clamp(
                    -iCarrierEff * carrierAfterRoad,
                    -jTotalDrivelineLoss,
                    jTotalDrivelineLoss);

                if (math.abs(dJCarrierLoss) > 1e-9f)
                {
                    for (int w = 0; w < wheelCount; w++)
                    {
                        float q = qVec.Get(w);
                        if (q <= 0f) continue;
                        omegaWork.Set(w, omegaWork.Get(w) + betaGbx * dJCarrierLoss * q);
                    }
                }
            }

            // -----------------------------------------------------------------
            // 4. Warm-Started PGS Solver (Audit Fix #1: Accumulated Clamps)
            // -----------------------------------------------------------------
            float jClutchAccum = 0f;
            if (clutchActive && jClutchMax > 0f)
            {
                jClutchAccum = math.clamp(pt.WarmStartClutchImpulse, -jClutchMax, jClutchMax);
                ApplyClutchImpulse(
                    jClutchAccum, effectiveRatio, invIe, betaGbx, wheelCount,
                    in qVec, ref omegaE, ref omegaWork);
            }

            for (int w = 0; w < wheelCount; w++)
            {
                if (invMDiag.Get(w) <= 0f) continue;
                float bMax = jBrkMax.Get(w);
                float jWarm = math.clamp(wheels[start + w].WarmStartBrakeImpulse, -bMax, bMax);
                jBrkAccum.Set(w, jWarm);
                ApplyWheelImpulseExact(w, jWarm, wheelCount, alphaSm, in invIEff, in qVec, ref omegaWork);
            }

            float4 jLsdAccum = float4.zero;

            const int pgsIterations = 2;
            for (int iter = 0; iter < pgsIterations; iter++)
            {
                if (clutchActive && jClutchMax > 0f && invIDrive > 0f)
                {
                    float wCarrier = 0f;
                    for (int w = 0; w < wheelCount; w++)
                        wCarrier += driveShare.Get(w) * omegaWork.Get(w);

                    float wGbx = effectiveRatio * wCarrier;
                    float jLockDelta = (omegaE - wGbx) / (invIe + invIDrive);

                    float jOld = jClutchAccum;
                    jClutchAccum = math.clamp(jOld + jLockDelta, -jClutchMax, jClutchMax);
                    float dJ = jClutchAccum - jOld;

                    ApplyClutchImpulse(
                        dJ, effectiveRatio, invIe, betaGbx, wheelCount,
                        in qVec, ref omegaE, ref omegaWork);
                }

                float clutchTorqueEst = math.abs(jClutchAccum) * invDt;
                SolveAxleLsdConstraints(
                    in veh, in vCfg, in ptCfg, wheelConfigs, wheels, clutchTorqueEst, dtSub,
                    alphaSm, in driveShare, in invIEff, in qVec, ref omegaWork, ref jLsdAccum);

                for (int w = 0; w < wheelCount; w++)
                {
                    float invMd = invMDiag.Get(w);
                    if (invMd <= 0f) continue;
                    float bMax = jBrkMax.Get(w);
                    if (bMax <= 0f) continue;

                    float dJStop = -omegaWork.Get(w) / invMd;

                    float jOld = jBrkAccum.Get(w);
                    float jNew = math.clamp(jOld + dJStop, -bMax, bMax);
                    float dJ = jNew - jOld;

                    jBrkAccum.Set(w, jNew);
                    ApplyWheelImpulseExact(w, dJ, wheelCount, alphaSm, in invIEff, in qVec, ref omegaWork);
                }
            }

            // -----------------------------------------------------------------
            // 5. Write Final Powertrain & Wheel Velocities / Telemetry
            // -----------------------------------------------------------------
            omegaE = math.max(0f, omegaE);
            if (!stalled && !autoClutchEnabled && clutchEngage > 0.5f && omegaE < ptCfg.IdleOmega * 0.25f)
            {
                pt.DrivelineFlags |= DrivelineFlagBits.EngineStalled;
            }

            float finalCarrier = 0f;
            for (int w = 0; w < wheelCount; w++)
                finalCarrier += driveShare.Get(w) * omegaWork.Get(w);

            if (gearRatio != 0f)
            {
                pt.DriveshaftOmega = ptCfg.FinalDriveRatio * finalCarrier;
                pt.GearboxInputOmega = gearRatio * pt.DriveshaftOmega;
            }
            else
            {
                pt.DriveshaftOmega = ptCfg.FinalDriveRatio * finalCarrier;
                pt.GearboxInputOmega *= 1.0f / (1.0f + 2.5f * dtSub);
            }

            pt.EngineOmega = omegaE;
            pt.ClutchSlipOmega = omegaE - pt.GearboxInputOmega;
            pt.ClutchTorqueNm = jClutchAccum * invDt;
            pt.WarmStartClutchImpulse = jClutchAccum;
            pt.WarmStartFrontLsdImpulse = jLsdAccum.x;
            pt.WarmStartRearLsdImpulse = jLsdAccum.y;

            if (!clutchActive)
                pt.ClutchLockState = ClutchLockStates.Open;
            else if (math.abs(jClutchAccum) < jClutchMax * 0.995f && math.abs(pt.ClutchSlipOmega) < 0.5f)
                pt.ClutchLockState = ClutchLockStates.Locked;
            else
                pt.ClutchLockState = ClutchLockStates.Slipping;

            pt.CrankAngleRad = math.fmod(pt.CrankAngleRad + omegaE * dtSub, math.PI * 4.0f);
            if (pt.CrankAngleRad < 0f) pt.CrankAngleRad += math.PI * 4.0f;

            for (int w = 0; w < wheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];
                if ((ws.WheelFlags & WheelFlagBits.IsDetached) != 0) continue;

                float wFinal = omegaWork.Get(w);
                float jBrk = jBrkAccum.Get(w);
                float jAxleDrive = (wFinal - wsInitOmega.Get(w)) * iEff.Get(w) - jExtRoadRr.Get(w) - jBrk;

                ws.AngularVelocity = wFinal;
                ws.NetDriveTorqueNm = jAxleDrive * invDt;
                ws.BrakeTorqueAppliedNm = math.abs(jBrk) * invDt;
                ws.WarmStartBrakeImpulse = jBrk;

                if (math.abs(wFinal) <= 1e-4f && jBrkMax.Get(w) > dtSub)
                    ws.WheelFlags |= WheelFlagBits.IsBrakeLocked;
                else
                    ws.WheelFlags = (byte)(ws.WheelFlags & ~WheelFlagBits.IsBrakeLocked);

                ws.SpinAngle = math.fmod(ws.SpinAngle + wFinal * dtSub, math.PI * 2.0f);
                if (ws.SpinAngle < 0f) ws.SpinAngle += math.PI * 2.0f;

                wheels[slot] = ws;
            }
        }

        private static void ApplyWheelImpulseExact(
            int wheelIdx,
            float impulse,
            int wheelCount,
            float alphaSm,
            in FixedFloat16 invIEff,
            in FixedFloat16 qVec,
            ref FixedFloat16 omegaWork)
        {
            if (math.abs(impulse) < 1e-11f) return;

            omegaWork.Set(wheelIdx, omegaWork.Get(wheelIdx) + impulse * invIEff.Get(wheelIdx));
            if (alphaSm <= 0f) return;

            float qw = qVec.Get(wheelIdx);
            if (qw <= 0f) return;

            float scale = alphaSm * qw * impulse;
            for (int k = 0; k < wheelCount; k++)
            {
                float qk = qVec.Get(k);
                if (qk > 0f)
                    omegaWork.Set(k, omegaWork.Get(k) - scale * qk);
            }
        }

        private static void ApplyClutchImpulse(
            float impulse,
            float effectiveRatio,
            float invIe,
            float betaGbx,
            int wheelCount,
            in FixedFloat16 qVec,
            ref float omegaE,
            ref FixedFloat16 omegaWork)
        {
            if (math.abs(impulse) < 1e-9f) return;

            omegaE -= impulse * invIe;
            float gJ = effectiveRatio * (impulse * betaGbx);

            for (int w = 0; w < wheelCount; w++)
            {
                float q = qVec.Get(w);
                if (q > 0f)
                    omegaWork.Set(w, omegaWork.Get(w) + gJ * q);
            }
        }

        private static void SolveAxleLsdConstraints(
            in VehicleState veh,
            in VehicleConfig vCfg,
            in PowertrainConfig ptCfg,
            NativeArray<WheelConfig> wheelConfigs,
            NativeArray<WheelState> wheels,
            float clutchTorqueEst,
            float dtSub,
            float alphaSm,
            in FixedFloat16 driveShare,
            in FixedFloat16 invIEff,
            in FixedFloat16 qVec,
            ref FixedFloat16 omegaWork,
            ref float4 jLsdAccum)
        {
            int wheelCount = math.min(veh.WheelCount, 16);
            int start = veh.WheelStartIndex;
            int cfgStart = vCfg.WheelConfigStartIndex;

            for (byte axle = 0; axle < 4; axle++)
            {
                int leftIdx = -1;
                int rightIdx = -1;
                float axleShareSum = 0f;

                for (int w = 0; w < wheelCount; w++)
                {
                    if ((wheels[start + w].WheelFlags & WheelFlagBits.IsDetached) != 0) continue;
                    if (driveShare.Get(w) <= 0f) continue;

                    WheelConfig wCfg = wheelConfigs[cfgStart + w];
                    if (wCfg.AxleIndex != axle) continue;

                    axleShareSum += driveShare.Get(w);
                    if (wCfg.SideIndex == 0 && leftIdx < 0) leftIdx = w;
                    else if (wCfg.SideIndex == 1 && rightIdx < 0) rightIdx = w;
                }

                if (leftIdx < 0 || rightIdx < 0) continue;

                float preload = axle == 0 ? ptCfg.FrontLsdPreloadNm : ptCfg.RearLsdPreloadNm;
                float bias = axle == 0 ? ptCfg.FrontLsdLockBias : ptCfg.RearLsdLockBias;
                float viscous = axle == 0 ? ptCfg.FrontLsdViscous : ptCfg.RearLsdViscous;

                float wL = omegaWork.Get(leftIdx);
                float wR = omegaWork.Get(rightIdx);
                float slipDiff = wL - wR;

                float torqueThroughAxle = clutchTorqueEst * math.abs(ptCfg.FinalDriveRatio) * axleShareSum;
                float lsdMaxTorque = preload + bias * torqueThroughAxle + viscous * math.abs(slipDiff);
                float jLsdMax = lsdMaxTorque * dtSub;
                if (jLsdMax <= 1e-6f) continue;

                float qDiff = qVec.Get(leftIdx) - qVec.Get(rightIdx);
                float invSum = invIEff.Get(leftIdx) + invIEff.Get(rightIdx) - alphaSm * qDiff * qDiff;
                float dJLock = slipDiff / math.max(invSum, 1e-6f);

                float jOld = jLsdAccum[axle];
                float jNew = math.clamp(jOld + dJLock, -jLsdMax, jLsdMax);
                float dJ = jNew - jOld;
                jLsdAccum[axle] = jNew;

                if (math.abs(dJ) > 1e-11f)
                {
                    omegaWork.Set(leftIdx, wL - dJ * invIEff.Get(leftIdx));
                    omegaWork.Set(rightIdx, wR + dJ * invIEff.Get(rightIdx));
                    if (alphaSm > 0f && math.abs(qDiff) > 1e-9f)
                    {
                        float scale = alphaSm * qDiff * dJ;
                        for (int k = 0; k < wheelCount; k++)
                        {
                            float qk = qVec.Get(k);
                            if (qk > 0f)
                                omegaWork.Set(k, omegaWork.Get(k) + scale * qk);
                        }
                    }
                }

                // Rigidly bolt any dual/inner tires sharing the same axle and side
                for (int w = 0; w < wheelCount; w++)
                {
                    if (w == leftIdx || w == rightIdx) continue;
                    if ((wheels[start + w].WheelFlags & WheelFlagBits.IsDetached) != 0) continue;
                    if (driveShare.Get(w) <= 0f) continue;

                    WheelConfig wCfg = wheelConfigs[cfgStart + w];
                    if (wCfg.AxleIndex != axle) continue;

                    int primary = wCfg.SideIndex == 0 ? leftIdx : rightIdx;
                    float wP = omegaWork.Get(primary);
                    float wSec = omegaWork.Get(w);
                    float qPair = qVec.Get(primary) - qVec.Get(w);
                    float invBolt = invIEff.Get(primary) + invIEff.Get(w) - alphaSm * qPair * qPair;
                    float jBolt = (wP - wSec) / math.max(invBolt, 1e-6f);
                    omegaWork.Set(primary, wP - jBolt * invIEff.Get(primary));
                    omegaWork.Set(w, wSec + jBolt * invIEff.Get(w));
                    if (alphaSm > 0f && math.abs(qPair) > 1e-9f)
                    {
                        float scale = alphaSm * qPair * jBolt;
                        for (int k = 0; k < wheelCount; k++)
                        {
                            float qk = qVec.Get(k);
                            if (qk > 0f)
                                omegaWork.Set(k, omegaWork.Get(k) + scale * qk);
                        }
                    }
                }
            }
        }

        private static void UpdateGearboxState(
            in VehicleState veh,
            in PowertrainConfig ptCfg,
            in VehicleDriverInput driver,
            float rawThrottle,
            ref PowertrainState pt,
            float dtSub)
        {
            bool isShifting = (pt.DrivelineFlags & DrivelineFlagBits.IsShifting) != 0;
            if (isShifting)
            {
                pt.ShiftTimer -= dtSub;
                float halfShift = 0.5f * math.max(ptCfg.ShiftDurationS, 0.02f);
                if (pt.ShiftTimer <= halfShift && pt.CurrentGear != pt.TargetGear)
                {
                    pt.CurrentGear = pt.TargetGear;
                    pt.WarmStartClutchImpulse = 0f;
                }
                if (pt.ShiftTimer <= 0f)
                {
                    pt.ShiftTimer = 0f;
                    pt.DrivelineFlags = (byte)(pt.DrivelineFlags & ~DrivelineFlagBits.IsShifting);
                }
                return;
            }

            sbyte desired = pt.CurrentGear;

            if (driver.RequestedGear >= -1 && driver.RequestedGear <= ptCfg.ForwardGearCount)
            {
                desired = driver.RequestedGear;
            }
            else if (driver.ShiftDelta != 0)
            {
                int next = math.clamp(pt.CurrentGear + driver.ShiftDelta, -1, ptCfg.ForwardGearCount);
                desired = (sbyte)next;
            }
            else if (ptCfg.AutoTransmission != 0 && driver.RequestedGear == -128)
            {
                if (pt.CurrentGear == 0)
                {
                    if (rawThrottle > 0.05f && veh.ForwardSpeedMs >= -0.5f)
                        desired = 1;
                }
                else if (pt.CurrentGear >= 1)
                {
                    if (pt.EngineOmega >= ptCfg.AutoShiftUpOmega &&
                        pt.CurrentGear < ptCfg.ForwardGearCount &&
                        pt.ClutchLockState == ClutchLockStates.Locked &&
                        rawThrottle > 0.1f)
                    {
                        desired = (sbyte)(pt.CurrentGear + 1);
                    }
                    else if (pt.EngineOmega <= ptCfg.AutoShiftDownOmega && pt.CurrentGear > 1)
                    {
                        desired = (sbyte)(pt.CurrentGear - 1);
                    }
                }
            }

            if (desired != pt.CurrentGear)
            {
                pt.TargetGear = desired;
                pt.ShiftTimer = math.max(ptCfg.ShiftDurationS, 0.02f);
                pt.DrivelineFlags |= DrivelineFlagBits.IsShifting;
            }
        }
    }
}
