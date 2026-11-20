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
            NativeArray<WheelContact> wheelContacts,
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

            // Compute attached driven share normalization so sum(s_attached) == 1
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

            // Evaluate current wheel carrier speed (weighted sum of attached driven wheels)
            float omegaCarrier = 0f;
            for (int w = 0; w < wheelCount; w++)
            {
                omegaCarrier += driveShare.Get(w) * wheels[start + w].AngularVelocity;
            }
            float kinematicGearboxOmega = effectiveRatio * omegaCarrier;

            // Traction Control (TCS) throttle reduction if attached driven wheels overspin
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

            // Compute Clutch Engagement [0..1]
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
            // 2. Implicit Wheel Stabilization & Free-Velocity Predictions
            // -----------------------------------------------------------------
            var iEff = new FixedFloat16();
            var invIEff = new FixedFloat16();
            var omegaStar = new FixedFloat16();
            var omegaWork = new FixedFloat16();
            var jBrkMax = new FixedFloat16();
            var jBrkAccum = new FixedFloat16();

            bool absAllowed = (driver.InputFlags & (1 << 0)) == 0;
            float invDt = 1.0f / math.max(dtSub, 1e-6f);

            float prevMeshDragTorque = (effectiveRatio != 0f && clutchEngage > 0.01f && attachedShareSum > 1e-5f)
                ? (ptCfg.DrivelineBaseDragNm + (1.0f - math.saturate(ptCfg.DrivelineEfficiency)) * math.abs(pt.ClutchTorqueNm))
                : 0f;

            for (int w = 0; w < wheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];

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
                    omegaStar.Set(w, 0f);
                    omegaWork.Set(w, 0f);
                    jBrkMax.Set(w, 0f);
                    continue;
                }

                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                WheelInput wIn = wheelInputs[slot];
                WheelContact contact = wheelContacts[slot];

                bool grounded = (ws.WheelFlags & WheelFlagBits.IsGrounded) != 0;
                TireStepResult tr = tireCache.Get(w);

                float inertiaW = math.max(wCfg.WheelInertiaKgM2, 0.2f)
                               + (grounded ? dtSub * ws.EffectiveRollingRadius * tr.dFx_dOmega : 0f);
                float invIW = 1.0f / inertiaW;

                float roadTorque = grounded ? (-tr.Fx * ws.EffectiveRollingRadius) : 0f;
                float share = driveShare.Get(w);

                float meshDrag = (share > 0f && prevMeshDragTorque > 0f)
                    ? -math.sign(ws.AngularVelocity) * prevMeshDragTorque * math.abs(effectiveRatio) * share
                    : 0f;

                float wFree = ws.AngularVelocity + (roadTorque + meshDrag) * invIW * dtSub;
                if (meshDrag != 0f && ws.AngularVelocity * wFree < 0f && math.abs(roadTorque) < math.abs(meshDrag))
                    wFree = 0f;

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

                float rollingTorque = grounded
                    ? contact.RollingResistance * ws.GroundLoadN * ws.EffectiveRollingRadius
                    : 0f;
                float maxResistTorque = serviceBrk + wIn.HandbrakeTorqueNm + rollingTorque;

                iEff.Set(w, inertiaW);
                invIEff.Set(w, invIW);
                omegaStar.Set(w, wFree);
                omegaWork.Set(w, wFree);
                jBrkMax.Set(w, maxResistTorque * dtSub);

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
            // 3. Closed-Form Reflected Driveline Inertia & Warm-Started PGS
            // -----------------------------------------------------------------
            bool clutchActive = (effectiveRatio != 0f) && (clutchEngage > 0.001f) && (attachedShareSum > 1e-5f);
            float cInv = 0f;
            float betaGbx = 1.0f;
            float invIDrive = 0f;

            if (clutchActive)
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
                invIDrive = cInv * betaGbx;
            }

            float hysteresis = pt.ClutchLockState == ClutchLockStates.Locked
                ? math.max(ptCfg.ClutchStaticHysteresis, 1.0f)
                : 1.0f;
            float jClutchMax = (clutchActive && invIDrive > 0f)
                ? clutchEngage * hysteresis * ptCfg.ClutchMaxTorqueNm * dtSub
                : 0f;

            float jClutchAccum = 0f;
            if (clutchActive && jClutchMax > 0f)
            {
                jClutchAccum = math.clamp(pt.WarmStartClutchImpulse, -jClutchMax, jClutchMax);
                ApplyClutchImpulse(
                    jClutchAccum, effectiveRatio, invIe, betaGbx, wheelCount,
                    in driveShare, in invIEff, ref omegaE, ref omegaWork);
            }

            for (int w = 0; w < wheelCount; w++)
            {
                if (invIEff.Get(w) <= 0f) continue;
                float bMax = jBrkMax.Get(w);
                float jWarm = math.clamp(wheels[start + w].WarmStartBrakeImpulse, -bMax, bMax);
                jBrkAccum.Set(w, jWarm);
                omegaWork.Set(w, omegaWork.Get(w) + jWarm * invIEff.Get(w));
            }

            float jFrontLsdAccum = 0f;
            float jRearLsdAccum = 0f;

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
                        in driveShare, in invIEff, ref omegaE, ref omegaWork);
                }

                float clutchTorqueEst = math.abs(jClutchAccum) * invDt;
                SolveAxleLsdConstraints(
                    in veh, in vCfg, in ptCfg, wheelConfigs, wheels, clutchTorqueEst, dtSub,
                    in driveShare, in invIEff, ref omegaWork,
                    ref jFrontLsdAccum, ref jRearLsdAccum);

                for (int w = 0; w < wheelCount; w++)
                {
                    if (invIEff.Get(w) <= 0f) continue;
                    float bMax = jBrkMax.Get(w);
                    if (bMax <= 0f) continue;

                    float wCurr = omegaWork.Get(w);
                    float dJStop = -iEff.Get(w) * wCurr;

                    float jOld = jBrkAccum.Get(w);
                    float jNew = math.clamp(jOld + dJStop, -bMax, bMax);
                    float dJ = jNew - jOld;

                    jBrkAccum.Set(w, jNew);
                    omegaWork.Set(w, wCurr + dJ * invIEff.Get(w));
                }
            }

            // -----------------------------------------------------------------
            // 4. Write Final Powertrain & Wheel Velocities / Telemetry
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
            pt.WarmStartFrontLsdImpulse = jFrontLsdAccum;
            pt.WarmStartRearLsdImpulse = jRearLsdAccum;

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
                float jDriveTotal = (wFinal - omegaStar.Get(w)) * iEff.Get(w) - jBrk;

                ws.AngularVelocity = wFinal;
                ws.NetDriveTorqueNm = jDriveTotal * invDt;
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

        private static void ApplyClutchImpulse(
            float impulse,
            float effectiveRatio,
            float invIe,
            float betaGbx,
            int wheelCount,
            in FixedFloat16 driveShare,
            in FixedFloat16 invIEff,
            ref float omegaE,
            ref FixedFloat16 omegaWork)
        {
            if (math.abs(impulse) < 1e-9f) return;

            omegaE -= impulse * invIe;
            float jDiffIn = impulse * betaGbx;
            float gJ = effectiveRatio * jDiffIn;

            for (int w = 0; w < wheelCount; w++)
            {
                float s = driveShare.Get(w);
                float invW = invIEff.Get(w);
                if (s <= 0f || invW <= 0f) continue;
                omegaWork.Set(w, omegaWork.Get(w) + s * gJ * invW);
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
            in FixedFloat16 driveShare,
            in FixedFloat16 invIEff,
            ref FixedFloat16 omegaWork,
            ref float jFrontLsdAccum,
            ref float jRearLsdAccum)
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

                float invSum = invIEff.Get(leftIdx) + invIEff.Get(rightIdx);
                float dJLock = slipDiff / math.max(invSum, 1e-6f);

                if (axle == 0)
                {
                    float jOld = jFrontLsdAccum;
                    jFrontLsdAccum = math.clamp(jOld + dJLock, -jLsdMax, jLsdMax);
                    float dJ = jFrontLsdAccum - jOld;
                    omegaWork.Set(leftIdx, wL - dJ * invIEff.Get(leftIdx));
                    omegaWork.Set(rightIdx, wR + dJ * invIEff.Get(rightIdx));
                }
                else
                {
                    float jOld = jRearLsdAccum;
                    jRearLsdAccum = math.clamp(jOld + dJLock, -jLsdMax, jLsdMax);
                    float dJ = jRearLsdAccum - jOld;
                    omegaWork.Set(leftIdx, wL - dJ * invIEff.Get(leftIdx));
                    omegaWork.Set(rightIdx, wR + dJ * invIEff.Get(rightIdx));
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
                    float jBolt = (wP - wSec) / math.max(invIEff.Get(primary) + invIEff.Get(w), 1e-6f);
                    omegaWork.Set(primary, wP - jBolt * invIEff.Get(primary));
                    omegaWork.Set(w, wSec + jBolt * invIEff.Get(w));
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
