using Unity.Collections;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    public static class SteeringAndInputSolver
    {
        public static void PrepareWheelInputsForFixedStep(
            in VehicleState veh,
            in VehicleConfig vCfg,
            in PowertrainConfig ptCfg,
            in VehicleDriverInput driver,
            NativeArray<WheelConfig> wheelConfigs,
            NativeArray<WheelInput> wheelInputs)
        {
            float absForwardSpeed = math.abs(veh.ForwardSpeedMs);
            float speedScale = 1.0f / (1.0f + vCfg.HighSpeedSteerScale * absForwardSpeed * absForwardSpeed);
            float centerSteer = math.clamp(driver.Steer, -1.0f, 1.0f) * vCfg.MaxSteerAngleRad * speedScale;

            float absSteer = math.abs(centerSteer);
            float steerSign = math.sign(centerSteer);
            float halfTrackFront = 0.5f * math.max(vCfg.TrackWidthFrontM, 0.5f);
            float wheelbase = math.max(vCfg.WheelbaseM, 0.5f);

            float tanAbs = math.tan(absSteer);
            float ackInside = absSteer;
            float ackOutside = absSteer;
            if (tanAbs > 1e-4f)
            {
                float ratio = halfTrackFront / wheelbase * tanAbs;
                float rawIn = math.atan(tanAbs / math.max(1.0f - ratio, 0.15f));
                float rawOut = math.atan(tanAbs / (1.0f + ratio));
                ackInside = math.lerp(absSteer, rawIn, math.saturate(vCfg.AckermannFactor));
                ackOutside = math.lerp(absSteer, rawOut, math.saturate(vCfg.AckermannFactor));
            }

            float brakeInput = math.saturate(driver.Brake);
            float handbrakeInput = math.saturate(driver.Handbrake);

            int start = veh.WheelStartIndex;
            int cfgStart = vCfg.WheelConfigStartIndex;

            // Count driven wheels per axle so multi-wheel bodies (e.g., 10-wheel semi) split torque cleanly.
            int frontDrivenCount = 0;
            int rearDrivenCount = 0;
            for (int w = 0; w < veh.WheelCount; w++)
            {
                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                if ((wCfg.DefaultControlFlags & WheelControlFlagBits.IsDriven) == 0) continue;
                if (wCfg.AxleIndex == 0) frontDrivenCount++;
                else rearDrivenCount++;
            }

            float frontTotalShare = (frontDrivenCount > 0 && rearDrivenCount > 0)
                ? math.saturate(ptCfg.FrontAxleTorqueSplit)
                : (frontDrivenCount > 0 ? 1.0f : 0.0f);
            float rearTotalShare = (frontDrivenCount > 0 && rearDrivenCount > 0)
                ? (1.0f - frontTotalShare)
                : (rearDrivenCount > 0 ? 1.0f : 0.0f);

            for (int w = 0; w < veh.WheelCount; w++)
            {
                WheelConfig wCfg = wheelConfigs[cfgStart + w];
                WheelInput wIn = wheelInputs[start + w];

                if ((wIn.ControlFlags & WheelControlFlagBits.IsSteerable) != 0)
                {
                    // Positive steer turns Right (+X): SideIndex 1 (Right) is inside wheel when steerSign > 0.
                    bool isInside = (steerSign > 0f && wCfg.SideIndex == 1) || (steerSign < 0f && wCfg.SideIndex == 0);
                    wIn.TargetSteerAngleRad = steerSign * (isInside ? ackInside : ackOutside);
                }
                else
                {
                    wIn.TargetSteerAngleRad = 0f;
                }

                wIn.ServiceBrakeTorqueNm = brakeInput * wCfg.MaxServiceBrakeTorqueNm;
                wIn.HandbrakeTorqueNm = handbrakeInput * wCfg.MaxHandbrakeTorqueNm;

                if ((wIn.ControlFlags & WheelControlFlagBits.IsDriven) != 0)
                {
                    wIn.DriveTorqueShare = (wCfg.AxleIndex == 0)
                        ? (frontDrivenCount > 0 ? frontTotalShare / frontDrivenCount : 0f)
                        : (rearDrivenCount > 0 ? rearTotalShare / rearDrivenCount : 0f);
                }
                else
                {
                    wIn.DriveTorqueShare = 0f;
                }

                wheelInputs[start + w] = wIn;
            }
        }

        public static void StepWheelSteering(
            in VehicleState veh,
            in VehicleConfig vCfg,
            float dtSub,
            NativeArray<WheelInput> wheelInputs,
            NativeArray<WheelState> wheels)
        {
            float maxDelta = math.max(vCfg.SteeringRateRadPerS, 0.5f) * dtSub;
            int start = veh.WheelStartIndex;

            for (int w = 0; w < veh.WheelCount; w++)
            {
                int slot = start + w;
                WheelState ws = wheels[slot];
                float target = wheelInputs[slot].TargetSteerAngleRad;
                float delta = math.clamp(target - ws.SteerAngleRad, -maxDelta, maxDelta);
                ws.SteerAngleRad += delta;
                wheels[slot] = ws;
            }
        }
    }
}
