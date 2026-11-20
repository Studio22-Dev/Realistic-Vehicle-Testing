using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace VehiclePhysics.Core
{
    [BurstCompile(FloatMode = FloatMode.Default)]
    public struct VehicleSimulationJob : IJobParallelFor
    {
        public float FixedDeltaTime;

        [ReadOnly] public NativeArray<VehicleConfig> VehicleConfigs;
        [ReadOnly] public NativeArray<PowertrainConfig> PowertrainConfigs;
        [ReadOnly] public NativeArray<WheelConfig> WheelConfigs;
        [ReadOnly] public NativeArray<VehicleDriverInput> DriverInputs;
        [ReadOnly] public NativeArray<WheelContact> WheelContacts;

        public NativeArray<VehicleState> Vehicles;
        public NativeArray<PowertrainState> Powertrains;
        public NativeArray<VehicleRenderPose> RenderPoses;

        [NativeDisableParallelForRestriction] public NativeArray<WheelInput> WheelInputs;
        [NativeDisableParallelForRestriction] public NativeArray<WheelState> Wheels;

        public void Execute(int vehIdx)
        {
            VehicleState veh = Vehicles[vehIdx];
            VehicleRenderPose pose = RenderPoses[vehIdx];
            pose.PreviousPosition = veh.Position;
            pose.PreviousRotation = veh.Rotation;

            if (veh.LodTier >= 3 || (veh.TierFlags & VehicleTierFlags.IsSleeping) != 0)
            {
                pose.CurrentPosition = veh.Position;
                pose.CurrentRotation = veh.Rotation;
                RenderPoses[vehIdx] = pose;
                return;
            }

            VehicleConfig vCfg = VehicleConfigs[veh.ConfigIndex];
            PowertrainConfig ptCfg = PowertrainConfigs[veh.ConfigIndex];
            PowertrainState pt = Powertrains[vehIdx];
            VehicleDriverInput driver = DriverInputs[vehIdx];

            SteeringAndInputSolver.PrepareWheelInputsForFixedStep(
                in veh, in vCfg, in ptCfg, in driver, WheelConfigs, WheelInputs);

            int substeps = math.max((int)veh.SubstepCount, 1);
            float dtSub = FixedDeltaTime / substeps;
            int cfgStart = vCfg.WheelConfigStartIndex;
            bool hasPowertrain = vCfg.HasPowertrain != 0;

            for (int s = 0; s < substeps; s++)
            {
                SteeringAndInputSolver.StepWheelSteering(in veh, in vCfg, dtSub, WheelInputs, Wheels);

                SuspensionSolver.SolveSuspension(ref veh, in vCfg, WheelConfigs, WheelContacts, Wheels);

                var tireCache = new FixedStepTireCache();
                for (int w = 0; w < veh.WheelCount; w++)
                {
                    int slot = veh.WheelStartIndex + w;
                    WheelConfig wCfg = WheelConfigs[cfgStart + w];
                    WheelContact contact = WheelContacts[slot];
                    WheelState ws = Wheels[slot];

                    TireStepResult res = TireSolver.EvaluateTireForces(
                        in veh, in vCfg, in wCfg, in contact, ref ws, dtSub);
                    Wheels[slot] = ws;
                    tireCache.Set(w, in res);
                }

                if (hasPowertrain)
                {
                    PowertrainSolver.SolveSubstep(
                        in veh, in vCfg, in ptCfg, in driver, ref pt,
                        WheelConfigs, WheelInputs, WheelContacts, Wheels, in tireCache, dtSub);
                }
                else
                {
                    WheelSpinFallbackSolver.SolveWheelSpinAndBrakes(
                        in veh, in vCfg, WheelConfigs, WheelInputs, WheelContacts, Wheels, in tireCache, dtSub);
                }

                ChassisDynamicsSolver.Integrate6Dof(
                    ref veh, in vCfg, WheelContacts, Wheels, in tireCache, s == 0, dtSub);
            }

            pose.CurrentPosition = veh.Position;
            pose.CurrentRotation = veh.Rotation;
            RenderPoses[vehIdx] = pose;
            Powertrains[vehIdx] = pt;
            Vehicles[vehIdx] = veh;
        }
    }
}
