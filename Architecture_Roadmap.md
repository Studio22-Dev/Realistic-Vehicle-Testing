# High-Realism Unity Burst / DOTS-Ready Vehicle Simulation — Project Blueprint (v2)

## 1. Overall Goal & Core Engineering Constraints
Build a high-realism, 60 FPS vehicle physics simulation supporting ~24 simultaneous vehicles with tiered sub-stepping:
- **Player Vehicle (LOD0):** ~500Hz+ (default `9` sub-steps per 60Hz `FixedUpdate` = 540Hz), positioned at index `0` (`batchSize = 1`) to eliminate worker-thread tail latency.
- **AI Vehicles (LOD1/LOD2):** ~120Hz (`2` sub-steps per 60Hz `FixedUpdate`).
- **Batched Physics Queries:** Run once per `FixedUpdate` via `RaycastCommand.ScheduleBatch` (expanding to volumetric multi-probe fans in Phase 2) with unmanaged `colliderInstanceID` lookup tables (`NativeHashMap`).
- **DOTS Migration Readiness:**
  - All core physics data lives in unmanaged, blittable C# structs (`[StructLayout(LayoutKind.Sequential)]`, 16-byte SIMD aligned, 64-byte cache-line padded) inside `NativeArray`s.
  - Core solver assembly (`VehiclePhysics.Core`) enforces `"noEngineReferences": true` so jobs migrate directly into Unity Entities (`ISystem` / `IComponentData` / `DynamicBuffer<T>` / `BlobAssetReference<T>`) without rewriting solver math.
  - Strictly `0 B` per-frame GC allocations in `FixedUpdate` and `Update`.

---

## 2. Phase 1 Status (Completed & Verified)
### Assembly Layout
- `VehiclePhysics.Core` (`noEngineReferences: true`): Unmanaged structs, `VehicleSimulationWorld`, `SteeringAndInputSolver`, `SuspensionSolver`, `TireSolver`, `PowertrainSolver`, `ChassisDynamicsSolver`, `VehicleSimulationJob`, `VehicleChassisCollisionJob`, `AiTrackDriverJob`, `VehicleLodSchedulerJob`.
- `VehiclePhysics.Bridge`: `VehicleRaycastPipeline`, `ColliderPhysicsMeta` (resolving Unity `Collider` properties into unmanaged Burst-safe `NativeHashMap<int, ColliderPhysicsMeta>`).
- `VehiclePhysics.Runtime`: `VehicleTransformSyncJobs` (`[BurstCompile] IJobParallelForTransform` over `TransformAccessArray`), `VehicleSimulationManager`.
- `VehiclePhysics.Tests`: EditMode layout/byte-alignment tests, `PowertrainSolverTests`, `MultiVehicleStep5Tests`.

### Phase 1 Core Structs & Physics
- **`VehicleState` (128B):** `Rotation`, `Position`, `InverseMass`, `LinearVelocity`, `ForwardSpeedMs`, `AngularVelocity`, `WheelStartIndex`, `InverseInertiaLocal`, `WheelCount`, `ExternalImpulse`, `VehicleId`, `ExternalAngularImpulse`, `ConfigIndex`, `LocalAcceleration`, `SubstepCount`, `TierFlags`, `LodTier` (byte 127).
- **`PowertrainState` (64B):** Full 1D rotational inertia (`Engine -> Flywheel -> Clutch -> Gearbox -> Differential -> Wheels`), closed-form reflected inertia `I_drive_eff^-1`, implicit tire-spin inertia stabilization `I_w_eff`, and 2-iteration PGS Coulomb solver for Clutch Lockup, Axle LSDs, and ABS/Handbrake hold.
- **`WheelState` (80B):** Hub-to-chassis `SuspensionCompression`/`SuspensionVelocity`, `AngularVelocity`, `SpinAngle`, transient carcass relaxation `(u_x, u_y)`, `WarmStartBrakeImpulse`, `LastWorldForce`.
- **Phase 1 Audit Fixes to Apply at Start of Phase 2:**
  1. Clamp *accumulated* PGS impulses (`J_accum = clamp(J_old + dJ, -J_max, +J_max); dJ_apply = J_accum - J_old`) rather than only per-iteration deltas.
  2. Keep kinematic PGS clutch-to-wheel impulse transfer exact ($\eta_d = 1$ in $\Delta\omega_{w,i}$) and model driveline efficiency loss $(1 - \eta_d)$ as an explicit opposing drag torque.
  3. Keep `WheelState.SuspensionCompression` strictly as hub-to-chassis travel (pure suspension spring $k_s$) so Phase 2 tire compliance sits cleanly in series.
  4. Guard rolling resistance at $\omega_w \approx 0$ to prevent parked creep.

---

## 3. Core Architecture Direction: Universal Alt 3 Articulated Multi-Body + Option 0 Tire Carcass
**Every vehicle** will converge on an **Alt 3 Articulated Multi-Body Chassis/Suspension/Driveline Constraint Architecture** paired initially with **Option 0 (Implicit Spring-Damper Carcass + 4-Cluster `ContactManifold`)**, executed in two cleanly staged sub-phases (Phase 2A & Phase 2B) before Phase 3 full damage mechanics:

### Phase 2A — Volumetric Tire, Terrain, Rim & Thermal Upgrade (Option 0 + Hard Rim/Stop Clamps)
Upgrade the tire and ground interaction pipeline first while running atop the Phase 1 strut model:
1. **`ContactManifold` (384B) & `ManifoldContactPoint` (80B):**
   - Replaces single `RaycastHit` / `WheelContact`.
   - Clusters LOD-scaled volumetric probe fans into 4 directional contact zones (`C0` Bottom, `C1` Leading Climb, `C2` Trailing, `C3` Sidewall/Rim) for realistic rock/curb climbing and side-angle impacts.
   - Tracks `WaterDepthM`, `TerrainSinkageM`, `HydroplaningFactor`, `BulldozingDragN`, and per-cluster `ColliderInstanceID` + `PendingObjectImpulse` (drained safely post-step to push dynamic objects).
2. **`TireCarcassState` (128B):**
   - Unconditional Backward-Euler 3-DOF+twist carcass flex (`CarcassDeflectionLocal`, `CarcassVelocityLocal`, `TorsionalTwistRad`) connecting the **Wheel Rim** independently to the **Contact Footprint**.
   - Dynamic `PneumaticTrailM` & `SelfAligningTorqueNm` for realistic steering FFB.
   - `InternalPressureKPa`, `LeakRateKPaPerSec`, `CarcassIntegrity`, `BeadSeatingIntegrity`, `BlowoutProgress` (punctures, blowouts, de-beading).
   - Separate **Wheel Rim** sub-state (`RimRadiusM`, `RimGroundLoadN`, `RimLocalImpactOffset`, `RimBentAngleRad`, `RimBentPhaseRad`, `RimRadialDentDepthM`, `RimStructuralHealth`) with velocity-level hard-stop clamping on rim bottom-outs.
3. **`TireThermalAndWearState` (128B):**
   - 3-zone `(Inner, Middle, Outer)` `TreadSurfaceTempC`, `TreadSubsurfaceTempC`, `CarcassBulkTempC`, `CoreGasTempC` (coupled via Ideal Gas Law) + per-tire-type compound curves (`Street`, `SemiSlick`, `Slick`, `AllTerrain`, `MudTerrain`, `Winter`, `Drag`).
   - 3-zone `RemainingTreadDepthMm`, `MudAndDebrisClogging`, thermal flat-spotting, and **16-bin persistent circumferential abrasion flat-spotting** (`FlatSpotDepthBins0..3`).
4. **Off-Road & Hydroplaning:** Bekker-Wong sinkage, plastic compaction, bulldozing drag, Mohr-Coulomb shear, and Horne hydrodynamic wedge lift.

### Phase 2B — Universal Alt 3 Articulated Multi-Body Constraints & Component Graph (All Vehicles)
Upgrade **every vehicle** from 1D raycast struts to a **per-vehicle, cache-local Alt 3 Articulated Multi-Body Constraint Solver + Kinematic/Spatial Component Graph** (without spawning separate colliding rigid bodies for every mechanical part):
1. **Local Articulated Multi-Body Solver (Chassis + Unsprung Wheel Hubs / Solid Axles):**
   - Solves true coupled **Chassis (6-DOF) $\leftrightarrow$ Wheel Hub / Upright (or Solid Axle)** dynamics via analytical 3D suspension linkage Jacobians (Double Wishbone, MacPherson Strut, Trailing Arm, Multi-Link, Solid Axle + Panhard/Leaf) and PGS/TGS velocity constraints.
   - Delivers real camber curves, caster trail, scrub radius, bump-steer, anti-dive/anti-squat jacking forces, hub wheel-hop mass, and zero-penetration bump-stop/steering-rack limits on every vehicle.
   - **LOD Scaling:**
     - **LOD0/LOD1:** Full coupled hub bodies + analytical linkage Jacobians + hard constraint stops.
     - **LOD2:** Reduced 1-DOF projection evaluated from the same linkage curve tables so distant AI cars remain physically consistent at low CPU cost.
2. **Explicit Non-Simulated Physical Component Topology (Every Vehicle):**
   Track lightweight unmanaged state, 3D chassis-relative bounding/mount geometry, thermal load, bend offsets, and visual transforms for:
   - **Wheel Hub / Upright (`WheelHubState`):** Bearing friction, wheel-stud/hub flange integrity, bend angle, bearing overheat seizure.
   - **Brake Assembly (`BrakeComponentState`):** Rotor & caliper temperatures (conductive/radiative heat transfer to hub & tire core gas), pad wear, fluid fade, rotor warp thickness variation (pulsing brake torque with `SpinAngle`), line leak pressure drop.
   - **Suspension Components (`SuspensionLinkageState`):** Upper/lower control arms, strut/damper assembly (oil leak/blown valving), coil/leaf spring sag/snap, steering tie-rod length/bend deltas, anti-roll bar link coupling.
   - **Physical Driveshaft & Half-Shafts (`DrivelineShaftState`):** Rotating visual transforms (`DriveshaftOmega`, `HalfShaft` CV articulation angles tracking hub compression/steer), torsional compliance/windup, U-joint/CV wear, imbalance wobble force when bent (`2*omega` vibration), and torque-overload shear threshold.
   - **Physical Differential (`DifferentialState`):** Housing spatial collision vulnerability (e.g., rock crawl pumpkin strike), gear-oil temperature, LSD clutch-pack wear, open/locked/welded/seized failure modes.
   - **Physical Gearbox & Clutch (`GearboxComponentState`):** Casing underbody hit exposure, gear-mesh temperature, per-gear synchro/dog-ring wear (`DamagedGearMask`), clutch thermal glazing/fade, engine/transmission mount deflection.

---

## 4. Per-Vehicle Tire-Shell Expansions (Layered Over Universal Alt 3 Chassis)
While **all vehicles share the Alt 3 articulated chassis/hub/driveline architecture**, individual vehicle archetypes can selectively swap the **LOD0 tire carcass layer** on top of the wheel rim:
- **Standard / Mixed Fleet (Default):** **Option 0** (Implicit 3-DOF+twist Spring-Damper Carcass + 4-cluster `ContactManifold`). Best balance for 24-car mixed traffic, street, and general off-road.
- **Motorsport / GT / Formula / Tarmac Rally (Alt 1 Tire Shell):** **6-DOF Rigid-Ring + Brush Enveloping** around the Alt 3 wheel hub for ~60–100 Hz belt eigenmodes, smooth QSA/tandem-cam kerb enveloping, and hyper-nuanced FFB road grain.
- **Extreme Rock Crawlers / Monster & Mud Trucks (Alt 2 Tire Shell):** **16–24-Node XPBD Pressurized Soft-Body Ring** around the Alt 3 wheel rim for true geometric wrapping over jagged boulders, pinched sidewall bulging, and physical deflated rubber folding at low crawl PSI.

---

## 5. Phase 3 Roadmap (Full Mechanical Damage & Permanent Deformation)
Phase 3 activates crash/overload thresholds across the Phase 2A/2B component graph:
- **Permanent Wheel Rim Deformation:** Curb/pothole impacts bend `RimBentAngleRad`, dent `RimRadialDentDepthM`, offset `RimLocalImpactOffset`, induce persistent wheel wobble + steering shimmy, and cause chronic bead-seal air leaks even on fresh rubber until repaired/replaced.
- **Bent & Broken Suspension/Steering:** High-G side impacts permanently offset Alt 3 pickup anchors (`LowerArmBend`, `TieRodBend`, `StrutBend`) causing pulled steering, skewed camber/toe, binding suspension travel, and blown damper seals.
- **Driveline & Brake Failures:** Underbody rock/crash hits dent/crack `Gearbox`, `Differential`, or `Driveshaft` (inducing imbalance vibration, gear-pop-out, oil starvation seizure, or complete torque disconnect); extreme braking warps rotors or bursts brake lines.
- **Detachable Components & Debris:** Exceeding ultimate shear detaches a wheel/hub assembly, snapped driveshaft, or body panel—updating `VehicleState.InverseMass` / `InverseInertiaLocal` dynamically and spawning a standalone debris rigidbody only upon detachment.
