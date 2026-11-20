# MDH100 CAD v11 Gazebo integration — 2026-11-18

## Sources and scope

Full assembly: `output/chassis_cad_v11_mdh100_laptop_d435/assembly_layout.json` and referenced Inventor parts. Overlay: Nov18 four-hole axle plates, eight axle plate bolts/nuts and caster-to-rail fasteners. Source CAD is read-only. The bolt visuals retain the existing generic button-head envelope, not a confirmed RLB manufacturer's solid.

`export_cad_v11_meshes.py` recursively exports supplier subassemblies with native transforms. `build_cad_v11_robot.py` generates colored meshes, inertias and fixed-structure collision boxes. `prepare_cad_v11_ros.py` generates the separate launch, Xacro and navigation profile. The exporter requires local Autodesk Inventor and pywin32; ROS build/execution requires WSL Ubuntu 22.04 / ROS Humble / Gazebo Classic.

## Model

| Property | Applied value |
|---|---|
| Drive wheels | 2 × MDH100, radius 65mm, modeled contact width 39mm |
| Tire center track | 407.5mm |
| Drive axis height | 65.00307mm in source CAD |
| Casters | 4 × GL65, radius 32mm, tire contact width 23mm, trail 22mm |
| Caster swivel axes | X ±165mm, Y ±185mm, Z 81mm (fixed-plate underside) |
| Joint states | 2 powered + 4 passive swivel + 4 passive rolling joints |
| Empty simulated mass | 20.848601kg; includes laptop 2.38kg, battery 1.2kg, motors 3kg each |
| Torque limit | 3.2Nm per wheel; constant cap, not motor current/thermal model |
| Friction assumptions | drive 0.8 longitudinal/lateral; caster 0.6 |
| Nav2 footprint | X -0.225 to +0.365m, Y ±0.245m, includes caster sweep |
| Speed/acceleration limits | 0.30m/s, 0.85rad/s; 0.35m/s², 0.8rad/s² |

MDH100 mass sources disagree (retained manufacturer manual says 2kg; prior vendor-page review says 3kg); this simulation uses 3kg conservatively. The rotating/fixed motor mass split is 2.8/0.2kg, an inertia estimate rather than a manufacturer measurement. GL65 total mass is assumed 0.30kg each. Fork/plate separation is clipped at the CAD plate underside; their inertia uses a bounding-box estimate. Other parts use CAD volume distribution with stated component masses/densities in `config/cad_v11_mass_properties.json`.

Axle plates, caster adapters and shims use solid-polymer density 1250kg/m³ for the planned printed fit-test parts. They remain rigid in Gazebo: print infill, elasticity, fatigue and load capacity are not tested by this model. Six nominal coplanar tire contacts do not establish real load sharing.

The latest full CAD retains the deck and laptop; deck procurement remains on hold. Battery and electrical block shapes are CAD space reservations. The LiDAR visual is still the retained A1 CAD envelope, although the purchase selection is A2M12. Scan origin X267.4mm/Z309.5mm and 12m maximum range are simulation assumptions, not measured installation calibration. D435 has its manufacturer visual and a nominal camera/optical frame; no camera image plugin or person tracking is implemented here.

## Verification

`output/cad_v11_sim_validation/static_validation.json` checks joint topology, positive physical inertias, mesh existence, mass sum, caster floor geometry and footprint coverage. URDF also converts to SDF successfully.

Isolated Gazebo domain 91 / port11991, no payload, flat floor:

| Stage | Command | Measured response during command |
|---|---|---|
| Forward, 2.5 simulated seconds | 0.20m/s | 0.442m |
| Reverse, 2.5 simulated seconds | -0.20m/s | 0.438m |
| Turn left, 2.5 simulated seconds | 0.50rad/s | +0.945rad |
| Turn right, 2.5 simulated seconds | -0.50rad/s | -0.947rad |

Maximum chassis tilt across this test: 0.041 degrees. The command ramps account for less-than-step displacement; subsequent stop stages include deceleration travel. 720-ray scan and ten joint states received. Raw results and launch log are alongside the static report. This isolated test covers basic motion. The subsequent live corridor test is documented below; payload performance remains untested.

Live GUI session: `v11_nav_20261118_115307.log`. Nav2 managed nodes activated. After applying `/unpause_physics`, clock and live transforms were received; a single target at odom (0.7,0), yaw0 returned action status4 (SUCCEEDED). The configured XY goal tolerance is 0.25m, so success does not mean exact 0.700m displacement. No obstacle or corner course was claimed by this short test.

Odometry uses Gazebo world ground truth as in the previous simulation. The MDH100 simulator navigation profile initializes AMCL at the known spawn origin. Neither assumption should be copied to the physical robot. Hardware still needs encoder odometry, calibrated TF and appropriate localization initialization.

## Running

From the Windows project folder as mounted in WSL:

```bash
bash scripts/prepare_current_sim.sh v11
bash scripts/start_current_sim.sh nav v11
```

Modes `view`, `slam`, `nav`; only one live Gazebo session at a time. Logs: `output/sim_sessions`. The native Linux package lives in `~/robot_sim_v11_ws`, avoiding Windows/Korean path parsing issues. Use the same UDP DDS profile for additional CLI/test nodes if discovery is inconsistent.

Repeat isolated regression, after stopping the live session:

```bash
bash scripts/test_cad_v11_wsl.sh
```

Original v9 workspace and artifacts remain available for comparison.


## Corridor tuning — 2026-11-18

The selected profile keeps DWB inside RotationShim, the original conservative rectangular footprint and 30mm padding. Cruise speed is 0.30m/s (previously 0.25); angular cap is 0.85rad/s (previously 0.70); shim rotation target is 0.65rad/s (previously 0.50); angular acceleration/deceleration magnitude is 0.8rad/s² (previously 0.6). Linear acceleration remains 0.35m/s². Velocity smoother, behavior server and simulator motion guard agree on these limits.

`config/mdh100_navigation_tuning.yaml` is merged by `scripts/prepare_cad_v11_ros.py`, so regenerating the simulator does not discard the selected tuning. Regeneration was checked against the selected complete YAML and left URDF/launch geometry unchanged. Runtime parameter readback: `output/cad_v11_corridor_tuning/final_readback.json`.

The same four-corner route is north → east → south → west → north. Setup motion is excluded from the time comparison. Raw results and the final summary are in `output/cad_v11_corridor_tuning/`; see `docs/cad_v11_corridor_tuning_20261118.md` for measurements and limitations.

The original east obstacle leaves 1.10m free width, despite its legacy model name containing “80cm”. A separate verified 0.80m fixture was tested. Some outward/return runs succeeded, but repeated narrow return runs failed or were stopped by the clearance monitor. RPP, larger obstacle cost, smaller padding, and tighter goal tolerances did not establish reliable 80cm round trips; those trial configurations are NOT selected. The final profile restores 30mm padding and the original DWB obstacle weighting. Narrow-passage reverse-direction maneuvers remain a separate planning/clearance problem, not a validated capability.

Two early narrow-fixture trials had a pose mismatch and are explicitly excluded from performance comparisons. Subsequent fixture reports verify both Gazebo pose and collision dimensions. The test watchdog cancels at stale odometry/timeout and pauses simulated physics for insufficient geometric clearance or excessive tilt. These are simulation test guards, not a hardware safety system.
