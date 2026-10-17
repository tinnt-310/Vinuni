# System integration regression suite

This directory owns the former workspace-level regression programs: Gazebo/ODE
observers, vehicle/FR10 scenarios, geometry evidence and optional MuJoCo probes.
They are test utilities, not operational startup commands. Shared helpers stay
colocated because tests exercise the mounted vehicle, arm, contacts and model
together. Python imports between these helpers therefore remain local.

Start normal operation only with `ros2 launch vehicle_bringup bringup.launch.py`.
Source `/opt/ros/humble/setup.bash` and the workspace `install/setup.bash` before
running ROS tests. The regression programs resolve the workspace relative to
this source directory and write evidence beneath `test_results/` or explicit
output paths. Optional MuJoCo/VTK/Shapely tests require their respective Python
dependencies and are not normal launch dependencies. Physical cabinet probes
remain opt-in and must not be run as part of a Gazebo smoke test.

Example, from the workspace root:

```bash
python3 -m pytest src/vehicle_bringup/test/integration/test_fr10_feedback_trace.py
```

Matched-CAD MuJoCo capture trials (fixed-base FR10, isolated ROS domain 212):

Step-by-step 6DOF commissioning, custom gains and live six-panel plots:
[test guide](../../../../docs/fr10_6dof_mujoco_test_guide.md).

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py
```

The default matrix increases the relative error through 0.5, 1, 2, 3 and 5 mm /
degrees along a diagonal in X/Y and Rx/Ry, plus pure X−5 mm, Ry−5°, Z±5 mm
and a spatial (3, 0, 4) mm / (3, −4, 0)° case. The full XYZ translation
and rotation-vector norms are bounded; this is a desired error region, not one fixed
offset or a claim of exhaustive coverage. The physical socket stays fixed.
Each trial sends one mouth pose, measures contact/physical
fit, observes five simulation seconds of HOLD, requests withdrawal and confirms
controlled shutdown. The report preserves failures; SUCCESS alone is insufficient.
The output directory contains `matrix_summary.json`, `force_alignment.png` and
`wrench_components.png` (Fx/Fy/Fz and Tx/Ty/Tz in the fixed insertion frame).
Timed-out trials remain in the success-rate denominator. A missing run summary
is reported separately. These are observed samples, not proof over the whole region.
Use `--cases combined_diagonal_5` for one case, `--domain` for isolation,
or `--report-only --output <existing-directory>` to replot completed trials.

`--mode passive` is the default: one diagonal 6D admittance, no spiral and no
active XY/yaw scans. Contact-derived axis capture and locating pressure remain
available. `--mode search` enables the existing bounded reference search for
comparison after passive testing; it does not introduce another controller.
The older 2 mm / 2° presets remain available for regression.

`--case-group smoke|axes|combined|boundary` selects staged matrices. The axes
group tests ±1 mm and ±1 degree independently on all six DOFs. Boundary tests
both signs at 5 mm / 5 degrees plus combined cases. `--overlay <YAML>` selects
custom gains; its search policy must match `--mode`. `--dry-run` writes
`trial_plan.json` with resolved overlay parameters without starting ROS/physics.

`mujoco_passive_5mm_5deg.yaml` is a simulation commissioning candidate: it
requests 10 mm/s free and 1 mm/s approach at real-time factor 1. Loaded CAD contact
can run slower. The report measures actual wall speed and RTF separately; no
wall-time clock substitution or relaxed geometry/force gates are used.

Initial passive evidence and the controller audit are recorded in
[the commissioning report](../../../../docs/fr10_mujoco_admittance_audit_20261016.md).
In particular, a plug resting on a flat face at 5 mm lateral error may provide
no directional wrench. Axial pose error also changes the controller's depth
datum; increasing correction bounds alone does not establish correct seating.
The 5 mm / 5° profiles are experimental and do not certify this capture region.

While a trial is active, a second sourced terminal can observe its force plot:

```bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/vehicle_bringup/scripts/fr10_force_plot.py
```

View the moving arm and blue socket during the active case (use the directory
printed by the runner, after `selected_station.json` appears):

```bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/vehicle_bringup/test/integration/view_fr10_force_mujoco.py \
  --run-directory <output-directory>/combined_diagonal_5 --live --close-up
```

This window subscribes only to measured joints. It cannot move or stop the arm;
closing it leaves the test running. Old/frozen samples are labelled STALE.
The matched viewer loads original FR10/tool/camera visual CAD, with contact
cells hidden and convex-hull display disabled. Visual meshes are only loaded
in the observer; the dynamics bridge keeps its existing contact model. Large
STL and D435 DAE visuals are converted to indexed OBJ triangles without
decimation. Fixture supports and exact socket faces remain visible.

To watch the robot and physical socket after a trial, replay its measured joints:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/view_fr10_force_mujoco.py \
  --run-directory <output-directory>/combined_diagonal_5 --speed 1
```

The blue socket uses the original contact surfaces. Space pauses; C switches
between the arm and close-up. This is recorded joint
replay in MuJoCo, not a second dynamics test or a live force command. The older
`--trace` viewer remains a separate proxy-fixture visualization.

Operational Gazebo acceptance uses the public bringup and isolated local ROS
domains. Run these separately after sourcing the workspace; they start and stop
their own headless simulations:

```bash
RUN_GAZEBO_PROFILE_TESTS=1 python3 -m pytest -s \
  src/vehicle_bringup/test/integration/test_bringup_profiles_gazebo.py
RUN_GAZEBO_NAVIGATION_TEST=1 python3 -m pytest -s \
  src/vehicle_bringup/test/integration/test_navigation_gazebo.py
```

The profile suite checks startup, active controllers, feedback and TF for the
vehicle, arm, combined robot and force fixtures, plus forward/strafe commands on
the vehicle. It does not execute insertion or certify clean arm shutdown. The
navigation suite checks SLAM, map saving, AMCL-only localization, full Nav2
lifecycle activation and a short NavigateToPose goal. Runtime logs and maps are
written to each pytest temporary directory. Recorded results and remaining
issues are in [the simulation report](../../../docs/command_simulation_validation.md).

The historical pre-RB-V3 `check_vehicle_geometry.py` remains an evidence utility;
current geometry acceptance is in `vehicle_description/test` and
`vehicle_bringup/test/test_vehicle_geometry_contract.py`.
