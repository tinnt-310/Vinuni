# Navigation integration validation — 2026-10-14

## Result

The published runtime is `.colcon/releases/rb_v3_humble_20261014T061006Z.9pjc3e`;
`ros2_current` and `install` resolve to it. Sixteen bringup dependency packages
built successfully. Colcon reported **271 tests, 0 errors, 0 failures, 21 skipped**.
The skipped checks belong to the retained FAIRINO/FR10 suite (20 cppcheck entries
and one disabled CTest); they are not newly skipped navigation checks.

Additional regression of the relocated Python helpers: **81 passed** across
controller profiles, traces, ODE readiness, geometry/contact evidence, wrench
calibration, uncertainty, GUI contracts and optional MuJoCo model tests.
Shapely-dependent socket convex-surface tests were not run in this environment.

The end-to-end test ran against the published workspace overlay in isolated
ROS domain 170 and passed in **18.49 seconds**:

1. Start headless Gazebo through the public bringup with `navigation_mode:=mapping`.
2. Receive a nonempty SLAM occupancy map.
3. Save YAML/PGM using `ros2 launch vehicle_bringup save_map.launch.py`.
4. Restart using `navigation_mode:=navigation` and that saved map.
5. Initialize AMCL; verify `map → base_footprint` TF and nine lifecycle nodes active.
6. Send a 0.4 m NavigateToPose goal; require SUCCEEDED and stamped controller commands.
7. Shut down the processes owned by the test.

Evidence: build/test log (local: `src/docs/validation/navigation_20261014/build_test.log`),
mapping (local: `src/docs/validation/navigation_20261014/mapping.log`),
map saving (local: `src/docs/validation/navigation_20261014/save_map.log`),
navigation (local: `src/docs/validation/navigation_20261014/navigation.log`).

## Migration checks

- `colcon list` discovers 18 packages with matching leaf directory/package names.
- No workspace-level `tools` directory remains. The 90 relocated files are listed
  in [file_migration.tsv](file_migration.tsv). Original backups/research profiles
  are retained under `docs/archive`, excluded from package discovery and installation.
- Python syntax, affected XML/YAML, installed launch arguments, model/controller
  contracts, and patch whitespace checks pass.
- Upstream vendor sources, actuator interfaces, frame and joint identifiers remain
  compatible; this change does not commission physical vehicle hardware.

## Validation boundary

Acceptance covers the simulated vehicle, static mapping, saved-map localization
and a short navigation goal. It does not establish long-route performance,
mapping coverage, dynamic obstacle handling, or the swept collision envelope of
an extended FR10 arm. No hardware motion test was run.
