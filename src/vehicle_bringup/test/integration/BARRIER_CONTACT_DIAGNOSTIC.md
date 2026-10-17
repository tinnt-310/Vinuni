# Angled-barrier contact diagnostic

The old V6 probe client uses the retired split-controller graph. Use the
current production swerve runner with its opt-in contact probe. It creates a
fresh Gazebo server and a generated launch file; the source world, robot Xacro,
controller configuration, solver, contact surfaces and safety limits are not
edited.

Run after sourcing /opt/ros/humble/setup.bash and install_grouped/setup.bash:

    python3 src/vehicle_bringup/test/integration/run_swerve_case.py \
      docs/ode_debug/p2_barrier_swerve_probe_1 lateral \
      --domain 97 --port 11397 --contact-probe \
      --contact-target drc_practice_angled_barrier_45 \
      --drive-duration 30 --settling-duration 8

The 30 s command duration is only a diagnostic path length: in the retained
failed V6 trace, first angled-barrier contact occurred at chassis y about
2.7 m. It is not an acceptance threshold. A run may return exit 1 because
its ordinary free-space gate requires zero LCP. Keep the artifact and read
barrier_contact_analysis.json and summary.json; do not relabel it PASS.

The artifact contains the generated launch, plugin SHA256 and source SHA256,
runtime.log, monotonic-receipt-stamped runtime_events.jsonl, physics.jsonl
with Gazebo sim time, collision pairs, point depths/normals, body-1 force
and chassis pose/velocity, plus ros_trace.json with published and normalized
commands, wheel state, odometry and clocks. The two
barrier_*_physics_window.jsonl files retain 0.25 s inspection windows. The
analysis marks missing streams and an LCP preceding target contact explicitly.
A nearby physics sample does not identify the exact ODE solver row.

The diagnostic plugin calls Gazebo ContactManager's SetNeverDropContacts(true)
so contacts remain observable. It does not change contact parameters, but
collecting every 1 ms physics sample can perturb wall-time scheduling. For a
comparison, run the same lateral command without --contact-probe on a
different fresh server/domain/port, then compare first LCP and chassis path.
Neither run alone establishes a production fix.
