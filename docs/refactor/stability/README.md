# Verification correction: LCP unresolved

The earlier log /tmp/solver-live.log contains 5403 LCP internal errors. Prior motion/action passes do NOT establish numerical stability. Further quick-solver, effort+inertia, and softened-contact trials failed and were reverted. The prior configuration remains; no accepted LCP fix exists yet.

# Gazebo stability verification

Changed vehicle_description/urdf/thanxe_new.gazebo base damping from 5.0 to 0.05. This change alone did not prevent tipping. Changed vinfast_map.world from implicit quick solver to ODE world solver, CFM 1e-5, ERP 0.2, contact correcting velocity 0.1 m/s, surface layer 0.001 m. Step remains 0.001 s. Chassis mass remains 150 kg; no world fixed joint added; arm kinematics and tool mounts unchanged.

Isolated domain 89, port 11389: four vehicle motions passed with arm enabled; wheel vectors, odometry vs Gazebo, command timeout and D455 tested. FR10 action moving j1 to 0.15 and j6 to 0.3 succeeded. Final Gazebo pose: 0.459194 1.56663 -0.001286 -0.000290 0.000211 1.47524. Wrist TF and camera checks recorded separately. This validates the tested poses, not every possible extended-arm trajectory. Direct solver can cost more CPU. Vendor inertias remain unverified; uniform-box replacement was previously rejected by experiment.

Use a new terminal and source install_grouped/setup.bash, then ros2 launch vehicle_bringup bringup.launch.py arm:=true vehicle:=true mode:=sim. Restart the old Gazebo process to load changed world physics.
