# Consolidated removal review — the only candidate list for this refactor

No file/folder was removed. No item is marked DELETE: static scanning cannot establish absence of all indirect/manual/external references, and some retained snapshots still reference older resources. REVIEW is not permission to delete. Source/package exclusions use COLCON_IGNORE, not removal.

| Path | Type | Reason | Referenced by | Risk | Recommendation |
|---|---|---|---|---|---|
| `src/thietkemoi/thanxe_new_description/` | legacy package | Source retained, excluded by COLCON_IGNORE; active responsibilities migrated | Old tools/docs and stale generated install may still refer here | MEDIUM | REVIEW |
| `src/my_ur_control_cpp/` | legacy package | Mixed source retained; active vehicle/perception nodes migrated; UR10-only nodes not enabled | Old standalone commands, docs and generated install | MEDIUM | REVIEW |
| `src/models/` | legacy asset package | Assets copied into vehicle_description/models; old package excluded | Legacy world/launch and old install | MEDIUM | REVIEW |
| `src/vehicle_description/urdf/camera.xacro` | retained legacy copy | Not installed by authoritative description URDF install list; original also retained | No literal reference in five active packages; external usage not proven absent | MEDIUM | REVIEW |
| `src/vehicle_description/urdf/complete_car.urdf` | retained legacy copy | Not installed by authoritative description URDF install list; original also retained | No literal reference in five active packages; external usage not proven absent | MEDIUM | REVIEW |
| `src/vehicle_description/urdf/dynamic_obstacle.urdf` | retained legacy copy | Not installed by authoritative description URDF install list; original also retained | No literal reference in five active packages; external usage not proven absent | MEDIUM | REVIEW |
| `src/vehicle_description/urdf/ros2_control.xacro` | retained legacy copy | Not installed by authoritative description URDF install list; original also retained | src/vehicle_bringup/config/simulation.xacro; src/vehicle_description/urdf/ur10_with_charge.xacro | MEDIUM | REVIEW |
| `src/vehicle_description/urdf/thanxe_new.trans` | retained legacy copy | Not installed by authoritative description URDF install list; original also retained | No literal reference in five active packages; external usage not proven absent | MEDIUM | REVIEW |
| `src/vehicle_description/urdf/ur10_with_charge.xacro` | retained legacy copy | Not installed by authoritative description URDF install list; original also retained | No literal reference in five active packages; external usage not proven absent | MEDIUM | REVIEW |
| `src/vehicle_description/meshes/{base_link*,Wheel*,Revolute*,dausac,cong_sac}.stl` | legacy meshes | 420mm wheels/chassis now use parametric geometry; some legacy snapshots still reference old meshes | Retained complete_car.urdf and other legacy resources | MEDIUM | REVIEW |
| `src/vehicle_description/worlds/ (except vinfast_map.world)` | optional environments | Not selected by default simulation, retained for alternative scenes | World/model internal references and manual usage | MEDIUM | REVIEW |
| `src/vehicle_bringup/maps/` | maps | Only selected YAML/PGM needed by a given Nav2 run; cannot infer other maps obsolete | Nav2/manual map selection | MEDIUM | KEEP |
| `src/Universal_Robots_ROS2_Description/` | vendor package tree | Not used by FR10 vehicle; still used by UR packages | UR driver, UR simulation, legacy UR10 model | HIGH | KEEP |
| `src/Universal_Robots_ROS2_Driver/` | vendor package tree | Independent UR hardware/control/MoveIt stack; not an unused-file conclusion | UR package dependency graph | HIGH | KEEP |
| `src/Universal_Robots_ROS2_Gazebo_Simulation/` | vendor package tree | Independent UR demo retained | ur_simulation_gazebo and UR description | HIGH | KEEP |
| `src/fr10/` | vendor/standalone control tree | FR10 meshes and MoveIt dependencies remain active | vehicle_description; fr10_arm_control; internal FAIRINO dependencies | HIGH | KEEP |
| `src/ccs2_socket_detector/` | ROS interface package | Message type identity preserved to avoid breaking subscribers | charger_detection and existing ROS clients | HIGH | KEEP |
| `build/; install/; log/` | generated artifacts | Not source; legacy installed packages may remain visible; no cleanup performed | ROS environment and ongoing runtime | HIGH | KEEP |
| `/tmp/vinuni_sim_*` | runtime configuration | Retained generated controller YAML; auto-removal disabled for this pass | Running Gazebo plugins may still use these files | MEDIUM | REVIEW |
