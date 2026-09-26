# Unified simulation model

Both simulation branches now use vehicle_bringup/simulation.launch.py. enable_vehicle=false loads vehicle_description/arm_only.xacro, which includes the same fr10_arm.urdf and fr10_wrist.xacro as complete_visual.xacro. It anchors only the standalone arm to world, omits vehicle controllers/stations/base sensors, and uses the same vehicle_geometry.yaml arm_home and arm interfaces/controller YAML. RViz fixed frame is fr10_base_link for standalone mode.

Build and route tests pass. Live ROS_DOMAIN_ID=92 check found plate_frame, charging_connector_link, camera_link and the actual jlr_ccs2_centered mesh, with no wheel or roof links. However LCP errors occurred and measured joint positions did not match Ready targets. This is model-source synchronization only, not a dynamics acceptance. Real-only vendor demo remains separate and has not been migrated here.
