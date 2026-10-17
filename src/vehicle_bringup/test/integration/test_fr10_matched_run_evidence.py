import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from fr10_model_evidence import fixed_link_transform
from fr10_matched_run_evidence import evaluate_physical_fit,evaluate_fk_consistency


def base_window():
    return [dict(wall=100.+i*.01,source=10.+i*.01,frame='world',child='base_footprint',
                 position=[0.,0.,0.],orientation_xyzw=[0.,0.,0.,1.],twist=[0.]*6)
            for i in range(24)]


def test_startup_waits_for_physical_stationarity_not_zero_wheel_odometry():
    from fr10_matched_run_evidence import stationary_base_window
    rows=base_window()
    assert stationary_base_window(rows,100.24)
    for row in rows:
        row['position'][2]=(row['source']-10.)*.002
    assert not stationary_base_window(rows,100.24)


def test_startup_requires_source_time_and_fresh_continuous_physical_feedback():
    from fr10_matched_run_evidence import stationary_base_window
    assert not stationary_base_window([],100.24)
    assert not stationary_base_window(base_window()[:10],100.24)
    assert not stationary_base_window(base_window(),101.)
    rows=base_window()
    for row in rows:row['source']=10.
    assert not stationary_base_window(rows,100.24)
    rows=base_window();rows[15]['wall']+=.2
    assert not stationary_base_window(rows,100.24)


def test_startup_rejects_motion_rotation_invalid_pose_and_wrong_frame():
    from fr10_matched_run_evidence import stationary_base_window
    for key,value in [('position',[float('nan'),0.,0.]),
                      ('orientation_xyzw',[0.,0.,0.,0.]),('frame','odom')]:
        rows=base_window();rows[-1][key]=value
        assert not stationary_base_window(rows,100.24)
    rows=base_window()
    for row in rows:
        row['orientation_xyzw']=Rotation.from_rotvec([0.,.002*(row['source']-10.),0.]).as_quat().tolist()
    assert not stationary_base_window(rows,100.24)


def test_startup_uses_physical_pose_windows_through_p3d_substep_twist():
    from fr10_matched_run_evidence import stationary_base_window
    rows=base_window()
    for row in rows:row['twist']=[0.,0.,-.0796,0.,.0094,0.]
    assert stationary_base_window(rows,100.24)
    for i,row in enumerate(rows):row['position'][2]=5e-6*(-1 if i%3 else 1)
    assert not stationary_base_window(rows,100.24)


def test_manifest_inventories_active_isolated_prefix_and_colocated_sdk(tmp_path):
    from fr10_matched_run_manifest import runtime_binary_inventory
    pose = tmp_path/'isolated/fr10_pose_controller'
    node = pose/'lib/fr10_pose_controller/fr10_pose_controller_node'
    node.parent.mkdir(parents=True);node.write_bytes(b'isolated node')
    driver = tmp_path/'isolated/fairino_hardware_v3_9_3'
    sdk = driver/'lib/fairino_hardware_v3_9_3/libfairino.so.2.3.3'
    sdk.parent.mkdir(parents=True);sdk.write_bytes(b'colocated sdk')
    prefixes,binaries = runtime_binary_inventory(dict(fr10_pose_controller=str(pose),
        fairino_hardware_v3_9_3=str(driver),unrelated_package='/opt/ros/humble'))
    assert set(binaries) == {node,sdk}
    assert set(prefixes) == {'fr10_pose_controller','fairino_hardware_v3_9_3'}


def test_hold_requires_actual_source_duration_when_physics_is_slow_or_frozen():
    from fr10_matched_run_evidence import hold_duration_complete
    import pytest
    assert not hold_duration_complete(100.,110.,20.,21.)
    assert not hold_duration_complete(100.,200.,20.,20.)
    assert not hold_duration_complete(100.,104.,20.,25.)
    assert hold_duration_complete(100.,150.,20.,25.)
    for end in [19.,float('nan'),float('inf')]:
        with pytest.raises(ValueError):hold_duration_complete(100.,150.,20.,end)


def test_hold_waits_for_feedback_transition_after_independently_delivered_result():
    from fr10_matched_run_evidence import hold_feedback_boundary
    import pytest
    feedback=[dict(wall=99.,state=7)]+[dict(wall=100.001+i*.001,state=state)
                                    for i,state in enumerate([16,13,14,15,7,8,8])]
    assert hold_feedback_boundary(feedback,100.) is None
    feedback.append(dict(wall=100.03,state=9))
    assert hold_feedback_boundary(feedback,100.) == len(feedback)-1
    for state in [10,11,12,99]:
        with pytest.raises(ValueError):hold_feedback_boundary([dict(wall=100.01,state=state)],100.)


def test_fixed_transform_composes_origins_in_urdf_order():
    robot=ET.fromstring('''<robot><joint type="fixed"><parent link="base"/><child link="sensor"/>
      <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>
      <joint type="fixed"><parent link="sensor"/><child link="tcp"/>
      <origin xyz="1 0 0" rpy="0 1.5707963267948966 0"/></joint></robot>''')
    tool=fixed_link_transform(robot,'base','tcp')
    assert np.allclose(tool[:3,3],[1,1,0])
    assert np.allclose(tool[:3,:3],Rotation.from_euler('z',np.pi/2).as_matrix()@
                       Rotation.from_euler('y',np.pi/2).as_matrix())


def test_physical_base_recovers_gazebo_lumped_mount_from_canonical_urdf(tmp_path):
    import json
    from fr10_matched_run_evidence import physical_tcp_trace
    (tmp_path/'model.urdf').write_text('''<robot>
      <joint type="fixed"><parent link="base_link"/><child link="fr10_base_link"/>
        <origin xyz="-.2 0 .44" rpy="0 0 1.5707963267948966"/></joint>
      <joint type="fixed"><parent link="charging_connector_link"/><child link="charging_tcp"/>
        <origin xyz="0 0 .039"/></joint></robot>''')
    turn=Rotation.from_euler('xyz',[.02,-.01,.3])
    sample={'sim':1.,'wall':2.,'models':[{'position':[4,5,6]}],
            'orientation_xyzw':[0,0,0,1],
            'reference_link':{'name':'robot::base_link','position':[1,2,3],
                              'orientation_xyzw':turn.as_quat().tolist()}}
    # A trailing live append cannot corrupt completed independent measurements.
    (tmp_path/'transport_state.jsonl').write_text(json.dumps(sample)+'\n{"partial":')
    row=physical_tcp_trace(tmp_path,'gazebo',[])[0]
    assert np.allclose(row['position'],[4,5,6.039])
    assert np.allclose(row['base']['position'],np.array([1,2,3])+turn.apply([-.2,0,.44]))
    expected=turn*Rotation.from_euler('z',np.pi/2)
    assert (expected.inv()*Rotation.from_quat(row['base']['orientation_xyzw'])).magnitude()<1e-12


def fixture():
    station={'world_mouth_position_m':[0,0,0],'world_mouth_rpy':[0,0,0],'insertion_depth_m':.015}
    geometry={'simulation_design':{'radial_clearance_mm':.25,'nose_length_mm':15},
              'pins':[{'center_mm':[10,0],'bore_inscribed_radius_mm':2}]}
    trace=[{'wall':1+i*.1,'source':1+i*.1,'position':[0,0,.015],
            'orientation_xyzw':[0,0,0,1]} for i in range(5)]
    return trace,station,geometry


def test_physical_fit_rejects_nominal_success_with_wrong_actual_pose():
    trace,station,geometry=fixture()
    assert evaluate_physical_fit(trace,station,geometry,1)['physical_seating_gate']=='PASS'
    for s in trace:s['position'][0]=.001
    assert evaluate_physical_fit(trace,station,geometry,1)['physical_seating_gate']=='FAIL'
    for s in trace:s['position'][0]=0;s['orientation_xyzw']=Rotation.from_euler('x',.1).as_quat().tolist()
    assert evaluate_physical_fit(trace,station,geometry,1)['physical_seating_gate']=='FAIL'


def test_self_alignment_requires_input_error_and_independent_full_acceptance():
    from fr10_matched_run_evidence import self_alignment_result
    summary = dict(backend='mujoco', physical_seating_gate='PASS',
                   pose_uncertainty=dict(lateral_mm=[2.,0.], rotation_vector_deg=[0.,0.,0.],
                                         physics_geometry_modified=False))
    summary['pass'] = True
    assert self_alignment_result(summary)['force_self_alignment_gate'] == 'PASS'
    for key, value in [('physical_seating_gate','FAIL'), ('pass', False)]:
        rejected = dict(summary); rejected[key] = value
        assert self_alignment_result(rejected)['force_self_alignment_gate'] == 'FAIL'
    moved = dict(summary, pose_uncertainty=dict(summary['pose_uncertainty'], physics_geometry_modified=True))
    assert self_alignment_result(moved)['force_self_alignment_gate'] == 'FAIL'
    assert self_alignment_result(dict(summary, backend='fairino'))['force_self_alignment_gate'] == 'VIRTUAL_CAD_PASS'
    assert self_alignment_result(dict(summary, pose_uncertainty={}))['force_self_alignment_gate'] == 'NOT_TESTED'
    axial = dict(summary, pose_uncertainty=dict(translation_mm=[0.,0.,5.],
        rotation_vector_deg=[0.,0.,0.], physics_geometry_modified=False))
    assert self_alignment_result(axial)['force_self_alignment_gate'] == 'PASS'
    assert self_alignment_result(dict(axial, **{'pass': False}))['force_self_alignment_gate'] == 'FAIL'


def test_fk_comparison_removes_actual_mounted_base_transform():
    trace,_,_=fixture();reported=[]
    base=Rotation.from_euler('z',.5)
    for s in trace:
        reported.append(dict(s))
        s['base']={'position':[1,2,3],'orientation_xyzw':base.as_quat().tolist()}
        s['position']=(np.array([1,2,3])+base.apply(s['position'])).tolist()
        s['orientation_xyzw']=base.as_quat().tolist()
    assert evaluate_fk_consistency(trace,reported)['physical_fk_max_position_error_m']<1e-12


def test_seated_evidence_window_excludes_subsequent_retract_and_verifies_withdrawal():
    from fr10_matched_run_evidence import evaluate_retract
    trace,station,geometry=fixture();station['preinsert_distance_m']=.03
    retreat=[dict(wall=2+i*.1,source=2+i*.1,position=[0,0,-.03],orientation_xyzw=[0,0,0,1]) for i in range(5)]
    all_samples=trace+retreat
    assert evaluate_physical_fit(all_samples,station,geometry,1,1.5)['physical_seating_gate']=='PASS'
    assert evaluate_physical_fit(all_samples,station,geometry,1)['physical_seating_gate']=='FAIL'
    assert evaluate_retract(all_samples,station,2)['physical_retract_gate']=='PASS'
    retreat[-1]['position']=[0,0,0]
    assert evaluate_retract(all_samples,station,2)['physical_retract_gate']=='FAIL'
