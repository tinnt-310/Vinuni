"""Independent physical TCP and keyed socket fit checks for commissioning runs."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from fr10_model_evidence import fixed_link_transform


def stationary_base_window(rows, now_wall, pose_window_s=.016):
    """Qualify fixture startup before sending a goal, never waive parking.

    Match the plant's 200 ms source dwell /50 ms freshness /1e-4 motion
    limits. Use the same physical-pose velocity window as BasePoseVelocity:
    Gazebo P3D instantaneous twist contains physics-substep impulses even when
    measured body pose is stationary. Wheel odometry cannot qualify this gate.
    """
    if not rows or not math.isfinite(now_wall) or not math.isfinite(pose_window_s) or not .001<=pose_window_s<=.020:
        return False
    last=rows[-1]
    if not 0. <= now_wall-last['wall'] <= .05:
        return False
    begin=len(rows)-1
    while begin>0 and last['source']-rows[begin]['source']<.2+pose_window_s-1e-12:
        begin-=1
    window=rows[begin:]
    if last['source']-window[0]['source']<.2+pose_window_s-1e-12:
        return False
    for row in window:
        values=[row['wall'],row['source'],*row['position'],*row['orientation_xyzw'],*row['twist']]
        if not all(math.isfinite(v) for v in values) or row['source']<=0. or \
                row['frame']!='world' or row['child']!='base_footprint' or \
                abs(np.linalg.norm(row['orientation_xyzw'])-1.)>1e-6:
            return False
    for first,second in zip(window,window[1:]):
        if not 0.<second['source']-first['source']<=.020 or \
                not 0.<second['wall']-first['wall']<=.05:
            return False
    sources=np.asarray([r['source'] for r in window])
    current=np.flatnonzero(sources-sources[0]>=pose_window_s-1e-12)
    if not len(current) or sources[-1]-sources[current[0]]<.2-1e-12:
        return False
    positions=np.asarray([r['position'] for r in window])
    rotations=Rotation.from_quat([r['orientation_xyzw'] for r in window])
    targets=np.maximum(sources[0],sources[current]-pose_window_s)
    old_positions=np.column_stack([np.interp(targets,sources,positions[:,i]) for i in range(3)])
    old_rotations=Slerp(sources,rotations)(targets)
    body_from_world=rotations[current].inv()
    linear=body_from_world.apply((positions[current]-old_positions)/pose_window_s)
    angular=body_from_world.apply((rotations[current]*old_rotations.inv()).as_rotvec()/pose_window_s)
    return bool(np.max(np.abs(linear))<=1e-4 and np.max(np.abs(angular))<=1e-4)


def self_alignment_result(summary):
    """Input-error acceptance requires the independent complete-run verdict."""
    uncertainty = summary.get('pose_uncertainty', {})
    translation = uncertainty.get('translation_mm', uncertainty.get('lateral_mm', []))
    nonzero = any(value != 0. for value in [*translation, *uncertainty.get('rotation_vector_deg', [])])
    if not nonzero:
        return {'force_self_alignment_gate': 'NOT_TESTED'}
    physical_geometry_unchanged = uncertainty.get('physics_geometry_modified') is False
    passed = (summary.get('pass') is True and physical_geometry_unchanged and
              summary.get('physical_seating_gate') == 'PASS')
    status = 'PASS' if passed else 'FAIL'
    if summary.get('backend') == 'fairino':
        status = 'VIRTUAL_CAD_' + status
    return {'force_self_alignment_gate': status,
            'self_alignment_acceptance_basis':
            'independent CAD fit at native measured joints; real FT/contact unverified'
            if summary.get('backend') == 'fairino' else
            'unchanged CAD, independent physical TCP and all requested motion/contact/HOLD/fault/withdrawal gates'}


def hold_duration_complete(wall_start, wall_now, source_start, source_now, minimum_s=5.):
    """Require elapsed measured source time as well as wall observation time."""
    values=(wall_start,wall_now,source_start,source_now,minimum_s)
    if not all(math.isfinite(v) for v in values) or minimum_s<=0:
        raise ValueError('Invalid HOLD observation clock')
    if wall_now<wall_start or source_now<source_start:
        raise ValueError('Backward HOLD observation clock')
    return wall_now-wall_start>=minimum_s and source_now-source_start>=minimum_s


def hold_feedback_boundary(feedback, result_wall):
    """Result and feedback use separate DDS streams; await an observed HOLD."""
    for index, row in enumerate(feedback):
        if row['wall'] < result_wall:
            continue
        if row['state'] == 9:
            return index
        # The appended compliant phases (13..16) precede SUCCESS/HOLD;
        # enum numeric order is not state-machine transition order.
        if row['state'] in (10, 11, 12) or row['state'] not in range(17):
            raise ValueError('Force session failed or left HOLD before observation')
    return None


def physical_tcp_trace(output,backend,mujoco_trace):
    if backend in ('mujoco','fairino'): return mujoco_trace
    path=output/'transport_state.jsonl'
    if not path.is_file() or not (output/'model.urdf').is_file(): return []
    robot=ET.parse(output/'model.urdf').getroot()
    tool=fixed_link_transform(robot,
                              'charging_connector_link','charging_tcp')
    result=[];mounts={}
    # A live observer may be partway through its final append. Consume only
    # complete records; malformed newline-terminated data must still fail.
    text=path.read_text()
    lines=text.splitlines()
    if text and not text.endswith('\n'): lines=lines[:-1]
    for line in lines:
        sample=json.loads(line);body=sample['models'][0]
        rotation=Rotation.from_quat(sample['orientation_xyzw']).as_matrix()
        point=np.asarray(body['position'])+rotation@tool[:3,3]
        q=Rotation.from_matrix(rotation@tool[:3,:3]).as_quat()
        row={'source':sample['sim'],'wall':sample['wall'],
             'position':point.tolist(),'orientation_xyzw':q.tolist()}
        if 'reference_link' in sample:
            base=sample['reference_link']
            reference=base['name'].split('::')[-1]
            if reference not in mounts:
                mounts[reference]=fixed_link_transform(robot,reference,'fr10_base_link')
            mount=mounts[reference]
            reference_rotation=Rotation.from_quat(base['orientation_xyzw']).as_matrix()
            row['base']={'name':'fr10_base_link',
                'position':(np.asarray(base['position'])+reference_rotation@mount[:3,3]).tolist(),
                'orientation_xyzw':Rotation.from_matrix(reference_rotation@mount[:3,:3]).as_quat().tolist()}
        result.append(row)
    return result


def evaluate_physical_fit(trace,station,geometry,result_wall,end_wall=None):
    recent=[s for s in trace if result_wall-.1<=s['wall']<= (end_wall if end_wall is not None else float('inf'))]
    if len(recent)<3: return {'physical_seating_gate':'INSUFFICIENT_PHYSICAL_TCP_SAMPLES'}
    mouth=np.asarray(station['world_mouth_position_m'])
    rotation=Rotation.from_euler('xyz',station['world_mouth_rpy']).as_matrix()
    depth=[];lateral=[];angular=[];feature_error=[]
    clearance=geometry['simulation_design']['radial_clearance_mm']*.001
    lever=.001*max(np.hypot(np.hypot(*p['center_mm'])+p['bore_inscribed_radius_mm'],
                           geometry['simulation_design']['nose_length_mm']) for p in geometry['pins'])
    for sample in recent:
        local=rotation.T@(np.asarray(sample['position'])-mouth)
        angle=Rotation.from_matrix(rotation.T@Rotation.from_quat(sample['orientation_xyzw']).as_matrix()).magnitude()
        depth.append(float(local[2]));lateral.append(float(np.linalg.norm(local[:2])));angular.append(float(angle))
        feature_error.append(lateral[-1]+2*lever*np.sin(angle/2))
    passed=(max(abs(d-station['insertion_depth_m']) for d in depth)<=.0005 and
            max(feature_error)<=clearance)
    hold_origin=np.asarray(recent[0]['position'])
    physical_drift=max(float(np.linalg.norm(np.asarray(s['position'])-hold_origin)) for s in recent)
    return {'physical_seating_gate':'PASS' if passed else 'FAIL',
            'physical_depth_range_m':[min(depth),max(depth)],
            'physical_max_lateral_error_m':max(lateral),
            'physical_max_angular_error_rad':max(angular),
            'physical_max_mating_feature_error_m':max(feature_error),
            'physical_hold_drift_m':physical_drift,
            'physical_mating_clearance_m':clearance,'physical_seated_samples':len(recent)}


def evaluate_fk_consistency(physical,reported):
    if not physical or not reported: return {'physical_fk_consistency_gate':'NO_SAMPLES'}
    times=np.asarray([s['source'] for s in reported]);distances=[];angles=[]
    for sample in physical:
        index=int(np.argmin(np.abs(times-sample['source'])))
        if abs(times[index]-sample['source'])>.055 or 'base' not in sample: continue
        base=sample['base'];rotation=Rotation.from_quat(base['orientation_xyzw']).as_matrix()
        actual=rotation.T@(np.asarray(sample['position'])-base['position'])
        orientation=Rotation.from_matrix(rotation.T@Rotation.from_quat(sample['orientation_xyzw']).as_matrix())
        distances.append(float(np.linalg.norm(actual-np.asarray(reported[index]['position']))))
        angles.append(float((orientation.inv()*Rotation.from_quat(reported[index]['orientation_xyzw'])).magnitude()))
    if not distances: return {'physical_fk_consistency_gate':'NO_SYNCHRONIZED_SAMPLES'}
    # Moving-arm nearest-source matching may include <=55 ms of travel.
    return {'physical_fk_comparison_samples':len(distances),
            'physical_fk_max_position_error_m':max(distances),
            'physical_fk_position_error_p95_m':float(np.percentile(distances,95)),
            'physical_fk_max_angular_error_rad':max(angles)}


def evaluate_retract(trace,station,result_wall):
    recent=[s for s in trace if s['wall']>=result_wall]
    if len(recent)<3: return {'physical_retract_gate':'INSUFFICIENT_PHYSICAL_TCP_SAMPLES'}
    mouth=np.asarray(station['world_mouth_position_m'])
    rotation=Rotation.from_euler('xyz',station['world_mouth_rpy']).as_matrix()
    local=np.asarray([rotation.T@(np.asarray(s['position'])-mouth) for s in recent])
    target=-station['preinsert_distance_m']
    depth_error=float(np.max(np.abs(local[:,2]-target)))
    lateral=float(np.max(np.linalg.norm(local[:,:2],axis=1)))
    angular=max(Rotation.from_matrix(rotation.T@Rotation.from_quat(s['orientation_xyzw']).as_matrix()).magnitude()
                for s in recent)
    return {'physical_retract_gate':'PASS' if depth_error<=.0005 and lateral<=.0005 and angular<=.005 else 'FAIL',
        'retract_depth_error_m':depth_error,'retract_lateral_error_m':lateral,
        'retract_angular_error_rad':float(angular),'physical_retracted_samples':len(recent)}
