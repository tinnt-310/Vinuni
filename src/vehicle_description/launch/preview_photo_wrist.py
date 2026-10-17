"""Render or open the photo-reference wrist; FK only, no ROS/actuation."""
from pathlib import Path
import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


def build_model(directory):
    import mujoco
    import xacro
    sys.path.insert(0, str(ROOT/'src/vehicle_bringup/test/integration'))
    from fr10_mujoco_visuals import retain_urdf_visuals
    robot = ET.fromstring(xacro.process_file(str(ROOT/'src/vehicle_description/urdf/vehicle.urdf.xacro'), mappings={
        'enable_vehicle': 'false', 'enable_arm': 'true', 'enable_arm_camera': 'false',
        'wrist_layout': 'photo_dc_d455', 'control_backend': 'none'}).toxml())
    for link in robot.findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            collision.set('name', f'collision__{link.get("name")}__{index}')
    retain_urdf_visuals(robot, directory)
    from ament_index_python.packages import get_package_share_directory
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            mesh.set('filename', str(Path(get_package_share_directory(package))/relative))
    path = Path(directory)/'photo.urdf';ET.ElementTree(robot).write(path, encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(path)), robot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'test_results/fr10_photo_dc_d455')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    if not args.show: os.environ.setdefault('MUJOCO_GL', 'egl')
    import mujoco
    import numpy as np
    with tempfile.TemporaryDirectory(prefix='fr10_photo_preview_') as directory:
        model, robot = build_model(directory);data = mujoco.MjData(model)
        data.qpos[:] = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        mujoco.mj_forward(model, data)
        model.vis.headlight.ambient[:] = .55;model.vis.headlight.diffuse[:] = .6
        # Original visuals only. No dynamics or contact-force claims.
        option = mujoco.MjvOption();option.geomgroup[0] = 0;option.geomgroup[1] = 1
        option.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        wrist = model.body('wrist3_link').id
        turn = data.xmat[wrist].reshape(3, 3)
        flange = data.xpos[wrist] + turn @ [0, 0, .106]
        turn = turn @ np.diag([-1., -1., 1.])
        if args.show:
            import mujoco.viewer,time
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer.opt.geomgroup[:] = option.geomgroup
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
                viewer.cam.lookat[:] = flange + turn @ [0., .06, .08]
                viewer.cam.distance = .6
                while viewer.is_running(): viewer.sync();time.sleep(.03)
            return
        from PIL import Image
        args.output.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, height=480, width=640)
        for label, offset, up in (
                ('front', [0, .065, .53], [0, 1, 0]),
                ('side', [.53, .065, .085], [0, 1, 0]),
                ('perspective', [.35, .23, .46], [0, 1, 0])):
            camera = mujoco.MjvCamera();camera.lookat[:] = flange;camera.distance = .7
            renderer.update_scene(data, camera=camera, scene_option=option)
            position = flange + turn @ offset
            target = flange + turn @ [0, .065, .08]
            forward = target-position;forward /= np.linalg.norm(forward)
            for eye in renderer.scene.camera:
                eye.pos[:] = position;eye.forward[:] = forward;eye.up[:] = turn @ up
            Image.fromarray(renderer.render()).save(args.output/(label+'.png'))
        renderer.close()
        print(f'Photo-layout previews: {args.output}')


if __name__ == '__main__': main()
