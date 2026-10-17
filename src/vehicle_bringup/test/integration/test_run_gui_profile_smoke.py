import importlib.util
import struct
from pathlib import Path


MODULE_PATH = Path(__file__).with_name('run_gui_profile_smoke.py')
SPEC = importlib.util.spec_from_file_location('run_gui_profile_smoke', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_gui_window_selection_uses_wm_class_not_title_substring():
    inventory = '''
     0x5a00010 "Gazebo": ("gazebo" "gazebo")  1920x1016+0+37
     0x360003e "Hướng dẫn cài Gazebo — Mozilla Firefox": ("Navigator" "firefox")  1920x1053+0+27
     0x1c00106 "simulation_vehicle.rviz - RViz": ("rviz2" "rviz2")  1600x900+10+45
    '''

    windows = MODULE.parse_window_inventory(inventory)

    assert MODULE.select_largest_window(windows, 'gazebo')[1] == 'Gazebo'
    assert MODULE.select_largest_window(windows, 'rviz2')[1].endswith('RViz')
    assert MODULE.select_largest_window(
        windows, 'gazebo', excluded_ids={0x5A00010}) is None


def test_top_level_window_walk_stops_at_child_of_root():
    parents = {0x40: 0x30, 0x30: 0x20, 0x20: 0x01}

    assert MODULE.top_level_window_id(
        0x40, root_id=0x01, parent_lookup=parents.__getitem__) == 0x20


def test_decode_xwd_image_reads_little_endian_truecolor_pixels():
    header = struct.pack(
        '>25I',
        100, 7, 2, 24, 2, 1, 0, 0, 32, 0, 32, 32, 8, 4,
        0x00FF0000, 0x0000FF00, 0x000000FF, 8, 256, 0,
        2, 1, 0, 0, 0,
    )
    raw = header + struct.pack('<2I', 0x00112233, 0x00A0B0C0)

    image = MODULE.decode_xwd_image(raw)

    assert image.size == (2, 1)
    assert list(image.getdata()) == [(0x11, 0x22, 0x33),
                                     (0xA0, 0xB0, 0xC0)]


def test_decode_xwd_image_reads_24_bit_pixels_with_row_padding():
    header = struct.pack(
        '>25I',
        100, 7, 2, 24, 2, 1, 0, 0, 32, 0, 32, 24, 8, 4,
        0x00FF0000, 0x0000FF00, 0x000000FF, 8, 256, 0,
        2, 1, 0, 0, 0,
    )
    raw = header + bytes((
        0x33, 0x22, 0x11,
        0xC0, 0xB0, 0xA0,
        0x00, 0x00,
    ))

    image = MODULE.decode_xwd_image(raw)

    assert image.size == (2, 1)
    assert list(image.getdata()) == [(0x11, 0x22, 0x33),
                                     (0xA0, 0xB0, 0xC0)]
