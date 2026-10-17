"""Deterministic tests for contact/LCP timestamp alignment; no Gazebo required."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from barrier_contact_diagnostics import (
    DEFAULT_TARGET, analyze_directory, analyze_trace, inject_probe)


def sample(wall, contact=False):
    contacts = []
    if contact:
        contacts = [{
            'a': 'my_mobile_manipulator::base_link::collision',
            'b': DEFAULT_TARGET + '::one::link::collision',
            'points': [{'position': [0, 2.7, 0.2],
                        'normal': [0, -1, 0], 'depth': 0.003,
                        'body1_force': [3, 4, 0]}],
        }]
    return {'wall': wall, 'sim': wall - 80,
            'models': [{'name': 'my_mobile_manipulator',
                        'position': [0, 2.7, 0], 'rpy': [0, 0, 0],
                        'linear': [0, 0.12, 0], 'angular': [0, 0, 0],
                        'joints': []}],
            'contacts': contacts}


class BarrierContactDiagnosticsTest(unittest.TestCase):
    def test_contact_command_and_lcp_are_distinct_events(self):
        rows = [sample(100), sample(101, contact=True), sample(101.49)]
        events = [{'wall': 101.5, 'line': 'ODE Message: LCP internal error'}]
        commands = {'published_commands': [
            {'wall': 100.9, 'vy': 0.12}, {'wall': 101.1, 'vy': 0.0}],
            'normalized_commands': [
                {'wall': 100.95, 'vy': 0.12}, {'wall': 101.2, 'vy': 0.0}]}
        result = analyze_trace(rows, events, commands)
        self.assertEqual(result['measurement_status'],
                         'TARGET_CONTACT_AND_LCP_OBSERVED')
        self.assertEqual(result['first_target_contact']['collision_pairs'][0]['b'],
                         DEFAULT_TARGET + '::one::link::collision')
        self.assertAlmostEqual(result['first_target_contact']['max_depth_m'], 0.003)
        self.assertAlmostEqual(result['first_target_contact']['max_body1_force_n'], 5)
        self.assertAlmostEqual(result['contact_to_lcp_log_arrival_s'], 0.5)
        self.assertEqual(result['published_command_at_contact']['vy'], 0.12)
        self.assertEqual(result['published_command_at_lcp']['vy'], 0.0)
        self.assertAlmostEqual(result['first_lcp_nearest_physics_wall_offset_s'],
                               -0.01)

    def test_no_target_contact_never_implies_collision_cause(self):
        result = analyze_trace([sample(1), sample(2)],
                               [{'wall': 2, 'line': 'LCP internal error'}], {})
        self.assertEqual(result['measurement_status'],
                         'NO_TARGET_CONTACT_OBSERVED')
        self.assertIsNone(result['first_target_contact'])

    def test_lcp_before_target_contact_does_not_implicate_barrier(self):
        result = analyze_trace(
            [sample(101), sample(102, contact=True)],
            [{'wall': 101.2, 'line': 'LCP internal error'}], {})
        self.assertEqual(result['measurement_status'],
                         'LCP_PRECEDES_TARGET_CONTACT')
        self.assertFalse(result['command_evidence_complete'])

    def test_missing_evidence_is_invalid(self):
        self.assertTrue(analyze_trace([], [], {})['measurement_status']
                        .startswith('INVALID'))

    def test_generated_launch_injects_only_diagnostic_plugin(self):
        source = 'before\n    # Source Xacros retain package://.\nafter'
        generated = inject_probe(source, Path('/tmp/libode_contact_probe.so'))
        self.assertEqual(generated.count('ode_contact_probe'), 2)
        self.assertIn('doc.documentElement.appendChild', generated)
        with self.assertRaises(ValueError):
            inject_probe('unexpected launch', Path('/tmp/libode_contact_probe.so'))

    def test_directory_analysis_streams_and_writes_event_windows(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'physics.jsonl').write_text('\n'.join(
                json.dumps(row) for row in
                [sample(100), sample(101, contact=True), sample(101.49)]) + '\n')
            (root / 'runtime_events.jsonl').write_text(
                json.dumps({'wall': 101.5, 'line': 'LCP internal error'}) + '\n')
            (root / 'ros_trace.json').write_text(json.dumps({
                'normalized_commands': [{'wall': 100.9, 'vy': 0.12}]}))
            result = analyze_directory(root)
            self.assertEqual(result['physics_samples'], 3)
            self.assertEqual(result['measurement_status'],
                             'TARGET_CONTACT_AND_LCP_OBSERVED')
            self.assertTrue((root / 'barrier_first_contact_physics_window.jsonl').exists())
            self.assertTrue((root / 'barrier_first_lcp_physics_window.jsonl').exists())


if __name__ == '__main__':
    unittest.main()
