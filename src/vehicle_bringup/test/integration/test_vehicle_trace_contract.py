import unittest

from vehicle_trace_contract import evaluate_vehicle_ros_trace


def valid_trace():
    return {
        'wheel_states': [
            {'wall': 1.5, 'source': 10.0, 'speed': [1.0]*4, 'steer': [0.0]*4},
            {'wall': 3.5, 'source': 12.0, 'speed': [0.0]*4, 'steer': [0.0]*4}],
        'odom': [
            {'wall': 1.5, 'source': 10.0, 'x': 0.0, 'y': 0.0, 'vx': .1, 'vy': 0.0},
            {'wall': 3.5, 'source': 12.0, 'x': .2, 'y': 0.0, 'vx': 0.0, 'vy': 0.0}],
        'tf': [
            {'wall': 1.5, 'source': 10.0, 'x': 0.0, 'y': 0.0},
            {'wall': 3.5, 'source': 12.0, 'x': .2, 'y': 0.0}],
        'normalized_commands': [
            {'wall': 1.5, 'source': 10.0, 'vx': .1, 'vy': 0.0, 'omega': 0.0},
            {'wall': 3.1, 'source': 10.0, 'vx': 0.0, 'vy': 0.0, 'omega': 0.0}],
    }


class VehicleTraceContractTest(unittest.TestCase):
    def test_valid_motion_and_settling_trace(self):
        self.assertTrue(evaluate_vehicle_ros_trace(valid_trace(), 1, 3)['valid'])

    def test_rejects_stale_odom_republished_with_fresh_receive_time(self):
        trace = valid_trace()
        trace['odom'][1]['source'] = 10.0
        self.assertFalse(evaluate_vehicle_ros_trace(trace, 1, 3)['valid'])

    def test_rejects_missing_tf_settling(self):
        trace = valid_trace()
        trace['tf'].pop()
        self.assertFalse(evaluate_vehicle_ros_trace(trace, 1, 3)['valid'])

    def test_rejects_nonfinite_wheel_state(self):
        trace = valid_trace()
        trace['wheel_states'][0]['speed'][0] = float('nan')
        self.assertFalse(evaluate_vehicle_ros_trace(trace, 1, 3)['valid'])

    def test_allows_same_stamp_zero_stop_after_drive(self):
        self.assertTrue(evaluate_vehicle_ros_trace(valid_trace(), 1, 3)['valid'])

    def test_rejects_missing_final_stop(self):
        trace = valid_trace()
        trace['normalized_commands'].pop()
        self.assertFalse(evaluate_vehicle_ros_trace(trace, 1, 3)['valid'])

    def test_rejects_missing_normalized_motion_command(self):
        trace = valid_trace()
        trace['normalized_commands'][0]['wall'] = 0.5
        self.assertFalse(evaluate_vehicle_ros_trace(trace, 1, 3)['valid'])


if __name__ == '__main__':
    unittest.main()
