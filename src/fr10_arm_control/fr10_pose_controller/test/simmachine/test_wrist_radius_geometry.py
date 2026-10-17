#!/usr/bin/env python3
"""Keep wrist-radius measurement tied to the live TCP-to-wrist transform."""

import math
import unittest

from test_cartesian_long_distance import wrist_radius


class WristRadiusGeometryTest(unittest.TestCase):
    def test_identity_tcp_orientation_uses_measured_fixed_translation(self):
        # The fixed TF translation is wrist3 in TCP coordinates. A stale
        # hardcoded tool offset would place this wrist at a different radius.
        self.assertAlmostEqual(
            wrist_radius((0.05, 0.20, 0.50), (0.0, 0.0, 0.0, 1.0),
                         (0.013, -0.021, -0.157)),
            math.hypot(0.063, 0.179), places=9)

    def test_rotated_tcp_orientation_rotates_fixed_translation(self):
        half = math.sqrt(0.5)
        self.assertAlmostEqual(
            wrist_radius((0.05, 0.20, 0.50), (0.0, 0.0, half, half),
                         (0.013, -0.021, -0.157)),
            math.hypot(0.071, 0.213), places=9)


if __name__ == "__main__":
    unittest.main()
