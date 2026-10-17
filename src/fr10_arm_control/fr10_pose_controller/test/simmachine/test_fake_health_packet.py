"""The FakeSystem-only heartbeat must satisfy the backend health contract."""

import unittest

from test_fake_runtime import fake_health_packet


class FakeHealthPacketTest(unittest.TestCase):
    def test_enabled_healthy_packet_has_exact_fields(self):
        data = fake_health_packet(23)
        self.assertEqual(len(data), 16)
        self.assertEqual(data[0], 1.0)
        self.assertEqual(data[1], 23.0)
        self.assertEqual(data[6], 1.0)
        self.assertTrue(all(value == 0.0 for index, value in enumerate(data)
                            if index not in (0, 1, 6)))

    def test_rejects_nonpositive_sequence(self):
        with self.assertRaises(ValueError):
            fake_health_packet(0)


if __name__ == '__main__':
    unittest.main()
