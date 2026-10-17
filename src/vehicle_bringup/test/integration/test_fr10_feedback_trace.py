import math
import unittest

from fr10_feedback_trace import summarize_feedback_sources


class FeedbackTraceTest(unittest.TestCase):
    def test_matches_joint_source_with_float_conversion_error(self):
        result = summarize_feedback_sources(
            [(1.1, 10.0), (1.2, 10.008), (1.3, 10.016)],
            [(10.0080001, None, None, 1.21, 2.0)])
        self.assertTrue(result['tcp_source_matches_joint'])
        self.assertEqual(result['tcp_source_samples'], 1)

    def test_rejects_publication_time_restamp(self):
        result = summarize_feedback_sources(
            [(1.1, 10.0), (1.2, 10.008)],
            [(10.1, None, None, 1.21, 2.0)])
        self.assertFalse(result['tcp_source_matches_joint'])

    def test_rejects_missing_or_replayed_source(self):
        self.assertFalse(summarize_feedback_sources([], [])['tcp_source_matches_joint'])
        result = summarize_feedback_sources(
            [(1.1, 10.0), (1.2, 10.008)],
            [(10.0, None, None, 1.21, 2.0),
             (10.0, None, None, 1.22, 2.1)])
        self.assertFalse(result['tcp_source_monotonic'])


if __name__ == '__main__':
    unittest.main()
