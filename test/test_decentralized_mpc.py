import unittest
import numpy as np
from decentralized_mpc import DT, N, EPOCH, packet, receive, rollout, solve, neighbor_gap, MARGIN


class DecentralizedMPC(unittest.TestCase):
    def test_absolute_timestamp_alignment_and_missing_coverage_rejection(self):
        states = np.array([[0, 0, 0, 1, 0], [1, 0, 0, 1, 0], [2, 0, 0, 1, 0.]])
        prediction = packet(105., states)
        aligned = receive(prediction, np.array([105.05, 105.15]))
        np.testing.assert_allclose(aligned[:, 0], [.5, 1.5])
        for times in ([104.9], [105.21]):
            with self.assertRaises(ValueError):
                receive(prediction, np.array(times))
        prediction['stamp'] = 100.
        with self.assertRaises(ValueError):
            receive(prediction, np.array([105.1]))

    def test_neighbor_prediction_changes_acceleration_and_keeps_body_clearance(self):
        state = np.array([0, 0, 0, 1, 0.])
        warm = np.zeros((N, 2))
        ref = np.zeros(2, dtype=[(k, float) for k in ('t', 'x', 'y', 'yaw', 'speed')])
        ref['t'] = [0, 20]; ref['x'] = [0, 20]; ref['speed'] = 1
        obstacles = np.array([[20, 20, 1, 1.]])
        close = np.tile([6, 0, 0, 0, 0.], (N * 4 + 5, 1))
        far = close.copy(); far[:, 1] = 10
        u_close, report = solve(state, EPOCH, packet(EPOCH, close), ref, obstacles, warm)
        u_far, far_report = solve(state, EPOCH, packet(EPOCH, far), ref, obstacles, warm)
        self.assertTrue(report['accepted'], report)
        self.assertEqual(report['attempts'][-1]['initialization'], 'braking')
        self.assertTrue(far_report['accepted'], far_report)
        self.assertLess(u_close[0, 0], u_far[0, 0] - .05)
        future = rollout(state, u_close)[1:]
        self.assertGreaterEqual(np.min(neighbor_gap(future, close[1:len(future)+1])), MARGIN - 1e-5)

    def test_prediction_tail_covers_next_control_cycle(self):
        state = np.array([0, 0, 0, 1, 0.])
        prediction = packet(EPOCH, rollout(state, np.zeros((N, 2)), tail=True))
        future = receive(prediction, EPOCH + DT + np.array([.1, N * DT]))
        np.testing.assert_allclose(future[:, 0], DT + np.array([.1, N * DT]))


if __name__ == '__main__':
    unittest.main()
