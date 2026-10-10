import unittest
import numpy as np
from decentralized_mpc import DT, EPOCH, N, packet, rollout
from ros_decentralized_mpc import braking, decode, encode, prediction_status, select_previous


class ROSPredictionProtocol(unittest.TestCase):
    def setUp(self):
        self.state = np.array([0, 0, .3, 1, .2])
        self.pred = packet(EPOCH, rollout(self.state, np.zeros((N, 2)), tail=True))

    def test_native_message_roundtrip(self):
        msg = encode('car_2', self.pred)
        self.assertEqual(msg.vehicle_id, 'car_2')
        self.assertEqual(msg.header.frame_id, 'map')
        restored = decode(msg)
        np.testing.assert_allclose(restored['times'], self.pred['times'], atol=1e-9)
        np.testing.assert_allclose(restored['states'], self.pred['states'], atol=1e-9)
        self.assertEqual(prediction_status(restored, EPOCH + DT, EPOCH), 'fresh')

    def test_missing_expired_future_and_short_coverage(self):
        self.assertEqual(prediction_status(None, EPOCH, EPOCH), 'missing')
        self.assertEqual(prediction_status(self.pred, EPOCH + 2 * DT, EPOCH), 'expired')
        self.assertEqual(prediction_status(self.pred, EPOCH - 1, EPOCH), 'future')
        self.assertEqual(prediction_status(self.pred, EPOCH + DT, EPOCH + DT), 'missing_round')
        short = packet(EPOCH, self.pred['states'][:4])
        self.assertEqual(prediction_status(short, EPOCH, EPOCH), 'invalid_or_uncovered')
        bad = packet(EPOCH, self.pred['states'])
        bad['states'][2, 0] = np.nan
        self.assertEqual(prediction_status(bad, EPOCH, EPOCH), 'invalid_or_uncovered')

    def test_same_epoch_bootstrap_and_current_round_do_not_replace_previous(self):
        initial = (self.pred, 1.)
        current = (dict(self.pred, sequence=0), 2.)
        inbox = {-1: initial, 0: current}
        self.assertIs(select_previous(inbox, 0), initial)
        self.assertIs(select_previous(inbox, 1), current)
        self.assertEqual(select_previous({0: current}, 0), (None, None))

    def test_braking_preserves_steering_stops_without_reversing(self):
        for speed in (0., .01, .3, 1.):
            state = self.state.copy(); state[3] = speed
            controls = braking(state)
            motion = rollout(state, controls)
            self.assertGreaterEqual(np.min(motion[:, 3]), -1e-10)
            self.assertAlmostEqual(motion[-1, 3], 0.)
            self.assertTrue(np.all(controls[:, 0] <= 0))
            self.assertTrue(np.all(controls[:, 0] >= -1))
            np.testing.assert_allclose(controls[:, 1], state[4])


if __name__ == '__main__':
    unittest.main()
