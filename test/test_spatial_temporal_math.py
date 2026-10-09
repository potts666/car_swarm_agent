import unittest
import numpy as np
from optimize_spatial_temporal import coefficients, evaluate, body_distance


class SpatialTemporalMath(unittest.TestCase):
    def test_physical_time_derivatives_and_c2_continuity(self):
        states = np.array([[0, 0, 0, 0, 0.3, 0], [1, 0.2, 0.5, 0.1, -0.1, 0.2],
                           [2, 1, 0, 0, -0.3, 0]])
        c = coefficients(states, 3)
        for derivative in range(3):
            actual = evaluate(c, 3, np.array([0., 3., 6.]), derivative)
            np.testing.assert_allclose(actual, states[:, derivative * 2:derivative * 2 + 2], atol=1e-12)
            left = evaluate(c[:1], 3, np.array([3.]), derivative)
            right = evaluate(c[1:], 3, np.array([0.]), derivative)
            np.testing.assert_allclose(left, right, atol=1e-12)

    def test_body_distance_uses_body_offset_margin_and_rotation(self):
        p = np.array([[0., 0.], [0., 0.], [0., 0.]])
        peer = np.array([[6., 0.], [0., 3.], [0., 0.]])
        distance = body_distance(p, np.zeros(3), peer, np.array([0., 0., np.pi / 2]))
        np.testing.assert_allclose(distance, [1.3, 1., 0.], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
