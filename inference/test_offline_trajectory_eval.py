import unittest
import numpy as np
from offline_trajectory_eval import integrate, rotation, state_rotation, angle


class Tests(unittest.TestCase):
    def test_base_translation_and_body_rotation(self):
        r = rotation([.4, -.3, .7])
        state = np.r_[[.5, .2, .6], r[:, :2].T.reshape(-1), 1]
        action = np.array([[.001, .002, -.003, .04, -.02, .03, 0]])
        p, orientations = integrate(state, action)
        np.testing.assert_allclose(p[0], state[:3] + action[0, :3])
        np.testing.assert_allclose(orientations[0], r @ rotation(action[0, 3:6]), atol=1e-14)
        np.testing.assert_allclose(state_rotation(state), r, atol=1e-14)

    def test_known_rotation_and_horizon(self):
        state = np.array([0, 0, 0, 1, 0, 0, 0, 1, 0, 1.], float)
        actions = np.tile([.001, 0, 0, 0, 0, .01, 1], (10, 1))
        p, r = integrate(state, actions)
        self.assertAlmostEqual(p[-1, 0], .01)
        self.assertAlmostEqual(angle(np.eye(3), r[-1]), np.degrees(.1), places=8)


if __name__ == "__main__":
    unittest.main()
