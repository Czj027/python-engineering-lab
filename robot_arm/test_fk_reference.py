"""Check that the FK benchmark detects a change to the local model alone."""

import unittest
from unittest.mock import patch

import numpy as np
from roboticstoolbox.models.DH import Puma560

from demo import compare_forward
import kinematics


class ForwardReferenceTest(unittest.TestCase):
    def test_benchmark_detects_offset_in_local_dh_table(self):
        reference = Puma560()
        q = np.deg2rad([15, 38, -65, 12, 45, -20])
        nominal = compare_forward(q, reference)
        shifted_dh = kinematics.DH.copy()
        shifted_dh[0, 1] += 1e-3
        with patch.object(kinematics, "DH", shifted_dh):
            shifted = compare_forward(q, reference)
        self.assertLess(nominal["position_error_m"], 1e-6)
        self.assertEqual(nominal["toolbox_xyz_m"], shifted["toolbox_xyz_m"])
        self.assertAlmostEqual(shifted["position_error_m"], 1e-3, delta=1e-12)


if __name__ == "__main__":
    unittest.main()
