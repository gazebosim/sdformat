# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from sdformat import Noise
import sdformat as sdf
import math
import unittest

class NoiseTEST(unittest.TestCase):

    def test_construct_and_set(self):
        noise = Noise()

        self.assertEqual(sdf.NoiseType.NONE, noise.type())
        noise.set_type(sdf.NoiseType.GAUSSIAN)
        self.assertEqual(sdf.NoiseType.GAUSSIAN, noise.type())

        self.assertAlmostEqual(0.0, noise.mean())
        noise.set_mean(1.2)
        self.assertAlmostEqual(1.2, noise.mean())

        self.assertAlmostEqual(0.0, noise.std_dev())
        noise.set_std_dev(2.3)
        self.assertAlmostEqual(2.3, noise.std_dev())

        self.assertAlmostEqual(0.0, noise.bias_mean())
        noise.set_bias_mean(4.5)
        self.assertAlmostEqual(4.5, noise.bias_mean())

        self.assertAlmostEqual(0.0, noise.bias_std_dev())
        noise.set_bias_std_dev(6.7)
        self.assertAlmostEqual(6.7, noise.bias_std_dev())

        self.assertAlmostEqual(0.0, noise.precision())
        noise.set_precision(8.9)
        self.assertAlmostEqual(8.9, noise.precision())

        self.assertAlmostEqual(0.0, noise.dynamic_bias_std_dev())
        noise.set_dynamic_bias_std_dev(9.1)
        self.assertAlmostEqual(9.1, noise.dynamic_bias_std_dev())

        self.assertAlmostEqual(0.0, noise.dynamic_bias_correlation_time())
        noise.set_dynamic_bias_correlation_time(19.12)
        self.assertAlmostEqual(19.12, noise.dynamic_bias_correlation_time())

        # Copy Constructor
        noise2 = Noise(noise)
        self.assertEqual(noise, noise2)

        # Copy operator
        noise3 = noise
        self.assertEqual(noise, noise3)

        noise4 = copy.deepcopy(noise)
        self.assertEqual(noise, noise4)


if __name__ == '__main__':
    unittest.main()
