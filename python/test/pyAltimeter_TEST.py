# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from sdformat import Altimeter, Noise
import sdformat as sdf
import unittest


class AtmosphereTEST(unittest.TestCase):

    def test_default_construction(self):
        alt = Altimeter()
        defaultNoise = Noise()
        self.assertTrue(defaultNoise, alt.vertical_position_noise())
        self.assertTrue(defaultNoise, alt.vertical_velocity_noise())


    def test_set(self):
        alt = Altimeter()
        defaultNoise = Noise()
        noise = Noise()
        self.assertTrue(defaultNoise, alt.vertical_position_noise())
        self.assertTrue(defaultNoise, alt.vertical_velocity_noise())

        noise.set_type(sdf.NoiseType.GAUSSIAN)
        noise.set_mean(1.2)
        noise.set_std_dev(2.3)
        noise.set_bias_mean(4.5)
        noise.set_bias_std_dev(6.7)
        noise.set_precision(8.9)

        alt.set_vertical_position_noise(noise)
        self.assertTrue(noise, alt.vertical_position_noise())
        self.assertTrue(defaultNoise, alt.vertical_velocity_noise())

        alt.set_vertical_velocity_noise(noise)
        self.assertTrue(noise, alt.vertical_position_noise())
        self.assertTrue(noise, alt.vertical_velocity_noise())

        # Copy Constructor
        alt2 = Altimeter(alt)
        self.assertTrue(alt, alt2)

        # Copy operator
        alt3 = alt
        self.assertTrue(alt, alt3)

        alt6 = Altimeter()
        self.assertNotEqual(alt3, alt6)

        # set position noise but velocity noise should still be different
        alt6.set_vertical_position_noise(alt3.vertical_position_noise());
        self.assertNotEqual(alt3, alt6);

if __name__ == '__main__':
    unittest.main()
