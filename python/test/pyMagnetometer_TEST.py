# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# you may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANy KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from sdformat import Magnetometer, Noise
import sdformat as sdf
import unittest


class MagnetometerTEST(unittest.TestCase):

  def test_default_construction(self):
    mag = Magnetometer()
    defaultNoise = Noise()
    self.assertEqual(defaultNoise, mag.x_noise())
    self.assertEqual(defaultNoise, mag.y_noise())
    self.assertEqual(defaultNoise, mag.z_noise())

  def test_set(self):
    mag = Magnetometer()
    defaultNoise = Noise()
    noise = Noise()
    self.assertEqual(defaultNoise, mag.x_noise())
    self.assertEqual(defaultNoise, mag.y_noise())
    self.assertEqual(defaultNoise, mag.z_noise())

    noise.set_type(sdf.NoiseType.GAUSSIAN)
    noise.set_mean(1.2)
    noise.set_std_dev(2.3)
    noise.set_bias_mean(4.5)
    noise.set_bias_std_dev(6.7)
    noise.set_precision(8.9)

    mag.set_x_noise(noise)
    self.assertEqual(noise, mag.x_noise())
    self.assertEqual(defaultNoise, mag.y_noise())
    self.assertEqual(defaultNoise, mag.z_noise())

    mag.set_y_noise(noise)
    self.assertEqual(noise, mag.x_noise())
    self.assertEqual(noise, mag.y_noise())
    self.assertEqual(defaultNoise, mag.z_noise())

    mag.set_z_noise(noise)
    self.assertEqual(noise, mag.x_noise())
    self.assertEqual(noise, mag.y_noise())
    self.assertEqual(noise, mag.z_noise())

    # Copy Constructor
    mag2 = Magnetometer(mag)
    self.assertEqual(mag, mag2)

    # Copy operator
    mag3 = mag
    self.assertEqual(mag, mag3)

    mag4 = Magnetometer()
    self.assertNotEqual(mag, mag4)


if __name__ == '__main__':
    unittest.main()
