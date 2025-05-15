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
from sdformat import NavSat, Noise
import unittest

class NavSatColor(unittest.TestCase):

  def test_default_construction(self):
    navSat = NavSat()
    defaultNoise = Noise()
    self.assertEqual(defaultNoise, navSat.vertical_position_noise())
    self.assertEqual(defaultNoise, navSat.horizontal_position_noise())
    self.assertEqual(defaultNoise, navSat.vertical_velocity_noise())
    self.assertEqual(defaultNoise, navSat.horizontal_velocity_noise())

  def test_set(self):
    navSat = NavSat()

    noise = Noise()
    defaultNoise = Noise()

    # set random values and check they apply.
    noise.set_mean(6.5)
    noise.set_std_dev(3.79)

    navSat.set_vertical_position_noise(noise)
    self.assertEqual(noise, navSat.vertical_position_noise())
    self.assertEqual(defaultNoise, navSat.horizontal_position_noise())
    self.assertEqual(defaultNoise, navSat.vertical_velocity_noise())
    self.assertEqual(defaultNoise, navSat.horizontal_velocity_noise())
    navSat.set_horizontal_position_noise(noise)
    self.assertEqual(noise, navSat.vertical_position_noise())
    self.assertEqual(noise, navSat.horizontal_position_noise())
    self.assertEqual(defaultNoise, navSat.vertical_velocity_noise())
    self.assertEqual(defaultNoise, navSat.horizontal_velocity_noise())
    navSat.set_vertical_velocity_noise(noise)
    self.assertEqual(noise, navSat.vertical_position_noise())
    self.assertEqual(noise, navSat.horizontal_position_noise())
    self.assertEqual(noise, navSat.vertical_velocity_noise())
    self.assertEqual(defaultNoise, navSat.horizontal_velocity_noise())
    navSat.set_horizontal_velocity_noise(noise)
    self.assertEqual(noise, navSat.vertical_position_noise())
    self.assertEqual(noise, navSat.horizontal_position_noise())
    self.assertEqual(noise, navSat.vertical_velocity_noise())
    self.assertEqual(noise, navSat.horizontal_velocity_noise())

    # Inequality operator
    navSat2 = NavSat()
    self.assertNotEqual(navSat2, navSat)

    # Copy constructor
    navSatCopied = NavSat(navSat)
    self.assertEqual(navSatCopied, navSat)

    # Assignment operator
    navSatAssigned = navSat
    self.assertEqual(navSatAssigned, navSat)

    # Test deep copy
    navSatCopied = copy.deepcopy(navSatAssigned)
    self.assertEqual(navSatAssigned, navSatCopied)


if __name__ == '__main__':
    unittest.main()
