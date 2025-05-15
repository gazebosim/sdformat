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

import copy
from gz.math import Angle, Helpers
from sdformat import Lidar, Error, Noise
import math
import unittest


class LidarTEST(unittest.TestCase):

  def test_default_construction(self):
    lidar = Lidar()
    defaultNoise = Noise()
    self.assertEqual(defaultNoise, lidar.lidar_noise())

  def test_set(self):

    lidar = Lidar()

    lidar.set_horizontal_scan_samples(123)
    self.assertEqual(lidar.horizontal_scan_samples(), 123)
    lidar.set_horizontal_scan_resolution(0.45)
    self.assertAlmostEqual(lidar.horizontal_scan_resolution(), 0.45)
    lidar.set_horizontal_scan_min_angle(Angle(0.67))
    self.assertAlmostEqual(lidar.horizontal_scan_min_angle().radian(), 0.67)
    lidar.set_horizontal_scan_max_angle(Angle(0.89))
    self.assertAlmostEqual(lidar.horizontal_scan_max_angle().radian(), 0.89)

    lidar.set_vertical_scan_samples(98)
    self.assertEqual(lidar.vertical_scan_samples(), 98)
    lidar.set_vertical_scan_resolution(0.76)
    self.assertAlmostEqual(lidar.vertical_scan_resolution(), 0.76)
    lidar.set_vertical_scan_min_angle(Angle(0.54))
    self.assertAlmostEqual(lidar.vertical_scan_min_angle().radian(), 0.54)
    lidar.set_vertical_scan_max_angle(Angle(0.321))
    self.assertAlmostEqual(lidar.vertical_scan_max_angle().radian(), 0.321)

    lidar.set_range_min(1.2)
    self.assertAlmostEqual(lidar.range_min(), 1.2)
    lidar.set_range_max(3.4)
    self.assertAlmostEqual(lidar.range_max(), 3.4)
    lidar.set_range_resolution(5.6)
    self.assertAlmostEqual(lidar.range_resolution(), 5.6)

    noise = Noise()
    noise.set_mean(6.5)
    noise.set_std_dev(3.79)
    lidar.set_lidar_noise(noise)
    self.assertEqual(noise, lidar.lidar_noise())

    lidar.set_horizontal_scan_samples(111)
    lidar.set_horizontal_scan_resolution(2.2)

    self.assertEqual(Helpers.MAX_UI32, lidar.visibility_mask());
    lidar.set_visibility_mask(123);
    self.assertEqual(123, lidar.visibility_mask());

    # Inequality operator
    lidar2 = Lidar()
    self.assertNotEqual(lidar2, lidar)

    # Copy constructor
    lidarCopied = Lidar(lidar)
    self.assertEqual(lidarCopied, lidar)

    # Assignment operator
    lidarAssigned = lidar
    self.assertEqual(lidarAssigned, lidar)


if __name__ == '__main__':
    unittest.main()
