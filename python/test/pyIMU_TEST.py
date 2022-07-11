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
from gz.math import Vector3d
from sdformat import IMU, Noise
import sdformat as sdf
import unittest

class IMUTest(unittest.TestCase):

  def test_default_construction(self):
    imu = IMU()
    defaultNoise = Noise()
    noise = Noise()

    noise.set_type(sdf.NoiseType.GAUSSIAN)
    noise.set_mean(1.2)
    noise.set_std_dev(2.3)
    noise.set_bias_mean(4.5)
    noise.set_bias_std_dev(6.7)
    noise.set_precision(8.9)

    self.assertEqual(defaultNoise, imu.linear_acceleration_x_noise())
    imu.set_linear_acceleration_x_noise(noise)
    self.assertEqual(noise, imu.linear_acceleration_x_noise())

    self.assertEqual(defaultNoise, imu.linear_acceleration_y_noise())
    imu.set_linear_acceleration_y_noise(noise)
    self.assertEqual(noise, imu.linear_acceleration_y_noise())

    self.assertEqual(defaultNoise, imu.linear_acceleration_z_noise())
    imu.set_linear_acceleration_z_noise(noise)
    self.assertEqual(noise, imu.linear_acceleration_z_noise())

    self.assertEqual(defaultNoise, imu.angular_velocity_x_noise())
    imu.set_angular_velocity_x_noise(noise)
    self.assertEqual(noise, imu.angular_velocity_x_noise())

    self.assertEqual(defaultNoise, imu.angular_velocity_y_noise())
    imu.set_angular_velocity_y_noise(noise)
    self.assertEqual(noise, imu.angular_velocity_y_noise())

    self.assertEqual(defaultNoise, imu.angular_velocity_z_noise())
    imu.set_angular_velocity_z_noise(noise)
    self.assertEqual(noise, imu.angular_velocity_z_noise())

    self.assertEqual(Vector3d.UNIT_X, imu.gravity_dir_x())
    imu.set_gravity_dir_x(Vector3d.ZERO)
    self.assertEqual(Vector3d.ZERO, imu.gravity_dir_x())

    self.assertFalse(imu.gravity_dir_x_parent_frame())
    imu.set_gravity_dir_x_parent_frame("my_frame")
    self.assertEqual("my_frame", imu.gravity_dir_x_parent_frame())

    self.assertEqual(Vector3d.ZERO, imu.custom_rpy())
    imu.set_custom_rpy(Vector3d.UNIT_Z)
    self.assertEqual(Vector3d.UNIT_Z, imu.custom_rpy())

    self.assertFalse(imu.custom_rpy_parent_frame())
    imu.set_custom_rpy_parent_frame("other_frame")
    self.assertEqual("other_frame", imu.custom_rpy_parent_frame())

    self.assertEqual("CUSTOM", imu.localization())
    imu.set_localization("NED")
    self.assertEqual("NED", imu.localization())

    self.assertTrue(imu.orientation_enabled())
    imu.set_orientation_enabled(False)
    self.assertFalse(imu.orientation_enabled())

    # Copy Constructor
    imu2 = IMU(imu)
    self.assertEqual(imu, imu2)

    # Assignment
    imu = imu2
    self.assertEqual(imu2, imu)

    # Copy assignment
    imu = copy.deepcopy(imu2)
    self.assertEqual(imu2, imu)

    # inequality
    imu6 = IMU()
    self.assertNotEqual(imu2, imu6)


if __name__ == '__main__':
    unittest.main()
