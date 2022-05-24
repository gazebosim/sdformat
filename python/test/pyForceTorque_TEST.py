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
from sdformat import ForceTorque, Noise
import sdformat as sdf
import unittest

class ForceTorqueTEST(unittest.TestCase):

  def test_default_construction(self):
    ft = ForceTorque()
    defaultNoise = Noise()
    noise = Noise()

    noise.set_type(sdf.NoiseType.GAUSSIAN)
    noise.set_mean(1.2)
    noise.set_std_dev(2.3)
    noise.set_bias_mean(4.5)
    noise.set_bias_std_dev(6.7)
    noise.set_precision(8.9)

    self.assertEqual(defaultNoise, ft.force_x_noise())
    ft.set_force_x_noise(noise)
    self.assertEqual(noise, ft.force_x_noise())

    self.assertEqual(defaultNoise, ft.force_y_noise())
    ft.set_force_y_noise(noise)
    self.assertEqual(noise, ft.force_y_noise())

    self.assertEqual(defaultNoise, ft.force_z_noise())
    ft.set_force_z_noise(noise)
    self.assertEqual(noise, ft.force_z_noise())

    self.assertEqual(defaultNoise, ft.torque_x_noise())
    ft.set_torque_x_noise(noise)
    self.assertEqual(noise, ft.torque_x_noise())

    self.assertEqual(defaultNoise, ft.torque_y_noise())
    ft.set_torque_y_noise(noise)
    self.assertEqual(noise, ft.torque_y_noise())

    self.assertEqual(defaultNoise, ft.torque_z_noise())
    ft.set_torque_z_noise(noise)
    self.assertEqual(noise, ft.torque_z_noise())

    self.assertEqual(ft.frame(), sdf.ForceTorqueFrame.CHILD)
    ft.set_frame(sdf.ForceTorqueFrame.PARENT)
    self.assertEqual(ft.frame(), sdf.ForceTorqueFrame.PARENT)

    self.assertEqual(ft.measure_direction(),
            sdf.ForceTorqueMeasureDirection.CHILD_TO_PARENT)
    ft.set_measure_direction(sdf.ForceTorqueMeasureDirection.PARENT_TO_CHILD)
    self.assertEqual(ft.measure_direction(),
            sdf.ForceTorqueMeasureDirection.PARENT_TO_CHILD)

    # Copy Constructor
    ft2 = ForceTorque(ft)
    self.assertEqual(ft, ft2)

    # Assignment
    ft = ft2
    self.assertEqual(ft2, ft)

    # Copy assignment
    ft = copy.deepcopy(ft2)
    self.assertEqual(ft2, ft)

    # inequality
    ft6 = ForceTorque()
    self.assertNotEqual(ft2, ft6)


if __name__ == '__main__':
    unittest.main()
