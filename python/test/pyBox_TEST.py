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

from gz.math import Inertiald, MassMatrix3d, Pose3d, Vector3d
from sdformat import Box
import unittest

class BoxTEST(unittest.TestCase):

  def test_default_construction(self):
    box = Box()

    self.assertEqual(Vector3d.ONE, box.size())

    box.set_size(Vector3d.ZERO)
    self.assertEqual(Vector3d.ZERO, box.size())


  def test_assignment(self):
    size = Vector3d(1, 2, 3)

    box = Box()
    box.set_size(size)

    box2 = box
    self.assertEqual(size, box2.size())

    self.assertEqual(1 * 2 * 3, box2.shape().volume())
    self.assertEqual(size, box2.shape().size())

  def test_copy_construction(self):
    size = Vector3d(0.1, 0.2, 0.3)

    box = Box()
    box.set_size(size)

    box2 = Box(box)
    self.assertEqual(size, box2.size())

  def test_shape(self):
    box = Box()
    self.assertEqual(Vector3d.ONE, box.size())

    box.shape().set_size(Vector3d(1, 2, 3))
    self.assertEqual(Vector3d(1, 2, 3), box.size())

  def test_calculate_inertial(self):
    box = Box()
    density = 2710

    box.set_size(Vector3d(-1, 1, 0))
    invalidBoxInertial = box.calculate_inertial(density)
    self.assertEqual(None, invalidBoxInertial)

    l = 2.0
    w = 2.0
    h = 2.0
    box.set_size(Vector3d(l, w, h))

    expectedMass = box.shape().volume() * density
    ixx = (1.0 / 12.0) * expectedMass * (w * w + h * h)
    iyy = (1.0 / 12.0) * expectedMass * (l * l + h * h)
    izz = (1.0 / 12.0) * expectedMass * (l * l + w * w)

    expectedMassMat = MassMatrix3d(expectedMass, Vector3d(ixx, iyy, izz), Vector3d.ZERO)

    expectedInertial = Inertiald()
    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    boxInertial = box.calculate_inertial(density)
    self.assertEqual(box.shape().material().density(), density)
    self.assertNotEqual(None, boxInertial)
    self.assertEqual(expectedInertial, boxInertial)
    self.assertEqual(expectedInertial.mass_matrix().diagonal_moments(),
      boxInertial.mass_matrix().diagonal_moments())
    self.assertEqual(expectedInertial.mass_matrix().mass(),
      boxInertial.mass_matrix().mass())
    self.assertEqual(expectedInertial.pose(), boxInertial.pose())

if __name__ == '__main__':
    unittest.main()
