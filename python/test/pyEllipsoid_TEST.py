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
from gz.math import AxisAlignedBox, Inertiald, MassMatrix3d, Pose3d, Vector3d
import math
from sdformat import Ellipsoid
import unittest


class BoxTEST(unittest.TestCase):

  def test_default_construction(self):
    ellipsoid = Ellipsoid()
    # A default ellipsoid has all three radii set to 1
    self.assertEqual(math.pi * 4. / 3., ellipsoid.shape().volume())
    self.assertEqual(Vector3d.ONE, ellipsoid.shape().radii())

    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.set_radii(expectedradii)
    self.assertEqual(expectedradii, ellipsoid.shape().radii())


  def test_assignment(self):
    ellipsoid = Ellipsoid()
    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.set_radii(expectedradii)

    ellipsoid2 = ellipsoid
    self.assertEqual(expectedradii, ellipsoid2.shape().radii())


  def test_deepcopy(self):
    ellipsoid = Ellipsoid()
    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.set_radii(expectedradii)

    ellipsoid2 = copy.deepcopy(ellipsoid)
    self.assertEqual(expectedradii, ellipsoid2.shape().radii())


  def test_deepcopy_after_assignment(self):
      ellipsoid1 = Ellipsoid()
      expectedradii1 = Vector3d(1.0, 2.0, 3.0)
      ellipsoid1.set_radii(expectedradii1)

      ellipsoid2 = Ellipsoid()
      expectedradii2 = Vector3d(10.0, 20.0, 30.0)
      ellipsoid2.set_radii(expectedradii2)

      # This is similar to what std::swap does except it uses std::move for each
      # assignment
      tmp = copy.deepcopy(ellipsoid1)
      ellipsoid1 = ellipsoid2
      ellipsoid2 = tmp

      self.assertEqual(expectedradii1, ellipsoid2.shape().radii())
      self.assertEqual(expectedradii2, ellipsoid1.shape().radii())


  def test_shape(self):
    ellipsoid = Ellipsoid()
    self.assertEqual(Vector3d.ONE, ellipsoid.radii())

    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.shape().set_radii(expectedradii)
    self.assertEqual(expectedradii, ellipsoid.radii())

  def test_calculate_inertial(self):
    ellipsoid = Ellipsoid()

    # density of Aluminum
    density = 2170

    # Invalid dimensions leading to std::nullopt return in
    # CalculateInertial()
    ellipsoid.set_radii(Vector3d(-1, 2, 0))
    invalidEllipsoidInertial = ellipsoid.calculate_inertial(density)
    self.assertEqual(None, invalidEllipsoidInertial)

    a = 1.0
    b = 10.0
    c = 100.0

    ellipsoid.set_radii(Vector3d(a, b, c))

    expectedMass = ellipsoid.shape().volume() * density
    ixx = (expectedMass / 5.0) * (b * b + c * c)
    iyy = (expectedMass / 5.0) * (a * a + c * c)
    izz = (expectedMass / 5.0) * (a * a + b * b)

    expectedMassMat = MassMatrix3d(expectedMass, Vector3d(ixx, iyy, izz), Vector3d.ZERO)

    expectedInertial = Inertiald()
    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    ellipsoidInertial = ellipsoid.calculate_inertial(density)
    self.assertEqual(ellipsoid.shape().material().density(), density)
    self.assertNotEqual(None, ellipsoidInertial)
    self.assertEqual(expectedInertial, ellipsoidInertial)
    self.assertEqual(expectedInertial.mass_matrix().diagonal_moments(),
      ellipsoidInertial.mass_matrix().diagonal_moments())
    self.assertEqual(expectedInertial.mass_matrix().mass(),
      ellipsoidInertial.mass_matrix().mass())
    self.assertEqual(expectedInertial.pose(), ellipsoidInertial.pose())

  def test_axis_aligned_box(self):
    ellipsoid = Ellipsoid()
    ellipsoid.set_radii(Vector3d(1.0, 2.0, 3.0))

    self.assertEqual(
      AxisAlignedBox(Vector3d(-1.0, -2.0, -3.0), Vector3d(1.0, 2.0, 3.0)),
      ellipsoid.axis_aligned_box())


if __name__ == '__main__':
    unittest.main()
