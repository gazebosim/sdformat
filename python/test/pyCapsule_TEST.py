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

import math

from gz.math import AxisAlignedBox, Inertiald, MassMatrix3d, Pose3d, Vector3d
from sdformat import Capsule

import unittest


class CapsuleTEST(unittest.TestCase):

  def test_default_construction(self):
    capsule = Capsule()

    self.assertEqual(math.pi * math.pow(0.5, 2) * (1.0 + 4./3. * 0.5),
                     capsule.shape().volume())

    self.assertEqual(0.5, capsule.radius())
    self.assertEqual(1.0, capsule.length())

    capsule.set_radius(0.5)
    capsule.set_length(2.3)

    self.assertEqual(0.5, capsule.radius())
    self.assertEqual(2.3, capsule.length())

  def test_move_construction(self):
    capsule = Capsule()
    capsule.set_radius(0.2)
    capsule.set_length(3.0)
    self.assertEqual(math.pi * math.pow(0.2, 2) * (3.0 + 4./3. * 0.2),
                   capsule.shape().volume())

    capsule2 = capsule
    self.assertEqual(0.2, capsule2.radius())
    self.assertEqual(3.0, capsule2.length())

    self.assertEqual(math.pi * math.pow(0.2, 2) * (3.0 + 4./3. * 0.2),
                   capsule2.shape().volume())
    self.assertEqual(0.2, capsule2.shape().radius())
    self.assertEqual(3.0, capsule2.shape().length())


  def test_copy_construction(self):
    capsule = Capsule()
    capsule.set_radius(0.2)
    capsule.set_length(3.0)

    capsule2 = Capsule(capsule)
    self.assertEqual(0.2, capsule2.radius())
    self.assertEqual(3.0, capsule2.length())

  def test_copy_construction(self):
    capsule = Capsule()
    capsule.set_radius(0.2)
    capsule.set_length(3.0)

    capsule2 = copy.deepcopy(capsule)
    self.assertEqual(0.2, capsule2.radius())
    self.assertEqual(3.0, capsule2.length())


  def test_assignment_after_move(self):
    capsule1 = Capsule()
    capsule1.set_radius(0.2)
    capsule1.set_length(3.0)

    capsule2 = Capsule()
    capsule2.set_radius(2)
    capsule2.set_length(30.0)

    # This is similar to what std::swap does except it uses std::move for each
    # assignment
    tmp = capsule1
    capsule1 = copy.deepcopy(capsule2)
    capsule2 = tmp

    self.assertEqual(2, capsule1.radius())
    self.assertEqual(30, capsule1.length())

    self.assertEqual(0.2, capsule2.radius())
    self.assertEqual(3.0, capsule2.length())


  def test_shape(self):
    capsule = Capsule()
    self.assertEqual(0.5, capsule.radius())
    self.assertEqual(1.0, capsule.length())

    capsule.shape().set_radius(0.123)
    capsule.shape().set_length(0.456)
    self.assertEqual(0.123, capsule.radius())
    self.assertEqual(0.456, capsule.length())

  def test_calculate_inertial(self):
    capsule = Capsule()

    # density of Aluminum
    density = 2710
    l = 2.0
    r = 0.1

    capsule.set_length(l)
    capsule.set_radius(r)

    expectedMass = capsule.shape().volume() * density
    cylinderVolume = math.pi * r * r * l
    sphereVolume = math.pi * 4. / 3. * r * r * r
    volume = cylinderVolume + sphereVolume
    cylinderMass = expectedMass * cylinderVolume / volume
    sphereMass = expectedMass * sphereVolume / volume

    ixxIyy = (1 / 12.0) * cylinderMass * (3 * r * r + l * l) + sphereMass * (
      0.4 * r * r + 0.375 * r * l + 0.25 * l * l)
    izz = r*r * (0.5 * cylinderMass + 0.4 * sphereMass)

    expectedMassMat = MassMatrix3d(expectedMass, Vector3d(ixxIyy, ixxIyy, izz), Vector3d.ZERO)

    expectedInertial = Inertiald()
    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    capsuleInertial = capsule.calculate_inertial(density)
    self.assertEqual(capsule.shape().material().density(), density)
    self.assertNotEqual(None, capsuleInertial)
    self.assertEqual(expectedInertial, capsuleInertial)
    self.assertEqual(expectedInertial.mass_matrix().diagonal_moments(),
      capsuleInertial.mass_matrix().diagonal_moments())
    self.assertEqual(expectedInertial.mass_matrix().mass(),
      capsuleInertial.mass_matrix().mass())
    self.assertEqual(expectedInertial.pose(), capsuleInertial.pose())

  def test_axis_aligned_box(self):
    capsule = Capsule()
    capsule.set_radius(0.5)
    capsule.set_length(3.0)

    self.assertEqual(
      AxisAlignedBox(Vector3d(-0.5, -0.5, -2.0), Vector3d(0.5, 0.5, 2.0)),
      capsule.axis_aligned_box())


if __name__ == '__main__':
    unittest.main()
