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
from gz_test_deps.sdformat import Sphere
from gz_test_deps.math import AxisAlignedBox, Inertiald, MassMatrix3d, Pose3d, Vector3d
import unittest

class SphereTEST(unittest.TestCase):

  def test_default_construction(self):
    sphere = Sphere()
    self.assertEqual(1.0, sphere.radius())
    sphere.set_radius(0.5)
    self.assertEqual(0.5, sphere.radius())


  def test_assignment(self):
    sphere = Sphere()
    sphere.set_radius(0.2)

    sphere2 = sphere
    self.assertEqual(0.2, sphere2.radius())


  def test_copy_construction(self):
    sphere = Sphere()
    sphere.set_radius(0.2)

    sphere2 = Sphere(sphere)
    self.assertEqual(0.2, sphere2.radius())

    self.assertEqual(4.0/3.0*math.pi*math.pow(0.2, 3), sphere2.shape().volume())
    self.assertEqual(0.2, sphere2.shape().radius())


  def test_deepcopy_construction(self):
    sphere = Sphere()
    sphere.set_radius(0.2)

    sphere2 = copy.deepcopy(sphere)
    self.assertEqual(0.2, sphere2.radius())


  def test_deepcopy_after_assignment(self):
    sphere1 = Sphere()
    sphere1.set_radius(0.1)

    sphere2 = Sphere()
    sphere2.set_radius(0.2)

    # This is similar to what swap does
    tmp = copy.deepcopy(sphere1)
    sphere1 = sphere2
    sphere2 = tmp

    self.assertEqual(0.2, sphere1.radius())
    self.assertEqual(0.1, sphere2.radius())


  def test_shape(self):
    sphere = Sphere()
    self.assertEqual(1.0, sphere.radius())

    sphere.shape().set_radius(0.123)
    self.assertEqual(0.123, sphere.radius())

  def test_calculate_inertial(self):
    sphere = Sphere()

    # density of aluminium
    density = 2170

    sphere.set_radius(-2)
    invalidSphereInertial = sphere.calculate_inertial(density)
    self.assertEqual(None, invalidSphereInertial)

    r = 0.1

    sphere.set_radius(r)

    expectedMass = sphere.shape().volume() * density
    ixxIyyIzz = 0.4 * expectedMass * r * r

    expectedMassMat = MassMatrix3d(
      expectedMass, Vector3d(ixxIyyIzz, ixxIyyIzz, ixxIyyIzz), Vector3d.ZERO)

    expectedInertial = Inertiald()
    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    sphereInertial = sphere.calculate_inertial(density)
    self.assertEqual(sphere.shape().material().density(), density)
    self.assertNotEqual(None, sphereInertial)
    self.assertEqual(expectedInertial, sphereInertial)
    self.assertEqual(expectedInertial.mass_matrix().diagonal_moments(),
      sphereInertial.mass_matrix().diagonal_moments())
    self.assertEqual(expectedInertial.mass_matrix().mass(),
      sphereInertial.mass_matrix().mass())
    self.assertEqual(expectedInertial.pose(), sphereInertial.pose())

  def test_axis_aligned_box(self):
    sphere = Sphere()
    sphere.set_radius(0.75)

    self.assertEqual(
      AxisAlignedBox(Vector3d(-0.75, -0.75, -0.75), Vector3d(0.75, 0.75, 0.75)),
      sphere.axis_aligned_box())


if __name__ == '__main__':
    unittest.main()
