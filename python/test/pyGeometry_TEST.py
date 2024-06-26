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
from gz_test_deps.sdformat import (Element, Error, Geometry, Box, Capsule, Cone, Cylinder, Ellipsoid,
                                   Mesh, ParserConfig, Plane, Sphere)
from gz_test_deps.math import Inertiald, MassMatrix3d, Pose3d, Vector3d, Vector2d
import gz_test_deps.sdformat as sdf
import math
import unittest


class GeometryTEST(unittest.TestCase):

  def test_default_construction(self):
    geom = Geometry()
    self.assertEqual(sdf.GeometryType.EMPTY, geom.type())

    geom.set_type(sdf.GeometryType.BOX)
    self.assertEqual(sdf.GeometryType.BOX, geom.type())

    geom.set_type(sdf.GeometryType.CAPSULE)
    self.assertEqual(sdf.GeometryType.CAPSULE, geom.type())

    geom.set_type(sdf.GeometryType.CONE)
    self.assertEqual(sdf.GeometryType.CONE, geom.type())

    geom.set_type(sdf.GeometryType.CYLINDER)
    self.assertEqual(sdf.GeometryType.CYLINDER, geom.type())

    geom.set_type(sdf.GeometryType.ELLIPSOID)
    self.assertEqual(sdf.GeometryType.ELLIPSOID, geom.type())

    geom.set_type(sdf.GeometryType.PLANE)
    self.assertEqual(sdf.GeometryType.PLANE, geom.type())

    geom.set_type(sdf.GeometryType.SPHERE)
    self.assertEqual(sdf.GeometryType.SPHERE, geom.type())


  def test_assignment(self):
    geometry = Geometry()
    geometry.set_type(sdf.GeometryType.BOX)

    geometry2 = geometry
    self.assertEqual(sdf.GeometryType.BOX, geometry2.type())


  def test_deepcopy_construction(self):
    geometry = Geometry()
    geometry.set_type(sdf.GeometryType.BOX)
    boxShape = Box()
    boxShape.set_size(Vector3d(1, 2, 3))
    geometry.set_box_shape(boxShape)

    geometry2 = copy.deepcopy(geometry)
    self.assertEqual(sdf.GeometryType.BOX, geometry2.type())


  def test_deepcopy_after_assignment(self):
    geometry1 = Geometry()
    geometry1.set_type(sdf.GeometryType.BOX)

    geometry2 = Geometry()
    geometry2.set_type(sdf.GeometryType.SPHERE)

    # This is similar to what std::swap does except it uses std::move for each
    # assignment
    tmp = copy.deepcopy(geometry1)
    geometry1 = geometry2
    geometry2 = tmp

    self.assertEqual(sdf.GeometryType.SPHERE, geometry1.type())
    self.assertEqual(sdf.GeometryType.BOX, geometry2.type())


  def test_box(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.BOX)

    boxShape = Box()
    boxShape.set_size(Vector3d(1, 2, 3))
    geom.set_box_shape(boxShape)

    self.assertEqual(sdf.GeometryType.BOX, geom.type())
    self.assertNotEqual(None, geom.box_shape())
    self.assertEqual(Vector3d(1, 2, 3), geom.box_shape().size())


  def test_sphere(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.SPHERE)

    sphereShape = Sphere()
    sphereShape.set_radius(0.123)
    geom.set_sphere_shape(sphereShape)

    self.assertEqual(sdf.GeometryType.SPHERE, geom.type())
    self.assertNotEqual(None, geom.sphere_shape())
    self.assertEqual(0.123, geom.sphere_shape().radius())


  def test_capsule(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.CAPSULE)

    capsuleShape = Capsule()
    capsuleShape.set_radius(0.123)
    capsuleShape.set_length(4.56)
    geom.set_capsule_shape(capsuleShape)

    self.assertEqual(sdf.GeometryType.CAPSULE, geom.type())
    self.assertNotEqual(None, geom.capsule_shape())
    self.assertEqual(0.123, geom.capsule_shape().radius())
    self.assertEqual(4.56, geom.capsule_shape().length())


  def test_cone(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.CONE)

    coneShape = Cone()
    coneShape.set_radius(0.123)
    coneShape.set_length(4.56)
    geom.set_cone_shape(coneShape)

    self.assertEqual(sdf.GeometryType.CONE, geom.type())
    self.assertNotEqual(None, geom.cone_shape())
    self.assertEqual(0.123, geom.cone_shape().radius())
    self.assertEqual(4.56, geom.cone_shape().length())


  def test_cylinder(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.CYLINDER)

    cylinderShape = Cylinder()
    cylinderShape.set_radius(0.123)
    cylinderShape.set_length(4.56)
    geom.set_cylinder_shape(cylinderShape)

    self.assertEqual(sdf.GeometryType.CYLINDER, geom.type())
    self.assertNotEqual(None, geom.cylinder_shape())
    self.assertEqual(0.123, geom.cylinder_shape().radius())
    self.assertEqual(4.56, geom.cylinder_shape().length())


  def test_ellipsoid(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.ELLIPSOID)

    ellipsoidShape = Ellipsoid()
    expectedRadii = Vector3d(1, 2, 3)
    ellipsoidShape.set_radii(expectedRadii)
    geom.set_ellipsoid_shape(ellipsoidShape)

    self.assertEqual(sdf.GeometryType.ELLIPSOID, geom.type())
    self.assertNotEqual(None, geom.ellipsoid_shape())
    self.assertEqual(expectedRadii, geom.ellipsoid_shape().radii())


  def test_mesh(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.MESH)

    meshShape = Mesh()
    meshShape.set_scale(Vector3d(1, 2, 3))
    meshShape.set_uri("banana")
    meshShape.set_submesh("orange")
    meshShape.set_center_submesh(True)
    geom.set_mesh_shape(meshShape)

    self.assertEqual(sdf.GeometryType.MESH, geom.type())
    self.assertNotEqual(None, geom.mesh_shape())
    self.assertEqual(Vector3d(1, 2, 3), geom.mesh_shape().scale())
    self.assertEqual("banana", geom.mesh_shape().uri())
    self.assertEqual("orange", geom.mesh_shape().submesh())
    self.assertTrue(geom.mesh_shape().center_submesh())


  def test_plane(self):
    geom = Geometry()
    geom.set_type(sdf.GeometryType.PLANE)

    planeShape = Plane()
    planeShape.set_normal(Vector3d.UNIT_X)
    planeShape.set_size(Vector2d(9, 8))
    geom.set_plane_shape(planeShape)

    self.assertEqual(sdf.GeometryType.PLANE, geom.type())
    self.assertNotEqual(None, geom.plane_shape())
    self.assertEqual(Vector3d.UNIT_X, geom.plane_shape().normal())
    self.assertEqual(Vector2d(9, 8), geom.plane_shape().size())


  def test_calculate_inertial(self):
    geom = Geometry()

    # Density of Aluminimum
    density = 2170.0
    expectedMass = 0
    expectedMassMat = MassMatrix3d()
    expectedInertial = Inertiald()
    sdfParserConfig = ParserConfig()
    errors = []

    # Not supported geom type
    geom.set_type(sdf.GeometryType.EMPTY)
    element = Element()
    notSupportedInertial = geom.calculate_inertial(errors,
      sdfParserConfig, density, element)
    self.assertEqual(notSupportedInertial, None)

    box = Box()
    l = 2.0
    w = 2.0
    h = 2.0
    box.set_size(Vector3d(l, w, h))

    expectedMass = box.shape().volume() * density
    ixx = (1.0 / 12.0) * expectedMass * (w * w + h * h)
    iyy = (1.0 / 12.0) * expectedMass * (l * l + h * h)
    izz = (1.0 / 12.0) * expectedMass * (l * l + w * w)

    expectedMassMat.set_mass(expectedMass)
    expectedMassMat.set_diagonal_moments(Vector3d(ixx, iyy, izz))
    expectedMassMat.set_off_diagonal_moments(Vector3d.ZERO)

    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    geom.set_type(sdf.GeometryType.BOX)
    geom.set_box_shape(box)
    boxInertial = geom.calculate_inertial(errors,
      sdfParserConfig, density, element)

    self.assertNotEqual(None, boxInertial)
    self.assertEqual(expectedInertial, boxInertial)
    self.assertEqual(expectedInertial.mass_matrix(), expectedMassMat)
    self.assertEqual(expectedInertial.pose(), boxInertial.pose())

    # Capsule
    capsule = Capsule()
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
    ixxIyy = (1 / 12.0) * cylinderMass * (3 * r *r + l * l) + sphereMass * (
      0.4 * r * r + 0.375 * r * l + 0.25 * l * l)
    izz = r * r * (0.5 * cylinderMass + 0.4 * sphereMass)

    expectedMassMat.set_mass(expectedMass)
    expectedMassMat.set_diagonal_moments(Vector3d(ixxIyy, ixxIyy, izz))
    expectedMassMat.set_off_diagonal_moments(Vector3d.ZERO)

    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    geom.set_type(sdf.GeometryType.CAPSULE)
    geom.set_capsule_shape(capsule)
    capsuleInertial = geom.calculate_inertial(errors,
      sdfParserConfig, density, element)

    self.assertNotEqual(None, capsuleInertial)
    self.assertEqual(expectedInertial, capsuleInertial)
    self.assertEqual(expectedInertial.mass_matrix(), expectedMassMat)
    self.assertEqual(expectedInertial.pose(), capsuleInertial.pose())

    # Cylinder
    cylinder = Cylinder()
    l = 2.0
    r = 0.1

    cylinder.set_length(l)
    cylinder.set_radius(r)

    expectedMass = cylinder.shape().volume() * density
    ixxIyy = (1/12.0) * expectedMass * (3*r*r + l*l)
    izz = 0.5 * expectedMass * r * r

    expectedMassMat.set_mass(expectedMass)
    expectedMassMat.set_diagonal_moments(Vector3d(ixxIyy, ixxIyy, izz))
    expectedMassMat.set_off_diagonal_moments(Vector3d.ZERO)

    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    geom.set_type(sdf.GeometryType.CYLINDER)
    geom.set_cylinder_shape(cylinder)
    cylinderInertial = geom.calculate_inertial(errors,
      sdfParserConfig, density, element)

    self.assertNotEqual(None, cylinderInertial)
    self.assertEqual(expectedInertial, cylinderInertial)
    self.assertEqual(expectedInertial.mass_matrix(), expectedMassMat)
    self.assertEqual(expectedInertial.pose(), cylinderInertial.pose())

    # Ellipsoid
    ellipsoid = Ellipsoid()

    a = 1.0
    b = 10.0
    c = 100.0

    ellipsoid.set_radii(Vector3d(a, b, c))

    expectedMass = ellipsoid.shape().volume() * density
    ixx = (expectedMass / 5.0) * (b*b + c*c)
    iyy = (expectedMass / 5.0) * (a*a + c*c)
    izz = (expectedMass / 5.0) * (a*a + b*b)

    expectedMassMat.set_mass(expectedMass)
    expectedMassMat.set_diagonal_moments(Vector3d(ixx, iyy, izz))
    expectedMassMat.set_off_diagonal_moments(Vector3d.ZERO)

    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    geom.set_type(sdf.GeometryType.ELLIPSOID)
    geom.set_ellipsoid_shape(ellipsoid)
    ellipsoidInertial = geom.calculate_inertial(errors,
      sdfParserConfig, density, element)

    self.assertNotEqual(None, ellipsoidInertial)
    self.assertEqual(expectedInertial, ellipsoidInertial)
    self.assertEqual(expectedInertial.mass_matrix(), expectedMassMat)
    self.assertEqual(expectedInertial.pose(), ellipsoidInertial.pose())

    # Sphere
    sphere = Sphere()
    r = 0.1

    sphere.set_radius(r)

    expectedMass = sphere.shape().volume() * density
    ixxIyyIzz = 0.4 * expectedMass * r * r

    expectedMassMat.set_mass(expectedMass)
    expectedMassMat.set_diagonal_moments(
      Vector3d(ixxIyyIzz, ixxIyyIzz, ixxIyyIzz))
    expectedMassMat.set_off_diagonal_moments(Vector3d.ZERO)

    expectedInertial.set_mass_matrix(expectedMassMat)
    expectedInertial.set_pose(Pose3d.ZERO)

    geom.set_type(sdf.GeometryType.SPHERE)
    geom.set_sphere_shape(sphere)
    sphereInertial = geom.calculate_inertial(errors,
      sdfParserConfig, density, element)

    self.assertNotEqual(None, sphereInertial)
    self.assertEqual(expectedInertial, sphereInertial)
    self.assertEqual(expectedInertial.mass_matrix(), expectedMassMat)
    self.assertEqual(expectedInertial.pose(), sphereInertial.pose())

if __name__ == '__main__':
    unittest.main()
