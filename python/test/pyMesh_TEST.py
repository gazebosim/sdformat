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
from sdformat import (Mesh, ConvexDecomposition, Element, Error, ErrorCode,
                      ParserConfig, SDFErrorsException)
from gz.math import AxisAlignedBox, Inertiald, MassMatrix3d, Pose3d, Vector3d
import sdformat as sdf
from typing import Optional
import unittest


class MeshTEST(unittest.TestCase):

  def test_default_construction(self):
    mesh = Mesh()

    self.assertEqual("", mesh.optimization_str())
    self.assertEqual(sdf.MeshOptimization.NONE, mesh.optimization())
    self.assertEqual(None, mesh.convex_decomposition())
    self.assertEqual("", mesh.file_path())
    self.assertEqual("", mesh.uri())
    self.assertEqual("", mesh.submesh())
    self.assertTrue(Vector3d(1, 1, 1) == mesh.scale())
    self.assertFalse(mesh.center_submesh())


  def test_assigment(self):
    mesh = Mesh()
    self.assertTrue(mesh.set_optimization("convex_decomposition"))
    mesh.set_uri("banana")
    mesh.set_submesh("watermelon")
    mesh.set_center_submesh(True)
    mesh.set_scale(Vector3d(0.5, 0.6, 0.7))
    mesh.set_file_path("/pear")

    convexDecomp = ConvexDecomposition()
    convexDecomp.set_max_convex_hulls(10)
    convexDecomp.set_voxel_resolution(100000)
    mesh.set_convex_decomposition(convexDecomp)

    mesh2 = mesh
    self.assertEqual("convex_decomposition", mesh2.optimization_str())
    self.assertEqual(sdf.MeshOptimization.CONVEX_DECOMPOSITION, mesh2.optimization())
    self.assertEqual("banana", mesh2.uri())
    self.assertEqual("watermelon", mesh2.submesh())
    self.assertEqual(Vector3d(0.5, 0.6, 0.7), mesh2.scale())
    self.assertTrue(mesh2.center_submesh())
    self.assertEqual("/pear", mesh2.file_path())

    convexDecomp2 = mesh2.convex_decomposition()
    self.assertEqual(10, convexDecomp2.max_convex_hulls())
    self.assertEqual(100000, convexDecomp2.voxel_resolution())

    mesh.set_file_path("/apple")
    self.assertEqual("/apple", mesh2.file_path())

    mesh.set_scale(Vector3d(0.3, 0.2, 0.4))
    self.assertEqual(Vector3d(0.3, 0.2, 0.4), mesh2.scale())

    mesh.set_center_submesh(False)
    self.assertFalse(mesh2.center_submesh())

    mesh.set_submesh("melon")
    self.assertEqual("melon", mesh2.submesh())

    mesh.set_uri("pineapple")
    self.assertEqual("pineapple", mesh2.uri())


  def test_deepcopy_construction(self):
    mesh = Mesh()
    self.assertTrue(mesh.set_optimization("convex_decomposition"))
    mesh.set_uri("banana")
    mesh.set_submesh("watermelon")
    mesh.set_center_submesh(True)
    mesh.set_scale(Vector3d(0.5, 0.6, 0.7))
    mesh.set_file_path("/pear")

    convexDecomp = ConvexDecomposition()
    convexDecomp.set_max_convex_hulls(10)
    convexDecomp.set_voxel_resolution(100000)
    mesh.set_convex_decomposition(convexDecomp)

    mesh2 = copy.deepcopy(mesh)
    self.assertEqual("convex_decomposition", mesh2.optimization_str())
    self.assertEqual(sdf.MeshOptimization.CONVEX_DECOMPOSITION, mesh2.optimization())
    self.assertEqual("banana", mesh2.uri())
    self.assertEqual("watermelon", mesh2.submesh())
    self.assertEqual(Vector3d(0.5, 0.6, 0.7), mesh2.scale())
    self.assertTrue(mesh2.center_submesh())
    self.assertEqual("/pear", mesh2.file_path())

    convexDecomp2 = mesh2.convex_decomposition()
    self.assertEqual(10, convexDecomp2.max_convex_hulls())
    self.assertEqual(100000, convexDecomp2.voxel_resolution())

    mesh.set_file_path("/apple")
    mesh.set_scale(Vector3d(0.3, 0.2, 0.4))
    mesh.set_center_submesh(False)
    mesh.set_submesh("melon")
    mesh.set_uri("pineapple")

    self.assertEqual("banana", mesh2.uri())
    self.assertEqual("watermelon", mesh2.submesh())
    self.assertEqual(Vector3d(0.5, 0.6, 0.7), mesh2.scale())
    self.assertTrue(mesh2.center_submesh())
    self.assertEqual("/pear", mesh2.file_path())

  def test_set(self):
    mesh = Mesh()

    self.assertEqual("", mesh.optimization_str())
    self.assertTrue(mesh.set_optimization("convex_hull"))
    self.assertEqual("convex_hull", mesh.optimization_str())
    self.assertEqual(sdf.MeshOptimization.CONVEX_HULL, mesh.optimization())
    mesh.set_optimization(sdf.MeshOptimization.CONVEX_DECOMPOSITION)
    self.assertEqual("convex_decomposition", mesh.optimization_str())
    self.assertEqual(sdf.MeshOptimization.CONVEX_DECOMPOSITION, mesh.optimization())

    self.assertFalse(mesh.set_optimization("invalid"))
    mesh.set_optimization(sdf.MeshOptimization(99))
    self.assertEqual(sdf.MeshOptimization(99), mesh.optimization())
    self.assertEqual("", mesh.optimization_str())

    convexDecomp = ConvexDecomposition()
    convexDecomp.set_max_convex_hulls(10)
    convexDecomp.set_voxel_resolution(100000)
    mesh.set_convex_decomposition(convexDecomp)
    self.assertEqual(10, mesh.convex_decomposition().max_convex_hulls())
    self.assertEqual(100000, mesh.convex_decomposition().voxel_resolution())

    self.assertEqual("", mesh.uri())
    mesh.set_uri("http://myuri.com")
    self.assertEqual("http://myuri.com", mesh.uri())

    self.assertEqual("", mesh.submesh())
    mesh.set_submesh("my_submesh")
    self.assertEqual("my_submesh", mesh.submesh())

    self.assertTrue(Vector3d(1, 1, 1) == mesh.scale())
    mesh.set_scale(Vector3d(0.2, 1.4, 7.8))
    self.assertTrue(Vector3d(0.2, 1.4, 7.8) == mesh.scale())

    self.assertFalse(mesh.center_submesh())
    mesh.set_center_submesh(True)
    self.assertTrue(mesh.center_submesh())

    self.assertEqual("", mesh.file_path())
    mesh.set_file_path("/mypath")
    self.assertEqual("/mypath", mesh.file_path())

  def test_axis_aligned_box(self):
    mesh = Mesh()
    self.assertEqual(None, mesh.axis_aligned_box(None))

    # Customly defined Mesh AABB calculator
    def mesh_aabb_calulator(sdf_mesh: Mesh) -> Optional[AxisAlignedBox]:
      if sdf_mesh.uri() == "no_mesh":
        return None

      if sdf_mesh.uri() == "banana":
        # Banana mesh should return invalid AABB
        return AxisAlignedBox()

      return AxisAlignedBox(Vector3d(-1, -1, -1), Vector3d(1, 1, 1))

    mesh.set_uri("no_mesh")
    self.assertEqual(
      None,
      mesh.axis_aligned_box(mesh_aabb_calulator))

    mesh.set_uri("banana")
    self.assertEqual(
      AxisAlignedBox(),
      mesh.axis_aligned_box(mesh_aabb_calulator))

    mesh.set_uri("apple")
    self.assertEqual(
      AxisAlignedBox(Vector3d(-1, -1, -1), Vector3d(1, 1, 1)),
      mesh.axis_aligned_box(mesh_aabb_calulator))

  def test_calculate_inertial(self):
    mesh = Mesh()
    mesh.set_uri("mesh.dae")
    auto_inertia_params = Element()
    auto_inertia_params.set_name("auto_inertia_params")
    config = ParserConfig()

    # Default inertial values are used without a custom calculator
    inertial = mesh.calculate_inertial(1240.0, auto_inertia_params, config)
    self.assertEqual(1.0, inertial.mass_matrix().mass())
    self.assertEqual(Vector3d.ONE, inertial.mass_matrix().diagonal_moments())

    calculator_props = []

    def custom_mesh_inertia_calculator(errors, props):
      calculator_props.append(props)
      if props.density() <= 0:
        errors.append(
          Error(ErrorCode.LINK_INERTIA_INVALID, "Inertia is invalid"))
        return None

      return Inertiald(
        MassMatrix3d(2.0, Vector3d.ONE, Vector3d.ZERO), Pose3d.ZERO)

    config.register_custom_inertia_calc(custom_mesh_inertia_calculator)

    with self.assertRaises(SDFErrorsException) as context:
      mesh.calculate_inertial(0.0, auto_inertia_params, config)
    self.assertEqual(1, len(context.exception.errors))
    self.assertEqual(ErrorCode.LINK_INERTIA_INVALID,
                     context.exception.errors[0].code())

    inertial = mesh.calculate_inertial(1240.0, auto_inertia_params, config)
    self.assertEqual(2.0, inertial.mass_matrix().mass())
    self.assertEqual(Vector3d.ONE, inertial.mass_matrix().diagonal_moments())

    # The calculator receives the mesh, density and auto inertia params
    self.assertEqual(2, len(calculator_props))
    props = calculator_props[-1]
    self.assertEqual(1240.0, props.density())
    self.assertEqual("mesh.dae", props.mesh().uri())
    self.assertEqual("auto_inertia_params",
                     props.auto_inertia_params().get_name())


if __name__ == '__main__':
    unittest.main()
