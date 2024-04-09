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
from gz_test_deps.sdformat import Mesh, ConvexDecomposition
from gz_test_deps.math import Vector3d
import gz_test_deps.sdformat as sdf
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
    mesh.set_convex_decomposition(convexDecomp)
    self.assertEqual(10, mesh.convex_decomposition().max_convex_hulls())

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


if __name__ == '__main__':
    unittest.main()
