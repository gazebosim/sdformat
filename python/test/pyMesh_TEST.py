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
from sdformat import Mesh
from gz.math import Vector3d
import unittest


class MeshTEST(unittest.TestCase):

  def test_default_construction(self):
    mesh = Mesh()

    self.assertEqual("", mesh.file_path())
    self.assertEqual("", mesh.uri())
    self.assertEqual("", mesh.submesh())
    self.assertTrue(Vector3d(1, 1, 1) == mesh.scale())
    self.assertFalse(mesh.center_submesh())


  def test_assigment(self):
    mesh = Mesh()
    mesh.set_uri("banana")
    mesh.set_submesh("watermelon")
    mesh.set_center_submesh(True)
    mesh.set_scale(Vector3d(0.5, 0.6, 0.7))
    mesh.set_file_path("/pear")

    mesh2 = mesh
    self.assertEqual("banana", mesh2.uri())
    self.assertEqual("watermelon", mesh2.submesh())
    self.assertEqual(Vector3d(0.5, 0.6, 0.7), mesh2.scale())
    self.assertTrue(mesh2.center_submesh())
    self.assertEqual("/pear", mesh2.file_path())

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
    mesh.set_uri("banana")
    mesh.set_submesh("watermelon")
    mesh.set_center_submesh(True)
    mesh.set_scale(Vector3d(0.5, 0.6, 0.7))
    mesh.set_file_path("/pear")

    mesh2 = copy.deepcopy(mesh)
    self.assertEqual("banana", mesh2.uri())
    self.assertEqual("watermelon", mesh2.submesh())
    self.assertEqual(Vector3d(0.5, 0.6, 0.7), mesh2.scale())
    self.assertTrue(mesh2.center_submesh())
    self.assertEqual("/pear", mesh2.file_path())

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
