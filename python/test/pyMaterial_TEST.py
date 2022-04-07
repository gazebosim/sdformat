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
from sdformat import Material
from ignition.math import Color
import unittest


class GeometryTEST(unittest.TestCase):

  def test_default_construction(self):
    material = Material()
    self.assertEqual(Color(0, 0, 0, 1), material.ambient())
    self.assertEqual(Color(0, 0, 0, 1), material.diffuse())
    self.assertEqual(Color(0, 0, 0, 1), material.specular())
    self.assertEqual(Color(0, 0, 0, 1), material.emissive())
    self.assertTrue(material.lighting())
    self.assertEqual(0, material.render_order())
    self.assertFalse(material.double_sided())
    self.assertEqual("", material.script_uri())
    self.assertEqual("", material.script_name())
    self.assertEqual(Material.ShaderType.PIXEL, material.shader())
    self.assertEqual("", material.normal_map())
    # TODO(ahcorde) Add Pbr python interface
    # self.assertEqual(nullptr, material.PbrMaterial())
    self.assertEqual("", material.file_path())


  def test_assignment(self):
    material = Material()
    material.set_ambient(Color(0.1, 0.2, 0.3, 0.5))
    material.set_diffuse(Color(0.2, 0.3, 0.4, 0.6))
    material.set_specular(Color(0.3, 0.4, 0.5, 0.7))
    material.set_emissive(Color(0.4, 0.5, 0.6, 0.8))
    material.set_lighting(False)
    material.set_render_order(2)
    material.set_double_sided(True)
    material.set_script_uri("banana")
    material.set_script_name("orange")
    material.set_shader(Material.ShaderType.VERTEX)
    material.set_normal_map("blueberry")
    material.set_file_path("/tmp/path")

    material2 = material
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7),
      material2.specular())
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8),
      material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(2.0, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(Material.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    # TODO(ahcorde) Add Pbr python interface
    # self.assertEqual(nullptr, material2.PbrMaterial())
    self.assertEqual("/tmp/path", material2.file_path())


  def test_copy_construction(self):
    material = Material()
    material.set_ambient(Color(0.1, 0.2, 0.3, 0.5))
    material.set_diffuse(Color(0.2, 0.3, 0.4, 0.6))
    material.set_specular(Color(0.3, 0.4, 0.5, 0.7))
    material.set_emissive(Color(0.4, 0.5, 0.6, 0.8))
    material.set_lighting(False)
    material.set_render_order(4)
    material.set_double_sided(True)
    material.set_script_uri("banana")
    material.set_script_name("orange")
    material.set_shader(Material.ShaderType.VERTEX)
    material.set_normal_map("blueberry")
    material.set_file_path("/tmp/other")

    material2 = Material(material)
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7),
      material2.specular())
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8),
      material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(4, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(Material.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    # TODO(ahcorde) Add Pbr python interface
    # self.assertEqual(nullptr, material2.PbrMaterial())
    self.assertEqual("/tmp/other", material2.file_path())


  def test_deepcopy_after_assignment(self):
    material1 = Material()
    material1.set_script_uri("material1")

    material2 = Material()
    material2.set_script_uri("material2")

    # This is similar to what std::swap does except it uses std::move for each
    # assignment
    tmp = copy.deepcopy(material1)
    material1 = material2
    material2 = tmp

    self.assertEqual("material2", material1.script_uri())
    self.assertEqual("material1", material2.script_uri())


  def test_set(self):
    material = Material()
    self.assertEqual(Color(0, 0, 0, 1), material.ambient())
    material.set_ambient(Color(0.1, 0.2, 0.3, 0.5))
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material.ambient())

    self.assertEqual(Color(0, 0, 0, 1), material.diffuse())
    material.set_diffuse(Color(0.2, 0.3, 0.4, 0.6))
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material.diffuse())

    self.assertEqual(Color(0, 0, 0, 1), material.specular())
    material.set_specular(Color(0.3, 0.4, 0.5, 0.7))
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7), material.specular())

    self.assertEqual(Color(0, 0, 0, 1), material.emissive())
    material.set_emissive(Color(0.4, 0.5, 0.6, 0.8))
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8), material.emissive())

    self.assertTrue(material.lighting())
    material.set_lighting(False)
    self.assertFalse(material.lighting())

    self.assertEqual(0, material.render_order())
    material.set_render_order(5)
    self.assertEqual(5, material.render_order())

    self.assertFalse(material.double_sided())
    material.set_double_sided(True)
    self.assertTrue(material.double_sided())

    self.assertEqual("", material.script_uri())
    material.set_script_uri("uri")
    self.assertEqual("uri", material.script_uri())

    self.assertEqual("", material.script_name())
    material.set_script_name("name")
    self.assertEqual("name", material.script_name())

    self.assertEqual(Material.ShaderType.PIXEL, material.shader())
    material.set_shader(Material.ShaderType.VERTEX)
    self.assertEqual(Material.ShaderType.VERTEX, material.shader())

    self.assertEqual("", material.normal_map())
    material.set_normal_map("map")
    self.assertEqual("map", material.normal_map())

    self.assertEqual("", material.file_path())
    material.set_file_path("/my/path")
    self.assertEqual("/my/path", material.file_path())

    # TODO(ahcorde) Add Pbr python interface
    # set pbr material
    # sdf::Pbr pbr
    # sdf::PbrWorkflow workflow
    # workflow.SetType(sdf::PbrWorkflowType::METAL)
    # pbr.SetWorkflow(workflow.Type(), workflow)
    # material.SetPbrMaterial(pbr)
    # EXPECT_NE(material.PbrMaterial(), nullptr)
    # self.assertEqual(workflow,
    #   *material.PbrMaterial()->Workflow(sdf::PbrWorkflowType::METAL))

    # Move the material
    moved = material
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), moved.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), moved.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7), moved.specular())
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8), moved.emissive())
    self.assertFalse(moved.lighting())
    self.assertEqual(5, moved.render_order())
    self.assertTrue(moved.double_sided())
    self.assertEqual("uri", moved.script_uri())
    self.assertEqual("name", moved.script_name())
    self.assertEqual(Material.ShaderType.VERTEX, moved.shader())
    self.assertEqual("map", moved.normal_map())
    # TODO(ahcorde) Add Pbr python interface
    # self.assertEqual(workflow,
    #   *moved.PbrMaterial()->Workflow(sdf::PbrWorkflowType::METAL))
    self.assertEqual("/my/path", moved.file_path())


if __name__ == '__main__':
    unittest.main()
