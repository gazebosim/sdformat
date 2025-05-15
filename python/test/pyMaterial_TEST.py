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
from sdformat import Material, Pbr, PbrWorkflow
from gz.math import Color
import sdformat as sdf
import unittest


class MaterialTEST(unittest.TestCase):

  def test_default_construction(self):
    material = Material()
    self.assertEqual(Color(0, 0, 0, 1), material.ambient())
    self.assertEqual(Color(0, 0, 0, 1), material.diffuse())
    self.assertEqual(Color(0, 0, 0, 1), material.specular())
    self.assertAlmostEqual(0.0, material.shininess());
    self.assertEqual(Color(0, 0, 0, 1), material.emissive())
    self.assertTrue(material.lighting())
    self.assertEqual(0, material.render_order())
    self.assertFalse(material.double_sided())
    self.assertEqual("", material.script_uri())
    self.assertEqual("", material.script_name())
    self.assertEqual(sdf.ShaderType.PIXEL, material.shader())
    self.assertEqual("", material.normal_map())
    self.assertEqual(None, material.pbr_material())
    self.assertEqual("", material.file_path())


  def test_assignment(self):
    material = Material()
    material.set_ambient(Color(0.1, 0.2, 0.3, 0.5))
    material.set_diffuse(Color(0.2, 0.3, 0.4, 0.6))
    material.set_specular(Color(0.3, 0.4, 0.5, 0.7))
    material.set_shininess(5.0)
    material.set_emissive(Color(0.4, 0.5, 0.6, 0.8))
    material.set_lighting(False)
    material.set_render_order(2)
    material.set_double_sided(True)
    material.set_script_uri("banana")
    material.set_script_name("orange")
    material.set_shader(sdf.ShaderType.VERTEX)
    material.set_normal_map("blueberry")
    material.set_file_path("/tmp/path")

    material2 = material
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7), material2.specular())
    self.assertAlmostEqual(5.0, material2.shininess());
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8), material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(2.0, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(sdf.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    self.assertEqual(None, material2.pbr_material())
    self.assertEqual("/tmp/path", material2.file_path())

    material.set_ambient(Color(0.3, 0.1, 0.2, 0.3))
    material.set_diffuse(Color(0.4, 0.4, 0.5, 0.6))
    material.set_specular(Color(0.5, 0.6, 0.7, 0.8))
    material.set_shininess(6.0)
    material.set_emissive(Color(0.6, 0.7, 0.8, 0.9))
    material.set_lighting(True)
    material.set_render_order(2)
    material.set_double_sided(False)
    material.set_script_uri("apple")
    material.set_script_name("melon")
    material.set_shader(sdf.ShaderType.VERTEX)
    material.set_normal_map("banana")
    material.set_file_path("/tmp/foo")

    self.assertEqual(Color(0.3, 0.1, 0.2, 0.3), material2.ambient())
    self.assertEqual(Color(0.4, 0.4, 0.5, 0.6), material2.diffuse())
    self.assertEqual(Color(0.5, 0.6, 0.7, 0.8), material2.specular())
    self.assertAlmostEqual(6.0, material2.shininess());
    self.assertEqual(Color(0.6, 0.7, 0.8, 0.9), material2.emissive())
    self.assertTrue(material2.lighting())
    self.assertFalse(material2.double_sided())
    self.assertEqual(2, material2.render_order())
    self.assertEqual("apple", material2.script_uri())
    self.assertEqual("melon", material2.script_name())
    self.assertEqual(sdf.ShaderType.VERTEX, material2.shader())
    self.assertEqual("banana", material2.normal_map())
    self.assertEqual(None, material2.pbr_material())
    self.assertEqual("/tmp/foo", material2.file_path())

  def test_copy_construction(self):
    material = Material()
    material.set_ambient(Color(0.1, 0.2, 0.3, 0.5))
    material.set_diffuse(Color(0.2, 0.3, 0.4, 0.6))
    material.set_specular(Color(0.3, 0.4, 0.5, 0.7))
    material.set_shininess(5.0)
    material.set_emissive(Color(0.4, 0.5, 0.6, 0.8))
    material.set_lighting(False)
    material.set_render_order(4)
    material.set_double_sided(True)
    material.set_script_uri("banana")
    material.set_script_name("orange")
    material.set_shader(sdf.ShaderType.VERTEX)
    material.set_normal_map("blueberry")
    material.set_file_path("/tmp/other")

    material2 = Material(material)
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7),
      material2.specular())
    self.assertAlmostEqual(5.0, material2.shininess());
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8),
      material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(4, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(sdf.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    self.assertEqual(None, material2.pbr_material())
    self.assertEqual("/tmp/other", material2.file_path())

    material.set_ambient(Color(0.3, 0.1, 0.2, 0.3))
    material.set_diffuse(Color(0.4, 0.4, 0.5, 0.6))
    material.set_specular(Color(0.5, 0.6, 0.7, 0.8))
    material.set_shininess(6.0)
    material.set_emissive(Color(0.6, 0.7, 0.8, 0.9))
    material.set_lighting(True)
    material.set_render_order(2)
    material.set_double_sided(False)
    material.set_script_uri("apple")
    material.set_script_name("melon")
    material.set_shader(sdf.ShaderType.VERTEX)
    material.set_normal_map("banana")
    material.set_file_path("/tmp/foo")

    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7),
      material2.specular())
    self.assertAlmostEqual(5.0, material2.shininess());
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8),
      material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(4, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(sdf.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    self.assertEqual(None, material2.pbr_material())
    self.assertEqual("/tmp/other", material2.file_path())

  def test_deepcopy(self):
    material = Material()
    material.set_ambient(Color(0.1, 0.2, 0.3, 0.5))
    material.set_diffuse(Color(0.2, 0.3, 0.4, 0.6))
    material.set_specular(Color(0.3, 0.4, 0.5, 0.7))
    material.set_shininess(5.0)
    material.set_emissive(Color(0.4, 0.5, 0.6, 0.8))
    material.set_lighting(False)
    material.set_render_order(4)
    material.set_double_sided(True)
    material.set_script_uri("banana")
    material.set_script_name("orange")
    material.set_shader(sdf.ShaderType.VERTEX)
    material.set_normal_map("blueberry")
    material.set_file_path("/tmp/other")

    material2 = copy.deepcopy(material)
    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7),
      material2.specular())
    self.assertAlmostEqual(5.0, material2.shininess());
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8),
      material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(4, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(sdf.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    self.assertEqual(None, material2.pbr_material())
    self.assertEqual("/tmp/other", material2.file_path())

    material.set_ambient(Color(0.3, 0.1, 0.2, 0.3))
    material.set_diffuse(Color(0.4, 0.4, 0.5, 0.6))
    material.set_specular(Color(0.5, 0.6, 0.7, 0.8))
    material.set_shininess(6.0)
    material.set_emissive(Color(0.6, 0.7, 0.8, 0.9))
    material.set_lighting(True)
    material.set_render_order(2)
    material.set_double_sided(False)
    material.set_script_uri("apple")
    material.set_script_name("melon")
    material.set_shader(sdf.ShaderType.VERTEX)
    material.set_normal_map("banana")
    material.set_file_path("/tmp/foo")

    self.assertEqual(Color(0.1, 0.2, 0.3, 0.5), material2.ambient())
    self.assertEqual(Color(0.2, 0.3, 0.4, 0.6), material2.diffuse())
    self.assertEqual(Color(0.3, 0.4, 0.5, 0.7),
      material2.specular())
    self.assertAlmostEqual(5.0, material2.shininess());
    self.assertEqual(Color(0.4, 0.5, 0.6, 0.8),
      material2.emissive())
    self.assertFalse(material2.lighting())
    self.assertTrue(material2.double_sided())
    self.assertEqual(4, material2.render_order())
    self.assertEqual("banana", material2.script_uri())
    self.assertEqual("orange", material2.script_name())
    self.assertEqual(sdf.ShaderType.VERTEX, material2.shader())
    self.assertEqual("blueberry", material2.normal_map())
    self.assertEqual(None, material2.pbr_material())
    self.assertEqual("/tmp/other", material2.file_path())

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

    self.assertAlmostEqual(0.0, material.shininess());
    material.set_shininess(5.0)
    self.assertAlmostEqual(5.0, material.shininess());

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

    self.assertEqual(sdf.ShaderType.PIXEL, material.shader())
    material.set_shader(sdf.ShaderType.VERTEX)
    self.assertEqual(sdf.ShaderType.VERTEX, material.shader())

    self.assertEqual("", material.normal_map())
    material.set_normal_map("map")
    self.assertEqual("map", material.normal_map())

    self.assertEqual("", material.file_path())
    material.set_file_path("/my/path")
    self.assertEqual("/my/path", material.file_path())

    pbr = Pbr()
    workflow = PbrWorkflow()
    workflow.set_type(sdf.PbrWorkflowType.METAL)
    pbr.set_workflow(workflow.type(), workflow)
    material.set_pbr_material(pbr)
    self.assertNotEqual(material.pbr_material(), None)
    self.assertEqual(workflow,
      material.pbr_material().workflow(sdf.PbrWorkflowType.METAL))


if __name__ == '__main__':
    unittest.main()
