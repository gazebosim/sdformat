# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from sdformat import Pbr, PbrWorkflow
import sdformat as sdf
import unittest


class PbrTEST(unittest.TestCase):

    def test_default_construction(self):
        pbr = Pbr()
        self.assertEqual(None, pbr.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr.workflow(sdf.PbrWorkflowType.SPECULAR))

        workflow = PbrWorkflow()
        self.assertEqual(sdf.PbrWorkflowType.NONE, workflow.type())
        self.assertEqual('', workflow.albedo_map())
        self.assertEqual('', workflow.normal_map())
        self.assertEqual(sdf.NormalMapSpace.TANGENT, workflow.normal_map_type())
        self.assertEqual('', workflow.roughness_map())
        self.assertEqual('', workflow.metalness_map())
        self.assertEqual('', workflow.emissive_map())
        self.assertEqual('', workflow.light_map())
        self.assertEqual(0, workflow.light_map_tex_coord_set())
        self.assertAlmostEqual(0.5, workflow.roughness())
        self.assertAlmostEqual(0.5, workflow.metalness())
        self.assertEqual('', workflow.specular_map())
        self.assertEqual('', workflow.glossiness_map())
        self.assertAlmostEqual(0.0, workflow.glossiness())
        self.assertEqual('', workflow.environment_map())
        self.assertEqual('', workflow.ambient_occlusion_map())


    def test_assignment(self):
        pbr = Pbr()
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        pbr.set_workflow(workflow.type(), workflow)

        self.assertEqual(workflow, pbr.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr.workflow(sdf.PbrWorkflowType.SPECULAR))

        pbr2 = Pbr()
        pbr2 = pbr
        self.assertEqual(workflow, pbr2.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr2.workflow(sdf.PbrWorkflowType.SPECULAR))

        # metal workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        workflow.set_albedo_map("metal_albedo_map.png")
        workflow.set_normal_map("metal_normal_map.png", sdf.NormalMapSpace.TANGENT)
        workflow.set_environment_map("metal_env_map.png")
        workflow.set_ambient_occlusion_map("metal_ambient_occlusion_map.png")
        workflow.set_emissive_map("metal_emissive_map.png")
        workflow.set_light_map("metal_light_map.png", 3)
        workflow.set_roughness_map("roughness_map.png")
        workflow.set_metalness_map("metalness_map.png")
        workflow.set_roughness(0.8)
        workflow.set_metalness(0.3)

        workflow2 = workflow
        self.assertEqual(sdf.PbrWorkflowType.METAL, workflow2.type())
        self.assertEqual("metal_albedo_map.png", workflow2.albedo_map())
        self.assertEqual("metal_normal_map.png", workflow2.normal_map())
        self.assertEqual(sdf.NormalMapSpace.TANGENT, workflow2.normal_map_type())
        self.assertEqual("metal_env_map.png", workflow2.environment_map())
        self.assertEqual("metal_ambient_occlusion_map.png",
            workflow2.ambient_occlusion_map())
        self.assertEqual("metal_emissive_map.png", workflow2.emissive_map())
        self.assertEqual("metal_light_map.png", workflow2.light_map())
        self.assertEqual(3, workflow2.light_map_tex_coord_set())
        self.assertEqual("roughness_map.png", workflow2.roughness_map())
        self.assertEqual("metalness_map.png", workflow2.metalness_map())
        self.assertAlmostEqual(0.8, workflow2.roughness())
        self.assertAlmostEqual(0.3, workflow2.metalness())

        self.assertEqual('', workflow2.glossiness_map())
        self.assertEqual('', workflow2.specular_map())
        self.assertAlmostEqual(0.0, workflow2.glossiness())

        # specular workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.SPECULAR)
        workflow.set_albedo_map("specular_albedo_map.png")
        workflow.set_normal_map("specular_normal_map.png",
            sdf.NormalMapSpace.TANGENT)
        workflow.set_environment_map("specular_env_map.png")
        workflow.set_ambient_occlusion_map("specular_ambient_occlusion_map.png")
        workflow.set_emissive_map("specular_emissive_map.png")
        workflow.set_light_map("specular_light_map.png", 1)
        workflow.set_glossiness_map("glossiness_map.png")
        workflow.set_specular_map("specular_map.png")
        workflow.set_glossiness(0.1)

        workflow2 = PbrWorkflow()
        workflow2 = workflow
        self.assertEqual(sdf.PbrWorkflowType.SPECULAR, workflow2.type())
        self.assertEqual("specular_albedo_map.png", workflow2.albedo_map())
        self.assertEqual("specular_normal_map.png", workflow2.normal_map())
        self.assertEqual(sdf.NormalMapSpace.TANGENT, workflow2.normal_map_type())
        self.assertEqual("specular_env_map.png", workflow2.environment_map())
        self.assertEqual("specular_ambient_occlusion_map.png",
            workflow2.ambient_occlusion_map())
        self.assertEqual("specular_emissive_map.png", workflow2.emissive_map())
        self.assertEqual("specular_light_map.png", workflow2.light_map())
        self.assertEqual(1, workflow2.light_map_tex_coord_set())
        self.assertEqual("specular_map.png", workflow2.specular_map())
        self.assertEqual("glossiness_map.png", workflow2.glossiness_map())
        self.assertAlmostEqual(0.1, workflow2.glossiness())

        self.assertEqual('', workflow2.roughness_map())
        self.assertEqual('', workflow2.metalness_map())
        self.assertAlmostEqual(0.5, workflow2.roughness())
        self.assertAlmostEqual(0.5, workflow2.metalness())


    def test_copy_constructor(self):
        pbr = Pbr()
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        pbr.set_workflow(workflow.type(), workflow)

        self.assertEqual(workflow, pbr.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr.workflow(sdf.PbrWorkflowType.SPECULAR))

        pbr2 = Pbr(pbr)
        self.assertEqual(workflow, pbr2.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr2.workflow(sdf.PbrWorkflowType.SPECULAR))

        # metal workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        workflow.set_albedo_map("metal_albedo_map.png")
        workflow.set_normal_map("metal_normal_map.png")
        workflow.set_environment_map("metal_env_map.png")
        workflow.set_ambient_occlusion_map("metal_ambient_occlusion_map.png")
        workflow.set_emissive_map("metal_emissive_map.png")
        workflow.set_light_map("metal_light_map.png", 2)
        workflow.set_roughness_map("roughness_map.png")
        workflow.set_metalness_map("metalness_map.png")
        workflow.set_roughness(0.8)
        workflow.set_metalness(0.3)

        workflow2 = PbrWorkflow(workflow)
        self.assertEqual(sdf.PbrWorkflowType.METAL, workflow2.type())
        self.assertEqual("metal_albedo_map.png", workflow2.albedo_map())
        self.assertEqual("metal_normal_map.png", workflow2.normal_map())
        self.assertEqual(sdf.NormalMapSpace.TANGENT, workflow2.normal_map_type())
        self.assertEqual("metal_env_map.png", workflow2.environment_map())
        self.assertEqual("metal_ambient_occlusion_map.png",
            workflow2.ambient_occlusion_map())
        self.assertEqual("metal_emissive_map.png", workflow2.emissive_map())
        self.assertEqual("metal_light_map.png", workflow2.light_map())
        self.assertEqual(2, workflow2.light_map_tex_coord_set())
        self.assertEqual("roughness_map.png", workflow2.roughness_map())
        self.assertEqual("metalness_map.png", workflow2.metalness_map())
        self.assertAlmostEqual(0.8, workflow2.roughness())
        self.assertAlmostEqual(0.3, workflow2.metalness())

        self.assertEqual('', workflow2.glossiness_map())
        self.assertEqual('', workflow2.specular_map())
        self.assertAlmostEqual(0.0, workflow2.glossiness())

        # specular workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.SPECULAR)
        workflow.set_albedo_map("specular_albedo_map.png")
        workflow.set_normal_map("specular_normal_map.png")
        workflow.set_environment_map("specular_env_map.png")
        workflow.set_ambient_occlusion_map("specular_ambient_occlusion_map.png")
        workflow.set_emissive_map("specular_emissive_map.png")
        workflow.set_light_map("specular_light_map.png", 1)
        workflow.set_glossiness_map("glossiness_map.png")
        workflow.set_specular_map("specular_map.png")
        workflow.set_glossiness(0.1)

        workflow2 = PbrWorkflow(workflow)
        self.assertEqual(sdf.PbrWorkflowType.SPECULAR, workflow2.type())
        self.assertEqual("specular_albedo_map.png", workflow2.albedo_map())
        self.assertEqual("specular_normal_map.png", workflow2.normal_map())
        self.assertEqual(sdf.NormalMapSpace.TANGENT, workflow2.normal_map_type())
        self.assertEqual("specular_env_map.png", workflow2.environment_map())
        self.assertEqual("specular_ambient_occlusion_map.png",
            workflow2.ambient_occlusion_map())
        self.assertEqual("specular_emissive_map.png", workflow2.emissive_map())
        self.assertEqual("specular_light_map.png", workflow2.light_map())
        self.assertEqual(1, workflow2.light_map_tex_coord_set())
        self.assertEqual("specular_map.png", workflow2.specular_map())
        self.assertEqual("glossiness_map.png", workflow2.glossiness_map())
        self.assertAlmostEqual(0.1, workflow2.glossiness())

        self.assertEqual('', workflow2.roughness_map())
        self.assertEqual('', workflow2.metalness_map())
        self.assertAlmostEqual(0.5, workflow2.roughness())
        self.assertAlmostEqual(0.5, workflow2.metalness())


    def test_deepcopy(self):
        pbr = Pbr()
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        pbr.set_workflow(workflow.type(), workflow)

        self.assertEqual(workflow, pbr.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr.workflow(sdf.PbrWorkflowType.SPECULAR))

        pbr2 = copy.deepcopy(pbr)
        self.assertEqual(workflow, pbr2.workflow(sdf.PbrWorkflowType.METAL))
        self.assertEqual(None, pbr2.workflow(sdf.PbrWorkflowType.SPECULAR))
        # metal workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        workflow.set_albedo_map("metal_albedo_map.png")
        workflow.set_normal_map("metal_normal_map.png")
        workflow.set_environment_map("metal_env_map.png")
        workflow.set_ambient_occlusion_map("metal_ambient_occlusion_map.png")
        workflow.set_emissive_map("metal_emissive_map.png")
        workflow.set_light_map("metal_light_map.png", 1)
        workflow.set_roughness_map("roughness_map.png")
        workflow.set_metalness_map("metalness_map.png")
        workflow.set_roughness(0.8)
        workflow.set_metalness(0.3)

        workflow2 = copy.deepcopy(workflow)
        self.assertEqual(sdf.PbrWorkflowType.METAL, workflow2.type())
        self.assertEqual("metal_albedo_map.png", workflow2.albedo_map())
        self.assertEqual("metal_normal_map.png", workflow2.normal_map())
        self.assertEqual("metal_env_map.png", workflow2.environment_map())
        self.assertEqual("metal_ambient_occlusion_map.png",
            workflow2.ambient_occlusion_map())
        self.assertEqual("metal_emissive_map.png", workflow2.emissive_map())
        self.assertEqual("metal_light_map.png", workflow2.light_map())
        self.assertEqual(1, workflow2.light_map_tex_coord_set())
        self.assertEqual("roughness_map.png", workflow2.roughness_map())
        self.assertEqual("metalness_map.png", workflow2.metalness_map())
        self.assertAlmostEqual(0.8, workflow2.roughness())
        self.assertAlmostEqual(0.3, workflow2.metalness())

        self.assertEqual('', workflow2.glossiness_map())
        self.assertEqual('', workflow2.specular_map())
        self.assertAlmostEqual(0.0, workflow2.glossiness())

        # specular workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.SPECULAR)
        workflow.set_albedo_map("specular_albedo_map.png")
        workflow.set_normal_map("specular_normal_map.png")
        workflow.set_environment_map("specular_env_map.png")
        workflow.set_ambient_occlusion_map("specular_ambient_occlusion_map.png")
        workflow.set_emissive_map("specular_emissive_map.png")
        workflow.set_light_map("specular_light_map.png", 2)
        workflow.set_glossiness_map("glossiness_map.png")
        workflow.set_specular_map("specular_map.png")
        workflow.set_glossiness(0.1)

        workflow2 = copy.deepcopy(workflow)
        self.assertEqual(sdf.PbrWorkflowType.SPECULAR, workflow2.type())
        self.assertEqual("specular_albedo_map.png", workflow2.albedo_map())
        self.assertEqual("specular_normal_map.png", workflow2.normal_map())
        self.assertEqual("specular_env_map.png", workflow2.environment_map())
        self.assertEqual("specular_ambient_occlusion_map.png",
            workflow2.ambient_occlusion_map())
        self.assertEqual("specular_emissive_map.png", workflow2.emissive_map())
        self.assertEqual("specular_light_map.png", workflow2.light_map())
        self.assertEqual(2, workflow2.light_map_tex_coord_set())
        self.assertEqual("specular_map.png", workflow2.specular_map())
        self.assertEqual("glossiness_map.png", workflow2.glossiness_map())
        self.assertAlmostEqual(0.1, workflow2.glossiness())

        self.assertEqual('', workflow2.roughness_map())
        self.assertEqual('', workflow2.metalness_map())
        self.assertAlmostEqual(0.5, workflow2.roughness())
        self.assertAlmostEqual(0.5, workflow2.metalness())


    def test_set(self):
        # metal workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.METAL)
        self.assertEqual(sdf.PbrWorkflowType.METAL, workflow.type())

        workflow.set_albedo_map("metal_albedo_map.png")
        self.assertEqual("metal_albedo_map.png", workflow.albedo_map())

        workflow.set_normal_map("metal_normal_map.png")
        self.assertEqual("metal_normal_map.png", workflow.normal_map())

        workflow.set_environment_map("metal_env_map.png")
        self.assertEqual("metal_env_map.png", workflow.environment_map())

        workflow.set_ambient_occlusion_map("metal_ambient_occlusion_map.png")
        self.assertEqual("metal_ambient_occlusion_map.png",
            workflow.ambient_occlusion_map())

        workflow.set_emissive_map("metal_emissive_map.png")
        self.assertEqual("metal_emissive_map.png", workflow.emissive_map())

        workflow.set_light_map("metal_light_map.png", 1)
        self.assertEqual("metal_light_map.png", workflow.light_map())
        self.assertEqual(1, workflow.light_map_tex_coord_set())

        workflow.set_roughness_map("roughness_map.png")
        self.assertEqual("roughness_map.png", workflow.roughness_map())

        workflow.set_metalness_map("metalness_map.png")
        self.assertEqual("metalness_map.png", workflow.metalness_map())

        workflow.set_roughness(0.8)
        self.assertAlmostEqual(0.8, workflow.roughness())

        workflow.set_metalness(0.3)
        self.assertAlmostEqual(0.3, workflow.metalness())

        self.assertEqual('', workflow.glossiness_map())
        self.assertEqual('', workflow.specular_map())
        self.assertAlmostEqual(0.0, workflow.glossiness())

        pbr = Pbr()
        pbr.set_workflow(workflow.type(), workflow)
        self.assertEqual(workflow, pbr.workflow(workflow.type()))

        empty = PbrWorkflow()
        self.assertNotEqual(empty, pbr.workflow(workflow.type()))

        # specular workflow
        workflow = PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflowType.SPECULAR)
        self.assertEqual(sdf.PbrWorkflowType.SPECULAR, workflow.type())

        workflow.set_albedo_map("specular_albedo_map.png")
        self.assertEqual("specular_albedo_map.png", workflow.albedo_map())

        workflow.set_normal_map("specular_normal_map.png",
            sdf.NormalMapSpace.OBJECT)
        self.assertEqual("specular_normal_map.png", workflow.normal_map())
        self.assertEqual(sdf.NormalMapSpace.OBJECT, workflow.normal_map_type())

        workflow.set_environment_map("specular_env_map.png")
        self.assertEqual("specular_env_map.png", workflow.environment_map())

        workflow.set_ambient_occlusion_map("specular_ambient_occlusion_map.png")
        self.assertEqual("specular_ambient_occlusion_map.png",
            workflow.ambient_occlusion_map())

        workflow.set_emissive_map("specular_emissive_map.png")
        self.assertEqual("specular_emissive_map.png", workflow.emissive_map())

        workflow.set_light_map("specular_light_map.png", 1)
        self.assertEqual("specular_light_map.png", workflow.light_map())
        self.assertEqual(1, workflow.light_map_tex_coord_set())

        workflow.set_glossiness_map("glossiness_map.png")
        self.assertEqual("glossiness_map.png", workflow.glossiness_map())

        workflow.set_specular_map("specular_map.png")
        self.assertEqual("specular_map.png", workflow.specular_map())

        workflow.set_glossiness(0.1)
        self.assertAlmostEqual(0.1, workflow.glossiness())

        self.assertEqual('', workflow.roughness_map())
        self.assertEqual('', workflow.metalness_map())
        self.assertAlmostEqual(0.5, workflow.roughness())
        self.assertAlmostEqual(0.5, workflow.metalness())

        pbr = Pbr()
        pbr.set_workflow(workflow.type(), workflow)
        self.assertEqual(workflow, pbr.workflow(workflow.type()))

        empty = PbrWorkflow()
        self.assertNotEqual(empty, pbr.workflow(workflow.type()))


if __name__ == '__main__':
    unittest.main()
