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
from gz_test_deps.math import Pose3d, Color
from gz_test_deps.sdformat import (Geometry, Material, Visual, Plugin,
                                   SDFErrorsException)
import gz_test_deps.sdformat as sdf
import unittest
import math

class VisualTEST(unittest.TestCase):

    def test_default_construction(self):
        visual = Visual()
        self.assertTrue(not visual.name())

        visual.set_name("test_visual")
        self.assertEqual(visual.name(), "test_visual")

        visual.set_cast_shadows(False)
        self.assertFalse(visual.cast_shadows())

        # check default transparency is 0
        self.assertEqual(0.0, visual.transparency())

        visual.set_transparency(0.34)
        self.assertAlmostEqual(0.34, visual.transparency())

        self.assertEqual(Pose3d.ZERO, visual.raw_pose())
        self.assertTrue(not visual.pose_relative_to())

        semanticPose = visual.semantic_pose()
        self.assertEqual(visual.raw_pose(), semanticPose.raw_pose())
        self.assertEqual('', semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        visual.set_raw_pose(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2))
        self.assertEqual(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2),
            visual.raw_pose())

        visual.set_pose_relative_to("link")
        self.assertEqual("link", visual.pose_relative_to())

        semanticPose = visual.semantic_pose()
        self.assertEqual(visual.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("link", semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        self.assertNotEqual(None, visual.geometry())
        self.assertEqual(sdf.GeometryType.EMPTY, visual.geometry().type())
        self.assertEqual(None, visual.geometry().box_shape())
        self.assertEqual(None, visual.geometry().cone_shape())
        self.assertEqual(None, visual.geometry().cylinder_shape())
        self.assertEqual(None, visual.geometry().plane_shape())
        self.assertEqual(None, visual.geometry().sphere_shape())

        self.assertEqual(None, visual.material())

        # visibility flags
        self.assertEqual(4294967295, visual.visibility_flags())
        visual.set_visibility_flags(1)
        self.assertEqual(1, visual.visibility_flags())


    def test_copy_construction(self):
        visual = Visual()
        visual.set_name("test_visual")
        visual.set_cast_shadows(False)
        visual.set_transparency(0.345)
        visual.set_raw_pose(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2))
        visual.set_visibility_flags(2)

        visual.set_pose_relative_to("link")

        mat = Material()
        mat.set_ambient(Color(0.1, 0.1, 0.1))
        visual.set_material(mat)

        visual2 = Visual(visual)

        self.assertEqual("test_visual", visual.name())
        self.assertFalse(visual.cast_shadows())
        self.assertAlmostEqual(0.345, visual.transparency())
        self.assertEqual(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2),
            visual.raw_pose())
        self.assertEqual("link", visual.pose_relative_to())
        material = visual.material()
        self.assertTrue(None != visual.material())
        self.assertEqual(mat.ambient(), visual.material().ambient())
        self.assertEqual(2, visual.visibility_flags())

        self.assertEqual("test_visual", visual2.name())
        self.assertFalse(visual2.cast_shadows())
        self.assertAlmostEqual(0.345, visual2.transparency())
        self.assertEqual(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2),
            visual2.raw_pose())
        self.assertEqual("link", visual2.pose_relative_to())
        self.assertTrue(None != visual2.material())
        self.assertEqual(mat.ambient(), visual2.material().ambient())
        self.assertEqual(2, visual2.visibility_flags())


    def test_deepcopy(self):
        visual = Visual()
        visual.set_name("test_visual")
        visual.set_cast_shadows(False)
        visual.set_transparency(0.345)
        visual.set_raw_pose(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2))
        visual.set_visibility_flags(2)

        visual.set_pose_relative_to("link")

        mat = Material()
        mat.set_ambient(Color(0.1, 0.1, 0.1))
        visual.set_material(mat)

        visual2 = copy.deepcopy(visual)

        self.assertEqual("test_visual", visual.name())
        self.assertFalse(visual.cast_shadows())
        self.assertAlmostEqual(0.345, visual.transparency())
        self.assertEqual(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2),
            visual.raw_pose())
        self.assertEqual("link", visual.pose_relative_to())
        self.assertTrue(None != visual.material())
        self.assertEqual(mat.ambient(), visual.material().ambient())
        self.assertEqual(2, visual.visibility_flags())

        self.assertEqual("test_visual", visual2.name())
        self.assertFalse(visual2.cast_shadows())
        self.assertAlmostEqual(0.345, visual2.transparency())
        self.assertEqual(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2),
            visual2.raw_pose())
        self.assertEqual("link", visual2.pose_relative_to())
        self.assertTrue(None != visual2.material())
        self.assertEqual(mat.ambient(), visual2.material().ambient())
        self.assertEqual(2, visual2.visibility_flags())


    def test_assignment(self):
        visual = Visual()
        visual.set_name("test_visual")
        visual.set_cast_shadows(False)
        visual.set_transparency(0.345)
        visual.set_raw_pose(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2))
        visual.set_visibility_flags(2)

        visual.set_pose_relative_to("link")

        mat = Material()
        mat.set_ambient(Color(0.1, 0.1, 0.1))
        visual.set_material(mat)

        visual2 = visual

        self.assertEqual("test_visual", visual2.name())
        self.assertFalse(visual2.cast_shadows())
        self.assertAlmostEqual(0.345, visual2.transparency())
        self.assertEqual(Pose3d(0, -20, 30, math.pi/2, -math.pi, math.pi/2),
            visual2.raw_pose())
        self.assertEqual("link", visual2.pose_relative_to())
        self.assertTrue(None != visual2.material())
        self.assertEqual(mat.ambient(), visual2.material().ambient())
        self.assertEqual(2, visual2.visibility_flags())


    def test_set_geometry(self):
        visual = Visual()
        self.assertTrue(not visual.name())

        geometry = Geometry()
        geometry.set_type(sdf.GeometryType.BOX)

        visual.set_geometry(geometry)

        self.assertNotEqual(None, visual.geometry())
        self.assertEqual(sdf.GeometryType.BOX, visual.geometry().type())


    def test_set_material(self):
        visual = Visual()
        self.assertTrue(not visual.name())

        material = Material()
        material.set_ambient(Color(0, 0.5, 0))
        material.set_diffuse(Color(1, 0, 0))
        material.set_specular(Color(0., 0.1, 0.9))

        visual.set_material(material)

        self.assertNotEqual(None, visual.material())
        self.assertEqual(Color(0, 0.5, 0), visual.material().ambient())
        self.assertEqual(Color(1, 0, 0), visual.material().diffuse())
        self.assertEqual(Color(0., 0.1, 0.9),
            visual.material().specular())


    def test_set_laser_retro(self):
        visual = Visual()
        self.assertTrue(not visual.name())

        visual.set_laser_retro(150)

        self.assertTrue(visual.has_laser_retro())
        self.assertEqual(150, visual.laser_retro())


    def test_plugins(self):
        visual = Visual()
        self.assertEqual(0, len(visual.plugins()))

        plugin = Plugin()
        plugin.set_name("name1")
        plugin.set_filename("filename1")

        visual.add_plugin(plugin)
        self.assertEqual(1, len(visual.plugins()))

        plugin.set_name("name2")
        visual.add_plugin(plugin)
        self.assertEqual(2, len(visual.plugins()))

        self.assertEqual("name1", visual.plugins()[0].name())
        self.assertEqual("name2", visual.plugins()[1].name())

        visual.clear_plugins()
        self.assertEqual(0, len(visual.plugins()))


if __name__ == '__main__':
    unittest.main()
