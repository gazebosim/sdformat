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
from gz.math import Angle, Color, Pose3d, Vector3d
from sdformat import Light, SDFErrorsException
import sdformat as sdf
import math
import unittest


class LightColor(unittest.TestCase):

    def test_default_construction(self):
        light = Light()
        self.assertEqual(sdf.LightType.POINT, light.type())
        self.assertFalse(light.name())

        light.set_name("test_light")
        self.assertEqual("test_light", light.name())

        self.assertEqual(Pose3d.ZERO, light.raw_pose())
        self.assertFalse(light.pose_relative_to())

        semanticPose = light.semantic_pose()
        self.assertEqual(light.raw_pose(), semanticPose.raw_pose())
        self.assertFalse(semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        light.set_raw_pose(Pose3d(1, 2, 3, 0, 0, math.pi))
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, math.pi), light.raw_pose())

        light.set_pose_relative_to("world")
        self.assertEqual("world", light.pose_relative_to())

        semanticPose = light.semantic_pose()
        self.assertEqual(light.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("world", semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        self.assertTrue(light.light_on())
        light.set_light_on(False)
        self.assertFalse(light.light_on())

        self.assertTrue(light.visualize())
        light.set_visualize(False)
        self.assertFalse(light.visualize())

        self.assertFalse(light.cast_shadows())
        light.set_cast_shadows(True)
        self.assertTrue(light.cast_shadows())

        self.assertEqual(Color(0, 0, 0, 1), light.diffuse())
        light.set_diffuse(Color(0.1,  0.2,  0.3,  1.0))
        self.assertEqual(Color(0.1,  0.2,  0.3,  1), light.diffuse())

        self.assertEqual(Color(0, 0, 0, 1), light.specular())
        light.set_specular(Color(0.4,  0.6,  0.7,  1.0))
        self.assertEqual(Color(0.4,  0.6,  0.7,  1), light.specular())

        self.assertAlmostEqual(10.0, light.attenuation_range())
        light.set_attenuation_range(1.2)
        self.assertAlmostEqual(1.2, light.attenuation_range())

        self.assertAlmostEqual(1.0, light.linear_attenuation_factor())
        light.set_linear_attenuation_factor(0.2)
        self.assertAlmostEqual(0.2, light.linear_attenuation_factor())

        self.assertAlmostEqual(1.0, light.constant_attenuation_factor())
        light.set_constant_attenuation_factor(0.4)
        self.assertAlmostEqual(0.4, light.constant_attenuation_factor())

        self.assertAlmostEqual(0.0, light.quadratic_attenuation_factor())
        light.set_quadratic_attenuation_factor(1.1)
        self.assertAlmostEqual(1.1, light.quadratic_attenuation_factor())

        self.assertEqual(Vector3d(0, 0, -1), light.direction())
        light.set_direction(Vector3d(0.4, 0.2, 0))
        self.assertEqual(Vector3d(0.4, 0.2, 0), light.direction())

        self.assertEqual(Angle(0.0), light.spot_inner_angle())
        light.set_spot_inner_angle(Angle(1.4))
        self.assertEqual(Angle(1.4), light.spot_inner_angle())

        self.assertEqual(Angle(0.0), light.spot_outer_angle())
        light.set_spot_outer_angle(Angle(0.2))
        self.assertEqual(Angle(0.2), light.spot_outer_angle())

        self.assertAlmostEqual(0.0, light.spot_falloff())
        light.set_spot_falloff(4.3)
        self.assertAlmostEqual(4.3, light.spot_falloff())

        self.assertAlmostEqual(1.0, light.intensity())
        light.set_intensity(0.3)
        self.assertAlmostEqual(0.3, light.intensity())

    def test_copy_construction(self):
        light = Light()
        light.set_name("test_copy_light")
        light.set_type(sdf.LightType.DIRECTIONAL)
        light.set_raw_pose(Pose3d(3, 2, 1, 0, math.pi, 0))
        light.set_pose_relative_to("ground_plane")
        light.set_cast_shadows(True)
        light.set_light_on(False)
        light.set_visualize(False)
        light.set_diffuse(Color(0.4,  0.5,  0.6,  1.0))
        light.set_specular(Color(0.8,  0.9,  0.1,  1.0))
        light.set_attenuation_range(3.2)
        light.set_linear_attenuation_factor(0.1)
        light.set_constant_attenuation_factor(0.5)
        light.set_quadratic_attenuation_factor(0.01)
        light.set_direction(Vector3d(0.1, 0.2, 1))
        light.set_spot_inner_angle(Angle(1.9))
        light.set_spot_outer_angle(Angle(3.3))
        light.set_spot_falloff(0.9)
        light.set_intensity(1.7)

        light2 = Light(light)
        self.assertEqual("test_copy_light", light2.name())
        self.assertEqual(sdf.LightType.DIRECTIONAL, light2.type())
        self.assertEqual(Pose3d(3, 2, 1, 0, math.pi, 0), light2.raw_pose())
        self.assertEqual("ground_plane", light2.pose_relative_to())
        self.assertTrue(light2.cast_shadows())
        self.assertFalse(light2.light_on())
        self.assertFalse(light2.visualize())
        self.assertEqual(Color(0.4,  0.5,  0.6,  1), light2.diffuse())
        self.assertEqual(Color(0.8,  0.9,  0.1,  1), light2.specular())
        self.assertAlmostEqual(3.2, light2.attenuation_range())
        self.assertAlmostEqual(0.1, light2.linear_attenuation_factor())
        self.assertAlmostEqual(0.5, light2.constant_attenuation_factor())
        self.assertAlmostEqual(0.01, light2.quadratic_attenuation_factor())
        self.assertEqual(Vector3d(0.1, 0.2, 1), light2.direction())
        self.assertEqual(Angle(1.9), light2.spot_inner_angle())
        self.assertEqual(Angle(3.3), light2.spot_outer_angle())
        self.assertAlmostEqual(0.9, light2.spot_falloff())
        self.assertAlmostEqual(1.7, light2.intensity())

    def test_deepcopy(self):
        light = Light()
        light.set_name("test_light_assignment")
        light.set_type(sdf.LightType.DIRECTIONAL)
        light.set_raw_pose(Pose3d(3, 2, 1, 0, math.pi, 0))
        light.set_pose_relative_to("ground_plane")
        light.set_cast_shadows(True)
        light.set_light_on(False)
        light.set_visualize(False)
        light.set_diffuse(Color(0.4,  0.5,  0.6,  1.0))
        light.set_specular(Color(0.8,  0.9,  0.1,  1.0))
        light.set_attenuation_range(3.2)
        light.set_linear_attenuation_factor(0.1)
        light.set_constant_attenuation_factor(0.5)
        light.set_quadratic_attenuation_factor(0.01)
        light.set_direction(Vector3d(0.1, 0.2, 1))
        light.set_spot_inner_angle(Angle(1.9))
        light.set_spot_outer_angle(Angle(3.3))
        light.set_spot_falloff(0.9)
        light.set_intensity(1.7)

        light2 = copy.deepcopy(light)
        self.assertEqual("test_light_assignment", light2.name())
        self.assertEqual(sdf.LightType.DIRECTIONAL, light2.type())
        self.assertEqual(Pose3d(3, 2, 1, 0, math.pi, 0), light2.raw_pose())
        self.assertEqual("ground_plane", light2.pose_relative_to())
        self.assertTrue(light2.cast_shadows())
        self.assertFalse(light2.light_on())
        self.assertFalse(light2.visualize())
        self.assertEqual(Color(0.4,  0.5,  0.6,  1), light2.diffuse())
        self.assertEqual(Color(0.8,  0.9,  0.1,  1), light2.specular())
        self.assertAlmostEqual(3.2, light2.attenuation_range())
        self.assertAlmostEqual(0.1, light2.linear_attenuation_factor())
        self.assertAlmostEqual(0.5, light2.constant_attenuation_factor())
        self.assertAlmostEqual(0.01, light2.quadratic_attenuation_factor())
        self.assertEqual(Vector3d(0.1, 0.2, 1), light2.direction())
        self.assertEqual(Angle(1.9), light2.spot_inner_angle())
        self.assertEqual(Angle(3.3), light2.spot_outer_angle())
        self.assertAlmostEqual(0.9, light2.spot_falloff())
        self.assertAlmostEqual(1.7, light2.intensity())

    def test_spot_light_negative_values(self):
        light = Light()
        light.set_spot_falloff(-1.0)
        self.assertAlmostEqual(0.0, light.spot_falloff())

        light.set_spot_inner_angle(Angle(-1.0))
        self.assertAlmostEqual(0.0, light.spot_inner_angle().radian())

        light.set_spot_outer_angle(Angle(-2.0))
        self.assertAlmostEqual(0.0, light.spot_outer_angle().radian())

    def test_attenuation_clamp(self):
        light = Light()

        light.set_linear_attenuation_factor(-1.0)
        self.assertAlmostEqual(0.0, light.linear_attenuation_factor())

        light.set_linear_attenuation_factor(20.0)
        self.assertAlmostEqual(1.0, light.linear_attenuation_factor())

        light.set_constant_attenuation_factor(-1.0)
        self.assertAlmostEqual(0.0, light.constant_attenuation_factor())

        light.set_constant_attenuation_factor(20.0)
        self.assertAlmostEqual(1.0, light.constant_attenuation_factor())

        light.set_quadratic_attenuation_factor(-1.0)
        self.assertAlmostEqual(0.0, light.quadratic_attenuation_factor())


if __name__ == '__main__':
    unittest.main()
