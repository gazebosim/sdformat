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
from gz_test_deps.math import Angle, Pose3d, Vector2d
import math
from gz_test_deps.sdformat import Camera
import gz_test_deps.sdformat as sdf
import unittest

class CameraTEST(unittest.TestCase):

  def test_default_construction(self):
      cam = Camera()
      self.assertFalse(cam.name())
      cam.set_name("my_camera")
      self.assertEqual("my_camera", cam.name())

      self.assertFalse(cam.triggered())
      cam.set_triggered(True)
      self.assertTrue(cam.triggered())

      self.assertFalse(cam.trigger_topic())
      cam.set_trigger_topic("my_camera/trigger")
      self.assertEqual("my_camera/trigger", cam.trigger_topic())

      self.assertEqual("", cam.camera_info_topic());
      cam.set_camera_info_topic("/camera/camera_info");
      self.assertEqual("/camera/camera_info", cam.camera_info_topic());

      self.assertAlmostEqual(1.047, cam.horizontal_fov().radian())
      cam.set_horizontal_fov(Angle(1.45))
      self.assertAlmostEqual(1.45, cam.horizontal_fov().radian())

      self.assertEqual(320, cam.image_width())
      cam.set_image_width(123)
      self.assertEqual(123, cam.image_width())

      self.assertEqual(240, cam.image_height())
      cam.set_image_height(125)
      self.assertEqual(125, cam.image_height())

      self.assertEqual(sdf.PixelFormatType.RGB_INT8, cam.pixel_format())
      cam.set_pixel_format(sdf.PixelFormatType.L_INT8)
      self.assertEqual(sdf.PixelFormatType.L_INT8 , cam.pixel_format())

      self.assertEqual(4, cam.anti_aliasing_value())
      cam.set_anti_aliasing_value(8)
      self.assertEqual(8, cam.anti_aliasing_value())

      self.assertAlmostEqual(0.1, cam.depth_near_clip())
      self.assertFalse(cam.has_depth_near_clip())
      cam.set_depth_near_clip(0.2)
      self.assertAlmostEqual(0.2, cam.depth_near_clip())
      self.assertTrue(cam.has_depth_near_clip())

      self.assertAlmostEqual(10.0, cam.depth_far_clip())
      self.assertFalse(cam.has_depth_far_clip())
      cam.set_depth_far_clip(20.2)
      self.assertAlmostEqual(20.2, cam.depth_far_clip())
      self.assertTrue(cam.has_depth_far_clip())

      self.assertAlmostEqual(0.1, cam.near_clip())
      cam.set_near_clip(0.2)
      self.assertAlmostEqual(0.2, cam.near_clip())

      self.assertAlmostEqual(100, cam.far_clip())
      cam.set_far_clip(200.2)
      self.assertAlmostEqual(200.2, cam.far_clip())

      self.assertEqual("semantic", cam.segmentation_type())
      self.assertFalse(cam.has_segmentation_type())
      cam.set_segmentation_type("panoptic")
      self.assertTrue(cam.has_segmentation_type())
      self.assertEqual("panoptic", cam.segmentation_type())
      cam.set_has_segmentation_type(False)
      self.assertFalse(cam.has_segmentation_type())

      self.assertEqual("2d", cam.bounding_box_type())
      self.assertFalse(cam.has_bounding_box_type())
      cam.set_bounding_box_type("3d")
      self.assertTrue(cam.has_bounding_box_type())
      self.assertEqual("3d", cam.bounding_box_type())
      cam.set_has_bounding_box_type(False)
      self.assertFalse(cam.has_bounding_box_type())

      self.assertFalse(cam.save_frames())
      cam.set_save_frames(True)
      self.assertTrue(cam.save_frames())

      self.assertEqual("", cam.save_frames_path())
      cam.set_save_frames_path("/tmp")
      self.assertEqual("/tmp", cam.save_frames_path())

      self.assertAlmostEqual(0.0, cam.distortion_k1())
      cam.set_distortion_k1(0.1)
      self.assertAlmostEqual(0.1, cam.distortion_k1())

      self.assertAlmostEqual(0.0, cam.distortion_k2())
      cam.set_distortion_k2(0.2)
      self.assertAlmostEqual(0.2, cam.distortion_k2())

      self.assertAlmostEqual(0.0, cam.distortion_k3())
      cam.set_distortion_k3(0.2)
      self.assertAlmostEqual(0.2, cam.distortion_k3())

      self.assertAlmostEqual(0.0, cam.distortion_p1())
      cam.set_distortion_p1(0.2)
      self.assertAlmostEqual(0.2, cam.distortion_p1())

      self.assertAlmostEqual(0.0, cam.distortion_p2())
      cam.set_distortion_p2(0.2)
      self.assertAlmostEqual(0.2, cam.distortion_p2())

      self.assertEqual(Vector2d(0.5, 0.5), cam.distortion_center())
      cam.set_distortion_center(Vector2d(0.1, 0.2))
      self.assertEqual(Vector2d(0.1, 0.2), cam.distortion_center())

      self.assertEqual(Pose3d.ZERO, cam.raw_pose())
      cam.set_raw_pose(Pose3d(1, 2, 3, 0, 0, 0))
      self.assertEqual(Pose3d(1, 2, 3, 0, 0, 0), cam.raw_pose())

      self.assertFalse(cam.pose_relative_to())
      cam.set_pose_relative_to("/frame")
      self.assertEqual("/frame", cam.pose_relative_to())

      self.assertFalse(cam.optical_frame_id());
      cam.set_optical_frame_id("/optical_frame");
      self.assertEqual("/optical_frame", cam.optical_frame_id());

      self.assertEqual("stereographic", cam.lens_type())
      cam.set_lens_type("custom")
      self.assertEqual("custom", cam.lens_type())

      self.assertTrue(cam.lens_scale_to_hfov())
      cam.set_lens_scale_to_hfov(False)
      self.assertFalse(cam.lens_scale_to_hfov())

      self.assertAlmostEqual(1.0, cam.lens_c1())
      cam.set_lens_c1(2.1)
      self.assertAlmostEqual(2.1, cam.lens_c1())

      self.assertAlmostEqual(1.0, cam.lens_c2())
      cam.set_lens_c2(1.2)
      self.assertAlmostEqual(1.2, cam.lens_c2())

      self.assertAlmostEqual(0.0, cam.lens_c3())
      cam.set_lens_c3(6.5)
      self.assertAlmostEqual(6.5, cam.lens_c3())

      self.assertAlmostEqual(1.0, cam.lens_focal_length())
      cam.set_lens_focal_length(10.3)
      self.assertAlmostEqual(10.3, cam.lens_focal_length())

      self.assertEqual("tan", cam.lens_function())
      cam.set_lens_function("sin")
      self.assertEqual("sin", cam.lens_function())

      self.assertAlmostEqual(math.pi / 2, cam.lens_cutoff_angle().radian())
      cam.set_lens_cutoff_angle(Angle(0.456))
      self.assertAlmostEqual(0.456, cam.lens_cutoff_angle().radian())

      self.assertEqual(256, cam.lens_environment_texture_size())
      cam.set_lens_environment_texture_size(512)
      self.assertEqual(512, cam.lens_environment_texture_size())

      self.assertAlmostEqual(277, cam.lens_intrinsics_fx())
      cam.set_lens_intrinsics_fx(132)
      self.assertAlmostEqual(132, cam.lens_intrinsics_fx())

      self.assertAlmostEqual(277, cam.lens_intrinsics_fy())
      cam.set_lens_intrinsics_fy(456)
      self.assertAlmostEqual(456, cam.lens_intrinsics_fy())

      self.assertAlmostEqual(160, cam.lens_intrinsics_cx())
      cam.set_lens_intrinsics_cx(254)
      self.assertAlmostEqual(254, cam.lens_intrinsics_cx())

      self.assertAlmostEqual(120, cam.lens_intrinsics_cy())
      cam.set_lens_intrinsics_cy(123)
      self.assertAlmostEqual(123, cam.lens_intrinsics_cy())

      self.assertAlmostEqual(0.0, cam.lens_intrinsics_skew())
      cam.set_lens_intrinsics_skew(2.3)
      self.assertAlmostEqual(2.3, cam.lens_intrinsics_skew())

      self.assertTrue(cam.has_lens_intrinsics())
      self.assertFalse(cam.has_lens_projection())

      self.assertEqual(4294967295, cam.visibility_mask())
      cam.set_visibility_mask(123)
      self.assertEqual(123, cam.visibility_mask())

      # Copy Constructor
      cam2 = Camera(cam)
      self.assertEqual(cam, cam2)

      cam3 = cam
      self.assertEqual(cam, cam3)

      cam = cam2
      self.assertEqual(cam2, cam)

      cam2 = copy.deepcopy(cam)
      self.assertEqual(cam, cam2)

      #inequality
      cam6 = Camera()
      self.assertNotEqual(cam2, cam6)


if __name__ == '__main__':
    unittest.main()
