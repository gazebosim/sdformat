/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include "sdf/Camera.hh"

/////////////////////////////////////////////////
TEST(DOMCamera, Construction)
{
  sdf::Camera cam;
  EXPECT_TRUE(cam.Name().empty());
  cam.SetName("my_camera");
  EXPECT_EQ("my_camera", cam.Name());

  EXPECT_FALSE(cam.Triggered());
  cam.SetTriggered(true);
  EXPECT_TRUE(cam.Triggered());

  EXPECT_TRUE(cam.TriggerTopic().empty());
  cam.SetTriggerTopic("my_camera/trigger");
  EXPECT_EQ("my_camera/trigger", cam.TriggerTopic());

  EXPECT_EQ("", cam.CameraInfoTopic());
  cam.SetCameraInfoTopic("/camera/camera_info");
  EXPECT_EQ("/camera/camera_info", cam.CameraInfoTopic());

  EXPECT_DOUBLE_EQ(1.047, cam.HorizontalFov().Radian());
  cam.SetHorizontalFov(1.45);
  EXPECT_DOUBLE_EQ(1.45, cam.HorizontalFov().Radian());

  EXPECT_EQ(320u, cam.ImageWidth());
  cam.SetImageWidth(123);
  EXPECT_EQ(123u, cam.ImageWidth());

  EXPECT_EQ(240u, cam.ImageHeight());
  cam.SetImageHeight(125);
  EXPECT_EQ(125u, cam.ImageHeight());

  EXPECT_EQ(sdf::PixelFormatType::RGB_INT8, cam.PixelFormat());
  cam.SetPixelFormat(sdf::PixelFormatType::L_INT8);
  EXPECT_EQ(sdf::PixelFormatType::L_INT8 , cam.PixelFormat());

  EXPECT_EQ(4u, cam.AntiAliasingValue());
  cam.SetAntiAliasingValue(8);
  EXPECT_EQ(8u, cam.AntiAliasingValue());

  EXPECT_DOUBLE_EQ(0.1, cam.DepthNearClip());
  EXPECT_FALSE(cam.HasDepthNearClip());
  cam.SetDepthNearClip(0.2);
  EXPECT_DOUBLE_EQ(0.2, cam.DepthNearClip());
  EXPECT_TRUE(cam.HasDepthNearClip());

  EXPECT_DOUBLE_EQ(10.0, cam.DepthFarClip());
  EXPECT_FALSE(cam.HasDepthFarClip());
  cam.SetDepthFarClip(20.2);
  EXPECT_DOUBLE_EQ(20.2, cam.DepthFarClip());
  EXPECT_TRUE(cam.HasDepthFarClip());

  EXPECT_DOUBLE_EQ(0.1, cam.NearClip());
  cam.SetNearClip(0.2);
  EXPECT_DOUBLE_EQ(0.2, cam.NearClip());

  EXPECT_DOUBLE_EQ(100, cam.FarClip());
  cam.SetFarClip(200.2);
  EXPECT_DOUBLE_EQ(200.2, cam.FarClip());

  EXPECT_EQ("semantic", cam.SegmentationType());
  EXPECT_FALSE(cam.HasSegmentationType());
  cam.SetSegmentationType("panoptic");
  EXPECT_TRUE(cam.HasSegmentationType());
  EXPECT_EQ("panoptic", cam.SegmentationType());
  cam.SetHasSegmentationType(false);
  EXPECT_FALSE(cam.HasSegmentationType());

  EXPECT_EQ("2d", cam.BoundingBoxType());
  EXPECT_FALSE(cam.HasBoundingBoxType());
  cam.SetBoundingBoxType("3d");
  EXPECT_TRUE(cam.HasBoundingBoxType());
  EXPECT_EQ("3d", cam.BoundingBoxType());
  cam.SetHasBoundingBoxType(false);
  EXPECT_FALSE(cam.HasBoundingBoxType());

  EXPECT_FALSE(cam.SaveFrames());
  cam.SetSaveFrames(true);
  EXPECT_TRUE(cam.SaveFrames());

  EXPECT_EQ("", cam.SaveFramesPath());
  cam.SetSaveFramesPath("/tmp");
  EXPECT_EQ("/tmp", cam.SaveFramesPath());

  EXPECT_DOUBLE_EQ(0.0, cam.DistortionK1());
  cam.SetDistortionK1(0.1);
  EXPECT_DOUBLE_EQ(0.1, cam.DistortionK1());

  EXPECT_DOUBLE_EQ(0.0, cam.DistortionK2());
  cam.SetDistortionK2(0.2);
  EXPECT_DOUBLE_EQ(0.2, cam.DistortionK2());

  EXPECT_DOUBLE_EQ(0.0, cam.DistortionK3());
  cam.SetDistortionK3(0.2);
  EXPECT_DOUBLE_EQ(0.2, cam.DistortionK3());

  EXPECT_DOUBLE_EQ(0.0, cam.DistortionP1());
  cam.SetDistortionP1(0.2);
  EXPECT_DOUBLE_EQ(0.2, cam.DistortionP1());

  EXPECT_DOUBLE_EQ(0.0, cam.DistortionP2());
  cam.SetDistortionP2(0.2);
  EXPECT_DOUBLE_EQ(0.2, cam.DistortionP2());

  EXPECT_EQ(gz::math::Vector2d(0.5, 0.5), cam.DistortionCenter());
  cam.SetDistortionCenter(gz::math::Vector2d(0.1, 0.2));
  EXPECT_EQ(gz::math::Vector2d(0.1, 0.2), cam.DistortionCenter());

  GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  EXPECT_EQ(gz::math::Pose3d::Zero, cam.RawPose());
  cam.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), cam.RawPose());

  EXPECT_TRUE(cam.PoseRelativeTo().empty());
  cam.SetPoseRelativeTo("/frame");
  EXPECT_EQ("/frame", cam.PoseRelativeTo());
  GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION

  EXPECT_TRUE(cam.OpticalFrameId().empty());
  cam.SetOpticalFrameId("/optical_frame");
  EXPECT_EQ("/optical_frame", cam.OpticalFrameId());

  EXPECT_EQ("stereographic", cam.LensType());
  cam.SetLensType("custom");
  EXPECT_EQ("custom", cam.LensType());

  EXPECT_TRUE(cam.LensScaleToHfov());
  cam.SetLensScaleToHfov(false);
  EXPECT_FALSE(cam.LensScaleToHfov());

  EXPECT_DOUBLE_EQ(1.0, cam.LensC1());
  cam.SetLensC1(2.1);
  EXPECT_DOUBLE_EQ(2.1, cam.LensC1());

  EXPECT_DOUBLE_EQ(1.0, cam.LensC2());
  cam.SetLensC2(1.2);
  EXPECT_DOUBLE_EQ(1.2, cam.LensC2());

  EXPECT_DOUBLE_EQ(0.0, cam.LensC3());
  cam.SetLensC3(6.5);
  EXPECT_DOUBLE_EQ(6.5, cam.LensC3());

  EXPECT_DOUBLE_EQ(1.0, cam.LensFocalLength());
  cam.SetLensFocalLength(10.3);
  EXPECT_DOUBLE_EQ(10.3, cam.LensFocalLength());

  EXPECT_EQ("tan", cam.LensFunction());
  cam.SetLensFunction("sin");
  EXPECT_EQ("sin", cam.LensFunction());

  EXPECT_DOUBLE_EQ(GZ_PI_2, cam.LensCutoffAngle().Radian());
  cam.SetLensCutoffAngle(0.456);
  EXPECT_DOUBLE_EQ(0.456, cam.LensCutoffAngle().Radian());

  EXPECT_EQ(256, cam.LensEnvironmentTextureSize());
  cam.SetLensEnvironmentTextureSize(512);
  EXPECT_EQ(512, cam.LensEnvironmentTextureSize());

  EXPECT_DOUBLE_EQ(277, cam.LensIntrinsicsFx());
  cam.SetLensIntrinsicsFx(132);
  EXPECT_DOUBLE_EQ(132, cam.LensIntrinsicsFx());

  EXPECT_DOUBLE_EQ(277, cam.LensIntrinsicsFy());
  cam.SetLensIntrinsicsFy(456);
  EXPECT_DOUBLE_EQ(456, cam.LensIntrinsicsFy());

  EXPECT_DOUBLE_EQ(160, cam.LensIntrinsicsCx());
  cam.SetLensIntrinsicsCx(254);
  EXPECT_DOUBLE_EQ(254, cam.LensIntrinsicsCx());

  EXPECT_DOUBLE_EQ(120, cam.LensIntrinsicsCy());
  cam.SetLensIntrinsicsCy(123);
  EXPECT_DOUBLE_EQ(123, cam.LensIntrinsicsCy());

  EXPECT_DOUBLE_EQ(277, cam.LensProjectionFx());
  cam.SetLensProjectionFx(132);
  EXPECT_DOUBLE_EQ(132, cam.LensProjectionFx());

  EXPECT_DOUBLE_EQ(277, cam.LensProjectionFy());
  cam.SetLensProjectionFy(456);
  EXPECT_DOUBLE_EQ(456, cam.LensProjectionFy());

  EXPECT_DOUBLE_EQ(160, cam.LensProjectionCx());
  cam.SetLensProjectionCx(254);
  EXPECT_DOUBLE_EQ(254, cam.LensProjectionCx());

  EXPECT_DOUBLE_EQ(120, cam.LensProjectionCy());
  cam.SetLensProjectionCy(123);
  EXPECT_DOUBLE_EQ(123, cam.LensProjectionCy());

  EXPECT_DOUBLE_EQ(0, cam.LensProjectionTx());
  cam.SetLensProjectionTx(1);
  EXPECT_DOUBLE_EQ(1, cam.LensProjectionTx());

  EXPECT_DOUBLE_EQ(0, cam.LensProjectionTy());
  cam.SetLensProjectionTy(2);
  EXPECT_DOUBLE_EQ(2, cam.LensProjectionTy());

  EXPECT_DOUBLE_EQ(1.0, cam.LensIntrinsicsSkew());
  cam.SetLensIntrinsicsSkew(2.3);
  EXPECT_DOUBLE_EQ(2.3, cam.LensIntrinsicsSkew());

  EXPECT_TRUE(cam.HasLensIntrinsics());
  EXPECT_TRUE(cam.HasLensProjection());

  EXPECT_EQ(UINT32_MAX, cam.VisibilityMask());
  cam.SetVisibilityMask(123u);
  EXPECT_EQ(123u, cam.VisibilityMask());

  // Copy Constructor
  sdf::Camera cam2(cam);
  EXPECT_EQ(cam, cam2);

  // Copy operator
  sdf::Camera cam3;
  cam3 = cam;
  EXPECT_EQ(cam, cam3);

  // Move Constructor
  sdf::Camera cam4(std::move(cam));
  EXPECT_EQ(cam2, cam4);

  cam = cam4;
  EXPECT_EQ(cam2, cam);

  // Move operator
  sdf::Camera cam5;
  cam5 = std::move(cam2);
  EXPECT_EQ(cam3, cam5);

  cam2 = cam5;
  EXPECT_EQ(cam3, cam2);

  // inequality
  sdf::Camera cam6;
  EXPECT_NE(cam3, cam6);

  // The Camera::Load function is tested more thoroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMCamera, ToElement)
{
  sdf::Camera cam;
  cam.SetName("my_camera");
  EXPECT_EQ("my_camera", cam.Name());
  cam.SetHorizontalFov(1.45);
  cam.SetImageWidth(123);
  cam.SetImageHeight(125);
  cam.SetPixelFormat(sdf::PixelFormatType::L_INT8);
  cam.SetNearClip(0.2);
  cam.SetFarClip(200.2);
  cam.SetVisibilityMask(123u);
  GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  cam.SetPoseRelativeTo("/frame");
  GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  cam.SetSaveFrames(true);
  cam.SetSaveFramesPath("/tmp");
  cam.SetOpticalFrameId("/optical_frame");
  cam.SetCameraInfoTopic("/camera_info_test");
  cam.SetTriggerTopic("/trigger_topic_test");
  cam.SetTriggered(true);

  sdf::ElementPtr camElem = cam.ToElement();
  EXPECT_NE(nullptr, camElem);
  EXPECT_EQ(nullptr, cam.Element());

  // verify values after loading the element back
  sdf::Camera cam2;
  cam2.Load(camElem);

  EXPECT_DOUBLE_EQ(1.45, cam2.HorizontalFov().Radian());
  EXPECT_EQ(123u, cam2.ImageWidth());
  EXPECT_EQ(125u, cam2.ImageHeight());
  EXPECT_EQ(sdf::PixelFormatType::L_INT8 , cam2.PixelFormat());
  EXPECT_DOUBLE_EQ(0.2, cam2.NearClip());
  EXPECT_DOUBLE_EQ(200.2, cam2.FarClip());
  EXPECT_EQ(123u, cam2.VisibilityMask());
  GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  EXPECT_EQ("/frame", cam2.PoseRelativeTo());
  GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  EXPECT_TRUE(cam2.SaveFrames());
  EXPECT_EQ("/tmp", cam2.SaveFramesPath());
  EXPECT_EQ("/optical_frame", cam2.OpticalFrameId());
  EXPECT_EQ("/camera_info_test", cam2.CameraInfoTopic());
  EXPECT_EQ("/trigger_topic_test", cam2.TriggerTopic());
  EXPECT_TRUE(cam2.Triggered());

  // make changes to DOM and verify ToElement produces updated values
  cam2.SetNearClip(0.33);
  sdf::ElementPtr cam2Elem = cam2.ToElement();
  EXPECT_NE(nullptr, cam2Elem);
  sdf::Camera cam3;
  cam3.Load(cam2Elem);
  EXPECT_DOUBLE_EQ(0.33, cam3.NearClip());
}
