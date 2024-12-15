/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
 */

#include "pyCamera.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Camera.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineCamera(pybind11::object module)
{
  pybind11::class_<sdf::Camera> cameraModule(module, "Camera");
  cameraModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Camera>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("name", &sdf::Camera::Name,
         "Get the name of the camera.")
    .def("set_name", &sdf::Camera::SetName,
         "Set the name of the camera.")
    .def("set_camera_info_topic", &sdf::Camera::SetCameraInfoTopic,
         "Set the camera info topic")
    .def("camera_info_topic", &sdf::Camera::CameraInfoTopic,
         "Get the camera info topic.")
    .def("triggered", &sdf::Camera::Triggered,
         "Get whether the camera is triggered by a topic.")
    .def("set_triggered", &sdf::Camera::SetTriggered,
         "Set whether the camera should be triggered by a topic.")
    .def("trigger_topic", &sdf::Camera::TriggerTopic,
         "Set whether the camera should be triggered by a topic.")
    .def("set_trigger_topic", &sdf::Camera::SetTriggerTopic,
         "Get the topic that will trigger the camera.")
    .def("horizontal_fov", &sdf::Camera::HorizontalFov,
         "Get the horizontal field of view in radians.")
    .def("set_horizontal_fov", &sdf::Camera::SetHorizontalFov,
         "Set the horizontal field of view in radians.")
    .def("image_width", &sdf::Camera::ImageWidth,
         "Get the image width in pixels.")
    .def("set_image_width", &sdf::Camera::SetImageWidth,
         "Set the image width in pixels.")
    .def("image_height", &sdf::Camera::ImageHeight,
         "Get the image height in pixels.")
    .def("set_image_height", &sdf::Camera::SetImageHeight,
         "Set the image height in pixels.")
    .def("pixel_format", &sdf::Camera::PixelFormat,
         "Get the pixel format. This value is set from the <format> "
         "element that is the child of <image>.")
    .def("set_pixel_format", &sdf::Camera::SetPixelFormat,
         "Set the pixel format type.")
    .def("pixel_format_str", &sdf::Camera::PixelFormatStr,
         "Get the pixel format as a string.")
    .def("set_pixel_format_str", &sdf::Camera::SetPixelFormatStr,
         "Set the pixel format from a string.")
    .def("anti_aliasing_value", &sdf::Camera::AntiAliasingValue,
         "Get the anti-aliasing value.")
    .def("set_anti_aliasing_value", &sdf::Camera::SetAntiAliasingValue,
         "Set the anti-aliasing value.")
    .def("depth_near_clip", &sdf::Camera::DepthNearClip,
         "Get the near clip distance for the depth camera.")
    .def("set_depth_near_clip", &sdf::Camera::SetDepthNearClip,
         "Set the near clip distance for the depth camera.")
    .def("depth_far_clip", &sdf::Camera::DepthFarClip,
         "Get the far clip distance for the depth camera.")
    .def("set_depth_far_clip", &sdf::Camera::SetDepthFarClip,
         "Set the far clip distance for the depth camera.")
    .def("near_clip", &sdf::Camera::NearClip,
         "Get the near clip distance.")
    .def("set_near_clip", &sdf::Camera::SetNearClip,
         "Set the near clip distance.")
    .def("set_has_depth_camera", &sdf::Camera::SetHasDepthCamera,
         "Set whether the depth camera has been specified.")
    .def("has_depth_camera", &sdf::Camera::HasDepthCamera,
         "Get whether the depth camera was set.")
    .def("set_has_depth_near_clip", &sdf::Camera::SetHasDepthNearClip,
         "Get whether the depth camera near clip distance was set.")
    .def("has_depth_near_clip", &sdf::Camera::HasDepthNearClip,
         "Set whether the depth camera near clip distance "
         "has been specified.")
    .def("set_has_depth_far_clip", &sdf::Camera::SetHasDepthFarClip,
         "Set whether the depth camera far clip distance "
         "has been specified.")
    .def("has_depth_far_clip", &sdf::Camera::HasDepthFarClip,
         "Get whether the depth camera far clip distance was set.")
    .def("far_clip", &sdf::Camera::FarClip,
         "Get the far clip distance.")
    .def("set_far_clip", &sdf::Camera::SetFarClip,
         "Set the far clip distance.")
    .def("set_has_segmentation_type", &sdf::Camera::SetHasSegmentationType,
         "Set whether the segmentation type has been specified.")
    .def("has_segmentation_type", &sdf::Camera::HasSegmentationType,
         "Get whether the segmentation type was set.")
    .def("segmentation_type", &sdf::Camera::SegmentationType,
         "Get the segmentation type.")
    .def("set_segmentation_type", &sdf::Camera::SetSegmentationType,
         "Set the segmentation type.")
    .def("set_has_bounding_box_type", &sdf::Camera::SetHasBoundingBoxType,
         "Set whether the boundingbox type has been specified.")
    .def("has_bounding_box_type", &sdf::Camera::HasBoundingBoxType,
         "Get whether the boundingbox type was set.")
    .def("bounding_box_type", &sdf::Camera::BoundingBoxType,
         "Get the boundingbox type.")
    .def("set_bounding_box_type", &sdf::Camera::SetBoundingBoxType,
         "Set the boundingbox type.")
    .def("save_frames", &sdf::Camera::SaveFrames,
         "Get whether frames should be saved.")
    .def("set_save_frames", &sdf::Camera::SetSaveFrames,
         "Set whether frames should be saved.")
    .def("save_frames_path", &sdf::Camera::SaveFramesPath,
         "Get the path in which to save frames.")
    .def("set_save_frames_path", &sdf::Camera::SetSaveFramesPath,
         "Set the path in which to save frames.")
    .def("image_noise", &sdf::Camera::ImageNoise,
         pybind11::return_value_policy::reference,
         "Get the image noise values.")
    .def("set_image_noise", &sdf::Camera::SetImageNoise,
         "Set the noise values related to the image.")
    .def("distortion_k1", &sdf::Camera::DistortionK1,
         "Set the radial distortion coefficient k1")
    .def("set_distortion_k1", &sdf::Camera::SetDistortionK1,
         "Set the radial distortion coefficient k1")
    .def("distortion_k2", &sdf::Camera::DistortionK2,
         "Set the radial distortion coefficient k2")
    .def("set_distortion_k2", &sdf::Camera::SetDistortionK2,
         "Set the radial distortion coefficient k2")
    .def("distortion_k3", &sdf::Camera::DistortionK3,
         "Set the radial distortion coefficient k3")
    .def("set_distortion_k3", &sdf::Camera::SetDistortionK3,
         "Set the radial distortion coefficient k3")
    .def("distortion_p1", &sdf::Camera::DistortionP1,
         "Set the radial distortion coefficient P1")
    .def("set_distortion_p1", &sdf::Camera::SetDistortionP1,
         "Set the radial distortion coefficient P1")
    .def("distortion_p2", &sdf::Camera::DistortionP2,
         "Set the radial distortion coefficient P2")
    .def("set_distortion_p2", &sdf::Camera::SetDistortionP2,
         "Set the radial distortion coefficient P2")
    .def("distortion_center", &sdf::Camera::DistortionCenter,
         "Get the distortion center or principal point.")
    .def("set_distortion_center", &sdf::Camera::SetDistortionCenter,
         "Set the distortion center or principal point.")
    .def("raw_pose", &sdf::Camera::RawPose,
         "Get the pose of the camer. This is the pose of the camera "
         "as specified in SDF (<camera> <pose> ... </pose></camera>).")
    .def("set_raw_pose", &sdf::Camera::SetRawPose,
         "Set the pose of the camera.")
    .def("pose_relative_to", &sdf::Camera::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("set_pose_relative_to", &sdf::Camera::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("lens_type", &sdf::Camera::LensType,
         "Get the lens type. This is the type of the lens mapping. "
         "Supported values are gnomonical, stereographic, equidistant, "
         "equisolid_angle, orthographic, custom. For gnomonical (perspective) "
         "projection, it is recommended to specify a horizontal_fov of less "
         "than or equal to 90 degrees")
    .def("set_lens_type", &sdf::Camera::SetLensType,
         "Set the lens type. Supported values are gnomonical, "
         "stereographic, equidistant, equisolid_angle, orthographic, custom.")
    .def("lens_scale_to_hfov", &sdf::Camera::LensScaleToHfov,
         "Get lens scale to horizontal field of field.")
    .def("set_lens_scale_to_hfov", &sdf::Camera::SetLensScaleToHfov,
         "Set lens scale to horizontal field of field.")
    .def("lens_c1", &sdf::Camera::LensC1,
         "Get lens custom function linear scaling constant.")
    .def("set_lens_c1", &sdf::Camera::SetLensC1,
         "Set lens custom function linear scaling constant.")
    .def("lens_c2", &sdf::Camera::LensC2,
         "Get lens custom function angular scaling constant.")
    .def("set_lens_c2", &sdf::Camera::SetLensC2,
         "Set lens custom function angular scaling constant.")
    .def("lens_c3", &sdf::Camera::LensC3,
         "Get lens custom function angle offset constant")
    .def("set_lens_c3", &sdf::Camera::SetLensC3,
         "Set lens custom function angle offset constant")
    .def("lens_focal_length", &sdf::Camera::LensFocalLength,
         "Get lens custom function focal length.")
    .def("set_lens_focal_length", &sdf::Camera::SetLensFocalLength,
         "Set lens custom function focal length.")
    .def("lens_function", &sdf::Camera::LensFunction,
         "Get lens custom function. Possible values are 'sin', 'tan', "
         "and 'id'.")
    .def("set_lens_function", &sdf::Camera::SetLensFunction,
         "Set lens custom function.")
    .def("lens_cutoff_angle", &sdf::Camera::LensCutoffAngle,
         "Get lens cutoff angle. Everything outside of the specified "
         "angle will be hidden.")
    .def("set_lens_cutoff_angle", &sdf::Camera::SetLensCutoffAngle,
         "Set lens cutoff angle. Everything outside of the specified "
         "angle will be hidden.")
    .def("lens_environment_texture_size",
         &sdf::Camera::LensEnvironmentTextureSize,
         "Get environment texture size. This is the resolution of the "
         "environment cube map used to draw the world.")
    .def("set_lens_environment_texture_size",
         &sdf::Camera::SetLensEnvironmentTextureSize,
         "Set environment texture size. This is the resolution of the "
         "environment cube map used to draw the world.")
    .def("lens_intrinsics_fx", &sdf::Camera::LensIntrinsicsFx,
         "Get the lens X focal length in pixels.")
    .def("set_lens_intrinsics_fx", &sdf::Camera::SetLensIntrinsicsFx,
         "Set the lens X focal length in pixels.")
    .def("lens_intrinsics_fy", &sdf::Camera::LensIntrinsicsFy,
         "Get the lens Y focal length in pixels.")
    .def("set_lens_intrinsics_fy", &sdf::Camera::SetLensIntrinsicsFy,
         "Set the lens Y focal length in pixels.")
    .def("lens_intrinsics_cx", &sdf::Camera::LensIntrinsicsCx,
         "Get the lens X principal point in pixels.")
    .def("set_lens_intrinsics_cx", &sdf::Camera::SetLensIntrinsicsCx,
         "Set the lens X principal point in pixels.")
    .def("lens_intrinsics_cy", &sdf::Camera::LensIntrinsicsCy,
         "Get the lens Y principal point in pixels.")
    .def("set_lens_intrinsics_cy", &sdf::Camera::SetLensIntrinsicsCy,
         "Set the lens Y principal point in pixels.")
    .def("lens_intrinsics_skew", &sdf::Camera::LensIntrinsicsSkew,
         "Get the lens XY axis skew.")
    .def("set_lens_intrinsics_skew", &sdf::Camera::SetLensIntrinsicsSkew,
         "Set the lens XY axis skew.")
    .def("convert_pixel_format",
         pybind11::overload_cast<const std::string &>(
           &sdf::Camera::ConvertPixelFormat),
         "Convert a string to a PixelFormatType.")
    .def("convert_pixel_format",
         pybind11::overload_cast<sdf::PixelFormatType>(
           &sdf::Camera::ConvertPixelFormat),
         "Convert a PixelFormatType to a string.")
    .def("visibility_mask", &sdf::Camera::VisibilityMask,
         "Get the visibility mask of a camera")
    .def("set_visibility_mask", &sdf::Camera::SetVisibilityMask,
         "Set the visibility mask of a camera")
    .def("has_lens_intrinsics", &sdf::Camera::HasLensIntrinsics,
         "Get whether or not the camera has instrinsics values set")
    .def("has_lens_projection", &sdf::Camera::HasLensProjection,
         "Get whether or not the camera has proejction values set")
    .def("__copy__", [](const sdf::Camera &self) {
      return sdf::Camera(self);
    })
    .def("__deepcopy__", [](const sdf::Camera &self, pybind11::dict) {
      return sdf::Camera(self);
    }, "memo"_a);

    pybind11::enum_<sdf::PixelFormatType>(module, "PixelFormatType")
      .value("UNKNOWN_PIXEL_FORMAT", sdf::PixelFormatType::UNKNOWN_PIXEL_FORMAT)
      .value("L_INT8", sdf::PixelFormatType::L_INT8)
      .value("L_INT16", sdf::PixelFormatType::L_INT16)
      .value("RGB_INT8", sdf::PixelFormatType::RGB_INT8)
      .value("RGBA_INT8", sdf::PixelFormatType::RGBA_INT8)
      .value("BGRA_INT8", sdf::PixelFormatType::BGRA_INT8)
      .value("RGB_INT16", sdf::PixelFormatType::RGB_INT16)
      .value("RGB_INT32", sdf::PixelFormatType::RGB_INT32)
      .value("BGR_INT8", sdf::PixelFormatType::BGR_INT8)
      .value("BGR_INT16", sdf::PixelFormatType::BGR_INT16)
      .value("BGR_INT32", sdf::PixelFormatType::BGR_INT32)
      .value("R_FLOAT16", sdf::PixelFormatType::R_FLOAT16)
      .value("RGB_FLOAT16", sdf::PixelFormatType::RGB_FLOAT16)
      .value("R_FLOAT32", sdf::PixelFormatType::R_FLOAT32)
      .value("RGB_FLOAT32", sdf::PixelFormatType::RGB_FLOAT32)
      .value("BAYER_RGGB8", sdf::PixelFormatType::BAYER_RGGB8)
      .value("BAYER_BGGR8", sdf::PixelFormatType::BAYER_BGGR8)
      .value("BAYER_GBRG8", sdf::PixelFormatType::BAYER_GBRG8)
      .value("BAYER_GRBG8", sdf::PixelFormatType::BAYER_GRBG8);

}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
