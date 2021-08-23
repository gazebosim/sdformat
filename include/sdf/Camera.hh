/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_CAMERA_HH_
#define SDF_CAMERA_HH_

#include <string>
#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \enum PixelFormatType
  /// \brief The set of pixel formats. This list should match
  /// ignition::common::Image::PixelFormatType.
  enum class PixelFormatType
  {
    UNKNOWN_PIXEL_FORMAT = 0,
    L_INT8,
    L_INT16,
    RGB_INT8,
    RGBA_INT8,
    BGRA_INT8,
    RGB_INT16,
    RGB_INT32,
    BGR_INT8,
    BGR_INT16,
    BGR_INT32,
    R_FLOAT16,
    RGB_FLOAT16,
    R_FLOAT32,
    RGB_FLOAT32,
    BAYER_RGGB8,
    BAYER_BGGR8,
    BAYER_GBRG8,
    BAYER_GRBG8,
  };

  /// \brief Information about a monocular camera sensor.
  class SDFORMAT_VISIBLE Camera
  {
    /// \brief Constructor
    public: Camera();

    /// \brief Return true if both Camera objects contain the same values.
    /// \param[_in] _alt Camera value to compare.
    /// \returen True if 'this' == _alt.
    public: bool operator==(const Camera &_alt) const;

    /// \brief Return true this Camera object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _alt Camera value to compare.
    /// \returen True if 'this' != _alt.
    public: bool operator!=(const Camera &_alt) const;

    /// \brief Load the camera sensor based on an element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the name of the camera.
    /// \return Name of the sensor.
    public: std::string Name() const;

    /// \brief Set the name of the camera.
    /// \param[in] _name Name of the sensor.
    public: void SetName(const std::string &_name);

    /// \brief Get the horizontal field of view in radians.
    /// \return The horizontal field of view in radians.
    public: ignition::math::Angle HorizontalFov() const;

    /// \brief Set the horizontal field of view in radians.
    /// \param[in] _hfov The horizontal field of view in radians.
    public: void SetHorizontalFov(const ignition::math::Angle &_hfov);

    /// \brief Get the image width in pixels.
    /// \return The image width in pixels.
    public: uint32_t ImageWidth() const;

    /// \brief Set the image width in pixels.
    /// \param[in] _width The image width in pixels.
    public: void SetImageWidth(uint32_t _width);

    /// \brief Get the image height in pixels.
    /// \return The image height in pixels.
    public: uint32_t ImageHeight() const;

    /// \brief Set the image height in pixels.
    /// \param[in] _height The image height in pixels.
    public: void SetImageHeight(uint32_t _height);

    /// \brief Get the pixel format. This value is set from the <format>
    /// element that is the child of <image>.
    /// \return The pixel format.
    public: PixelFormatType PixelFormat() const;

    /// \brief Set the pixel format type.
    /// \param[in] _format The image format.
    public: void SetPixelFormat(PixelFormatType _format);

    /// \brief Get the pixel format as a string.
    /// \return The pixel format string.
    public: std::string PixelFormatStr() const;

    /// \brief Set the pixel format from a string.
    /// \param[in] _fmt The pixel format string.
    public: void SetPixelFormatStr(const std::string &_fmt);

    /// \brief Get the near clip distance for the depth camera.
    /// \return The near clip depth distance.
    public: double DepthNearClip() const;

    /// \brief Set the near clip distance for the depth camera.
    /// \param[in] _near The near clip depth distance.
    public: void SetDepthNearClip(double _near);

    /// \brief Get the far clip distance for the depth camera.
    /// \return The far clip depth distance.
    public: double DepthFarClip() const;

    /// \brief Set the far clip distance for the depth camera.
    /// \param[in] _far The far clip depth distance.
    public: void SetDepthFarClip(double _far);

    /// \brief Get the near clip distance.
    /// \return The near clip distance.
    public: double NearClip() const;

    /// \brief Set the near clip distance.
    /// \param[in] _near The near clip distance.
    public: void SetNearClip(double _near);

    /// \brief Set whether the depth camera has been specified.
    /// \param[in] _camera True if the depth camera has been set in the sdf.
    public: void SetHasDepthCamera(bool _camera);

    /// \brief Get whether the depth camera was set.
    /// \return True if the depth camera was set.
    public: bool HasDepthCamera() const;

    /// \brief Set whether the depth camera near clip distance
    /// has been specified.
    /// \param[in] _camera True if the depth camera near clip distance
    /// has been set in the sdf.
    public: void SetHasDepthNearClip(bool _near);

    /// \brief Get whether the depth camera near clip distance was set.
    /// \return True if the depth camera near clip distance was set.
    public: bool HasDepthNearClip() const;

    /// \brief Set whether the depth camera far clip distance
    /// has been specified.
    /// \param[in] _camera True if the depth camera far clip distance
    /// has been set in the sdf.
    public: void SetHasDepthFarClip(bool _far);

    /// \brief Get whether the depth camera far clip distance was set.
    /// \return True if the depth camera far clip distance was set.
    public: bool HasDepthFarClip() const;

    /// \brief Get the far clip distance.
    /// \return The far clip distance.
    public: double FarClip() const;

    /// \brief Set the far clip distance.
    /// \param[in] _far The far clip distance.
    public: void SetFarClip(double _far);

    /// \brief Set whether the segmentation type has been specified.
    /// \param[in] _type True if the segmentation type
    /// has been set in the sdf.
    public: void SetHasSegmentationType(bool _type);

    /// \brief Get whether the segmentation type was set.
    /// \return True if the segmentation type was set.
    public: bool HasSegmentationType() const;

    /// \brief Get the segmentation type.
    /// \return The segmentation type.
    public: const std::string &SegmentationType() const;

    /// \brief Set the segmentation type.
    /// \param[in] _type The segmentation type.
    public: void SetSegmentationType(const std::string &_type);

    /// \brief Set whether the boundingbox type has been specified.
    /// \param[in] _type True if the boundingbox type
    /// has been set in the sdf.
    public: void SetHasBoundingBoxType(bool _type);

    /// \brief Get whether the boundingbox type was set.
    /// \return True if the boundingbox type was set.
    public: bool HasBoundingBoxType() const;

    /// \brief Get the boundingbox type.
    /// \return The boundingbox type.
    public: const std::string &BoundingBoxType() const;

    /// \brief Set the boundingbox type.
    /// \param[in] _type The boundingbox type.
    public: void SetBoundingBoxType(const std::string &_type);

    /// \brief Get whether frames should be saved.
    /// \return True if image frames should be saved.
    public: bool SaveFrames() const;

    /// \brief Set whether frames should be saved.
    /// \param[in] _save True if image frames should be saved.
    public: void SetSaveFrames(bool _save);

    /// \brief Get the path in which to save frames.
    /// \return Path to save frames.
    public: const std::string &SaveFramesPath() const;

    /// \brief Set the path in which to save frames.
    /// \param[in] _path Path to save frames.
    public: void SetSaveFramesPath(const std::string &_path);

    /// \brief Get the image noise values.
    /// \return Noise values for the image.
    public: const Noise &ImageNoise() const;

    /// \brief Set the noise values related to the image.
    /// \param[in] _noise Noise values for the image.
    public: void SetImageNoise(const Noise &_noise);

    /// \brief Get the radial distortion coefficient k1
    /// \return _k1 The k1 radial distortion.
    public: double DistortionK1() const;

    /// \brief Set the radial distortion coefficient k1
    /// \param[in] _k1 The k1 radial distortion.
    public: void SetDistortionK1(double _k1);

    /// \brief Get the radial distortion coefficient k2
    /// \return _k2 The k2 radial distortion.
    public: double DistortionK2() const;

    /// \brief Set the radial distortion coefficient k2
    /// \param[in] _k2 The k2 radial distortion.
    public: void SetDistortionK2(double _k2);

    /// \brief Get the radial distortion coefficient k3
    /// \return _k3 The k3 radial distortion.
    public: double DistortionK3() const;

    /// \brief Set the radial distortion coefficient k3
    /// \param[in] _k3 The k3 radial distortion.
    public: void SetDistortionK3(double _k3);

    /// \brief Get the tangential distortion coefficient p1
    /// \return _p1 The p1 tangential distortion.
    public: double DistortionP1() const;

    /// \brief Set the tangential distortion coefficient p1
    /// \param[in] _p1 The p1 tangential distortion.
    public: void SetDistortionP1(double _p1);

    /// \brief Get the tangential distortion coefficient p2
    /// \return The p2 tangential distortion.
    public: double DistortionP2() const;

    /// \brief Set the tangential distortion coefficient p2
    /// \return  The p2 tangential distortion.
    public: void SetDistortionP2(double _p2);

    /// \brief Get the distortion center or principal point.
    /// \return Distortion center or principal point.
    public: const ignition::math::Vector2d &DistortionCenter() const;

    /// \brief Set the distortion center or principal point.
    /// \param[in] _center Distortion center or principal point.
    public: void SetDistortionCenter(const ignition::math::Vector2d &_center);

    /// \brief Get the pose of the camer. This is the pose of the camera
    /// as specified in SDF (<camera> <pose> ... </pose></camera>).
    /// \return The pose of the link.
    public: const ignition::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the camera.
    /// \sa const ignition::math::Pose3d &RawPose() const
    /// \param[in] _pose The new camera pose.
    public: void SetRawPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get the lens type. This is the type of the lens mapping.
    /// Supported values are gnomonical, stereographic, equidistant,
    /// equisolid_angle, orthographic, custom. For gnomonical (perspective)
    /// projection, it is recommended to specify a horizontal_fov of less than
    /// or equal to 90 degrees
    /// \return The lens type.
    public: std::string LensType() const;

    /// \brief Set the lens type. Supported values are gnomonical,
    /// stereographic, equidistant, equisolid_angle, orthographic, custom.
    /// \param[in] _type The lens type.
    public: void SetLensType(const std::string &_type);

    /// \brief Get lens scale to horizontal field of field.
    /// \return True if the image should be scaled to fit horizontal FOV,
    /// otherwise it will be shown according to projection type parameters.
    public: bool LensScaleToHfov() const;

    /// \brief Set lens scale to horizontal field of field.
    /// \param[in] _scale True if the image should be scaled to fit horizontal
    /// FOV, otherwise it will be shown according to projection type parameters.
    public: void SetLensScaleToHfov(bool _scale);

    /// \brief Get lens custom function linear scaling constant.
    /// \return The lens custom function linear scaling constant.
    public: double LensC1() const;

    /// \brief Set lens custom function linear scaling constant.
    /// \param[in] _c1 The lens custom function linear scaling constant.
    public: void SetLensC1(double _c1);

    /// \brief Get lens custom function angular scaling constant.
    /// \return The lens custom function angular scaling constant.
    public: double LensC2() const;

    /// \brief Set lens custom function angular scaling constant.
    /// \param[in] _c2 The lens custom function angular scaling constant.
    public: void SetLensC2(double _c2);

    /// \brief Get lens custom function angle offset constant.
    /// \return The lens custom function angle offset constant.
    public: double LensC3() const;

    /// \brief Set lens custom function angle offset constant.
    /// \param[in] _c3 The lens custom function angle offset constant.
    public: void SetLensC3(double _c3);

    /// \brief Get lens custom function focal length.
    /// \return The lens custom function focal length.
    public: double LensFocalLength() const;

    /// \brief Set lens custom function focal length.
    /// \param[in] _f The lens custom function focal length.
    public: void SetLensFocalLength(double _f);

    /// \brief Get lens custom function. Possible values are 'sin', 'tan',
    /// and 'id'.
    /// \return The lens custom function.
    public: const std::string &LensFunction() const;

    /// \brief Set lens custom function.
    /// \param[in] _fun The lens custom function. Possible values are 'sin',
    /// 'tan', and 'id'.
    public: void SetLensFunction(const std::string &_fun);

    /// \brief Get lens cutoff angle. Everything outside of the specified
    /// angle will be hidden.
    /// \return The lens cutoff angle.
    public: ignition::math::Angle LensCutoffAngle() const;

    /// \brief Set lens cutoff angle. Everything outside of the specified
    /// angle will be hidden.
    /// \param[in] _angle The lens cutoff angle.
    public: void SetLensCutoffAngle(const ignition::math::Angle &_angle);

    /// \brief Get environment texture size. This is the resolution of the
    /// environment cube map used to draw the world.
    /// \return The lens environment texture size.
    public: int LensEnvironmentTextureSize() const;

    /// \brief Set environment texture size. This is the resolution of the
    /// environment cube map used to draw the world.
    /// \param[in] _size The lens environment texture size.
    public: void SetLensEnvironmentTextureSize(int _size);

    /// \brief Get the lens X focal length in pixels.
    /// \return The lens X focal length in pixels.
    public: double LensIntrinsicsFx() const;

    /// \brief Set the lens X focal length in pixels.
    /// \param[in] _fx The lens X focal length in pixels.
    public: void SetLensIntrinsicsFx(double _fx);

    /// \brief Get the lens Y focal length in pixels.
    /// \return The lens Y focal length in pixels.
    public: double LensIntrinsicsFy() const;

    /// \brief Set the lens Y focal length in pixels.
    /// \param[in] _fy The lens Y focal length in pixels.
    public: void SetLensIntrinsicsFy(double _fy);

    /// \brief Get the lens X principal point in pixels.
    /// \return The lens X principal point in pixels.
    public: double LensIntrinsicsCx() const;

    /// \brief Set the lens X principal point in pixels.
    /// \param[in] _cx The lens X principal point in pixels.
    public: void SetLensIntrinsicsCx(double _cx);

    /// \brief Get the lens Y principal point in pixels.
    /// \return The lens Y principal point in pixels.
    public: double LensIntrinsicsCy() const;

    /// \brief Set the lens Y principal point in pixels.
    /// \param[in] _cy The lens Y principal point in pixels.
    public: void SetLensIntrinsicsCy(double _cy);

    /// \brief Get the lens XY axis skew.
    /// \return The lens XY axis skew.
    public: double LensIntrinsicsSkew() const;

    /// \brief Set the lens XY axis skew.
    /// \param[in] _s The lens XY axis skew.
    public: void SetLensIntrinsicsSkew(double _s);

    /// \brief Convert a string to a PixelFormatType.
    /// \param[in] _format String equivalent of a pixel format type to convert.
    /// \return The matching PixelFormatType.
    public: static PixelFormatType ConvertPixelFormat(
                const std::string &_format);

    /// \brief Convert a PixelFormatType to a string.
    /// \param[in] _type Pixel format type to convert.
    /// \return String equivalent of _type.
    public: static std::string ConvertPixelFormat(PixelFormatType _type);

    /// \brief Get the visibility mask of a camera
    /// \return visibility mask
    public: uint32_t VisibilityMask() const;

    /// \brief Set the visibility mask of a camera
    /// \param[in] _mask visibility mask
    public: void SetVisibilityMask(uint32_t _mask);

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}

#endif
