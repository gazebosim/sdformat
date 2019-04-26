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

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Forward declare private data class.
  class CameraPrivate;

  /// \enum PixelFormatType
  /// \brief The set of pixel formats. This list should match
  /// ignition::common::Image::PixelFormatType.
  enum class PixelFormatType
  {
    UNKNOWN_PIXEL_FORMAT = 0;
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
    BAYER_RGGR8,
    BAYER_GBRG8,
    BAYER_GRBG8,
  };

  /// \brief Information about a monocular camera sensor.
  class SDFORMAT_VISIBLE Camera
  {
    /// \brief Constructor
    public: Camera();

    /// \brief Copy constructor
    /// \param[in] _camera Camera to copy.
    public: Camera(const Camera &_camera);

    /// \brief Move constructor
    /// \param[in] _camera Camera to move.
    public: Camera(Camera &&_camera) noexcept;

    /// \brief Destructor
    public: virtual ~Camera();

    /// \brief Assignment operator.
    /// \param[in] _camera The camera to set values from.
    /// \return *this
    public: Camera &operator=(const Camera &_camera);

    /// \brief Move assignment operator.
    /// \param[in] _camera The camera to set values from.
    /// \return *this
    public: Camera &operator=(Camera &&_camera) noexcept;

    /// \brief Return true if both Camera objects contain the same values.
    /// \param[_in] _alt Camera value to compare.
    /// \returen True if 'this' == _alt.
    public: bool operator==(const Camera &_alt) const;

    /// \brief Return true this Camera object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _alt Camera value to compare.
    /// \returen True if 'this' != _alt.
    public: bool operator!=(const Camera &_alt) const;

    /// \brief Load the camera sensor geometry based on a element pointer.
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
    public: double HorizontalFov() const;

    /// \brief Set the horizontal field of view in radians.
    /// \param[in] _hfov The horizontal field of view in radians.
    public: void SetHorizontalFov(double _hfov);

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

    /// \brief Get the near clip distance.
    /// \return The near clip distance.
    public: double NearClip() const;

    /// \brief Set the near clip distance.
    /// \param[in] _near The near clip distance.
    public: void SetNearClip(double _near);

    /// \brief Get the far clip distance.
    /// \return The far clip distance.
    public: double FarClip() const;

    /// \brief Set the far clip distance.
    /// \param[in] _far The far clip distance.
    public: void SetFarClip(double _far);

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
    public: void DistortionP1(double _p1);

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
    public: void SetDistortionCenter(
                const ignition::math::Vector2d &_center) const;

    /// \brief Private data pointer.
    private: CameraPrivate *dataPtr;
  };
}

#endif
