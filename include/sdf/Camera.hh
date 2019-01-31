/*
 * Copyright 2018 Open Source Robotics Foundation
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

namespace sdf
{
  // Forward declare private data class.
  class CameraPrivate;

  /// \enum ImageFormat
  /// \brief The set of image formats.
  enum class ImageFormatType
  {
    /// \brief 8-bit RGB channels
    R8G8B8 = 0,

    /// \brief 8-bit luminence channel.
    L8 = 1,

    /// \brief 8-bit BGR channels.
    B8G8R8 = 2,

    /// \brief Bayer RGGB8 channels.
    BAYER_RGGB8 = 3,

    /// \brief Bayer BGGR8 channels.
    BAYER_BGGR8 = 4,

    /// \brief Bayer GBRG8 channels.
    BAYER_GBRG8 = 5,

    /// \brief Bayer GRBG8 channels.
    BAYER_GRBG8 = 6,
  };

  /// \brief Information about a monocular camera sensor.
  class SDFORMAT_VISIBLE Camera
  {
    /// \brief Constructor
    public: Camera();

    /// \brief Destructor
    public: virtual ~Camera();

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

    /// \brief Get the image format.
    /// \return The image format.
    public: ImageFormatType ImageFormat() const;

    /// \brief Set the image format.
    /// \param[in] _format The image format.
    public: void SetImageFormat(ImageFormatType _format);

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

    /// \brief Private data pointer.
    private: CameraPrivate *dataPtr;
  };
}

#endif
