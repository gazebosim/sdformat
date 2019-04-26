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
#include "sdf/Camera.hh"
#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::CameraPrivate
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Name of the camera.
  public: std::string name = "";

  /// \brief Horizontal fied of view.
  public: double hfov{1.047};

  /// \brief Image width.
  public: uint32_t imageWidth{320};

  /// \brief Image height.
  public: uint32_t imageHeight{240};

  /// \brief Image format.
  public: ImageFormatType imageFormat{ImageFormatType::R8G8B8};

  /// \brief Near clip distance.
  public: double nearClip{0.1};

  /// \brief Far clip distance.
  public: double farClip{100};

  /// \brief True indicates frames should be saved.
  public: bool save{false};

  /// \brief Path in which to save frames.
  public: std::string savePath{""};

  /// \brief The image noise value.
  public: Noise imageNoise;

  /// \brief The radial distortion coefficient k1.
  public: double distortionK1{0.0};

  /// \brief The radial distortion coefficient k2.
  public: double distortionK2{0.0};

  /// \brief The radial distortion coefficient k3.
  public: double distortionK3{0.0};

  /// \brief Thecw tangential distortion coefficient p1.
  public: double distortionP1{0.0};

  /// \brief Thecw tangential distortion coefficient p2.
  public: double distortionP2{0.0};

  /// \brief The distortion center or principal point
  public: ignition::math::Vector2d distortionCenter{0.5, 0.5};
};

/////////////////////////////////////////////////
Camera::Camera()
  : dataPtr(new CameraPrivate)
{
}

/////////////////////////////////////////////////
Camera::Camera(const Camera &_camera)
  : dataPtr(new CameraPrivate(*_camera.dataPtr))
{
}

/////////////////////////////////////////////////
Camera::Camera(Camera &&_camera)
{
  this->dataPtr = _camera.dataPtr;
  _camera.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Camera::~Camera()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Camera::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a camera sensor, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a camera element
  if (_sdf->GetName() != "camera")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a camera sensor, but the provided SDF "
        "element is not a <camera>."});
    return errors;
  }

  // Read the camera's's name
  loadName(_sdf, this->dataPtr->name);

  this->dataPtr->hfov = _sdf->Get<double>("horizontal_fov",
      this->dataPtr->hfov).first;

  if (_sdf->HasElement("image"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("image");
    this->dataPtr->imageWidth = elem->Get<double>("width",
        this->dataPtr->imageWidth).first;
    this->dataPtr->imageHeight = elem->Get<double>("height",
        this->dataPtr->imageHeight).first;
    std::string format = elem->Get<std::string>("format", "R8G8B8").first;
    if (format == "R8G8B8")
      this->dataPtr->imageFormat = ImageFormatType::R8G8B8;
    else if (format == "L8")
      this->dataPtr->imageFormat = ImageFormatType::L8;
    else if (format == "B8G8R8")
      this->dataPtr->imageFormat = ImageFormatType::B8G8R8;
    else if (format == "BAYER_RGGB8")
      this->dataPtr->imageFormat = ImageFormatType::BAYER_RGGB8;
    else if (format == "BAYER_BGGR8")
      this->dataPtr->imageFormat = ImageFormatType::BAYER_BGGR8;
    else if (format == "BAYER_GBRG8")
      this->dataPtr->imageFormat = ImageFormatType::BAYER_GBRG8;
    else if (format == "BAYER_GRBG8")
      this->dataPtr->imageFormat = ImageFormatType::BAYER_GRBG8;
    else
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Camera sensor <image><format> has invalid value of " + format});
    }

  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Camera sensor is missing an <image> element."});
  }

  if (_sdf->HasElement("clip"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("clip");
    this->dataPtr->nearClip = elem->Get<double>("near",
        this->dataPtr->nearClip).first;
    this->dataPtr->farClip = elem->Get<double>("far",
        this->dataPtr->farClip).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Camera sensor is missing a <clip> element."});
  }

  if (_sdf->HasElement("save"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("save");
    this->dataPtr->save = elem->Get<bool>("enabled", this->dataPtr->save).first;
    if (this->dataPtr->save)
    {
      this->dataPtr->savePath = elem->Get<std::string>("path", "").first;
      if (this->dataPtr->savePath.empty())
      {
        errors.push_back({ErrorCode::ELEMENT_INVALID,
            "Camera sensor frame saving enabled, but no path has been"
            "specified."});
      }
    }
  }

  // Load the noise values.
  if (_sdf->HasElement("noise"))
  {
    Errors noiseErr = this->dataPtr->imageNoise.Load(_sdf->GetElement("noise"));
    errors.insert(errors.end(), noiseErr.begin(), noiseErr.end());
  }


  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Camera::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
std::string Camera::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Camera::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
double Camera::HorizontalFov() const
{
  return this->dataPtr->hfov;
}

/////////////////////////////////////////////////
void Camera::SetHorizontalFov(double _hfov)
{
  this->dataPtr->hfov = _hfov;
}

//////////////////////////////////////////////////
uint32_t Camera::ImageWidth() const
{
  return this->dataPtr->imageWidth;
}

//////////////////////////////////////////////////
void Camera::SetImageWidth(uint32_t _width)
{
  this->dataPtr->imageWidth = _width;
}

//////////////////////////////////////////////////
uint32_t Camera::ImageHeight() const
{
  return this->dataPtr->imageHeight;
}

//////////////////////////////////////////////////
void Camera::SetImageHeight(uint32_t _height)
{
  this->dataPtr->imageHeight = _height;
}

//////////////////////////////////////////////////
ImageFormatType Camera::ImageFormat() const
{
  return this->dataPtr->imageFormat;
}

//////////////////////////////////////////////////
void Camera::SetImageFormat(ImageFormatType _format)
{
  this->dataPtr->imageFormat = _format;
}

//////////////////////////////////////////////////
double Camera::NearClip() const
{
  return this->dataPtr->nearClip;
}

//////////////////////////////////////////////////
void Camera::SetNearClip(double _near)
{
  this->dataPtr->nearClip = _near;
}

//////////////////////////////////////////////////
double Camera::FarClip() const
{
  return this->dataPtr->farClip;
}

//////////////////////////////////////////////////
void Camera::SetFarClip(double _far)
{
  this->dataPtr->farClip = _far;
}

//////////////////////////////////////////////////
bool Camera::SaveFrames() const
{
  return this->dataPtr->save;
}

//////////////////////////////////////////////////
void Camera::SetSaveFrames(bool _save)
{
  this->dataPtr->save = _save;
}

//////////////////////////////////////////////////
const std::string &Camera::SaveFramesPath() const
{
  return this->dataPtr->savePath;
}

//////////////////////////////////////////////////
void Camera::SetSaveFramesPath(const std::string &_path)
{
  this->dataPtr->savePath = _path;
}

//////////////////////////////////////////////////
Camera &Camera::operator=(const Camera &_camera)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new CameraPrivate;
  }
  *this->dataPtr = *_camera.dataPtr;
  return *this;
}

//////////////////////////////////////////////////
Camera &Camera::operator=(Camera &&_camera)
{
  this->dataPtr = _camera.dataPtr;
  _camera.dataPtr = nullptr;
  return *this;
}

//////////////////////////////////////////////////
bool Camera::operator==(const Camera &_cam) const
{
  return this->Name() == _cam.Name() &&
    this->HorizontalFov() == _cam.HorizontalFov() &&
    this->ImageWidth() == _cam.ImageWidth() &&
    this->ImageHeight() == _cam.ImageHeight() &&
    this->PixelFormat() == _cam.PixelFormat() &&
    this->NearClip() == _cam.NearClip() &&
    this->FarClip() == _cam.FarClip() &&
    this->SaveFames() == _cam.SaveFames() &&
    this->SaveFramesPath() == _cam.SaveFramesPath() &&
    this->ImageNoise() == _cam.ImageNoise();
}

//////////////////////////////////////////////////
bool Camera::operator!=(const Camera &_alt) const
{
  return !(*this == _alt);
}

//////////////////////////////////////////////////
const Noise &Camera::ImageNoise() const
{
  return this->dataPtr->imageNoise;
}

//////////////////////////////////////////////////
void Camera::SetImageNoise(const Noise &_noise)
{
  this->imageNoise = _noise;
}

//////////////////////////////////////////////////
double Camera::DistortionK1() const
{
  return this->distortionK1;
}

//////////////////////////////////////////////////
void Camera::SetDistortionK1(double _k1)
{
  this->distortionK1 = _k1;
}

//////////////////////////////////////////////////
double Camera::DistortionK2() const
{
  return this->distortionK2;
}

//////////////////////////////////////////////////
void Camera::SetDistortionK2(double _k2)
{
  this->distortionK2 = _k2;
}

//////////////////////////////////////////////////
double Camera::DistortionK3() const
{
  return this->distortionK3;
}

//////////////////////////////////////////////////
void Camera::SetDistortionK3(double _k3)
{
  this->distortionK3 = _k3;
}

//////////////////////////////////////////////////
double Camera::DistortionP1() const
{
  return this->distortionP1;
}

//////////////////////////////////////////////////
void Camera::DistortionP1(double _p1)
{
  this->p1 = _p1;
}

//////////////////////////////////////////////////
double Camera::DistortionP2() const
{
  return this->distortionP2;
}

//////////////////////////////////////////////////
void Camera::SetDistortionP2(double _p2)
{
  this->p2 = _p2;
}

//////////////////////////////////////////////////
const ignition::math::Vector2d &Camera::DistortionCenter() const
{
  return this->distortionCenter;
}

//////////////////////////////////////////////////
void Camera::SetDistortionCenter(
                const ignition::math::Vector2d &_center) const
{
  this->distortionCenter = _center;
}
