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
  public: PixelFormatType pixelFormat{PixelFormatType::RGB_INT8};

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

  /// \brief Pose of the link
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseFrame = "";

  public: std::string lensType{"stereographic"};
  public: bool lensScaleToHfov{true};
  public: double lensC1{1.0};
  public: double lensC2{1.0};
  public: double lensC3{0.0};
  public: double lensF{1.0};
  public: std::string lensFun{"tan"};

  public: double lensCutoffAngle{IGN_PI};
  public: int lensEnvTextureSize{256};

  public: double lensIntrinsicsFx{277.0};
  public: double lensIntrinsicsFy{277.0};
  public: double lensIntrinsicsCx{160.0};
  public: double lensIntrinsicsCy{120.0};
  public: double lensIntrinsicsS{1.0};
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
Camera::Camera(Camera &&_camera) noexcept
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

  // Read the camera's name
  loadName(_sdf, this->dataPtr->name);

  this->dataPtr->hfov = _sdf->Get<double>("horizontal_fov",
      this->dataPtr->hfov).first;

  // Read the distortion
  if (_sdf->HasElement("distortion"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("distortion");
    this->dataPtr->distortionK1 = elem->Get<double>("k1",
      this->dataPtr->distortionK1).first;
    this->dataPtr->distortionK2 = elem->Get<double>("k2",
      this->dataPtr->distortionK2).first;
    this->dataPtr->distortionK3 = elem->Get<double>("k3",
      this->dataPtr->distortionK3).first;

    this->dataPtr->distortionP1 = elem->Get<double>("p1",
      this->dataPtr->distortionP1).first;
    this->dataPtr->distortionP2 = elem->Get<double>("p2",
      this->dataPtr->distortionP2).first;

    this->dataPtr->distortionCenter = elem->Get<ignition::math::Vector2d>(
        "center", this->dataPtr->distortionCenter).first;
  }

  if (_sdf->HasElement("image"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("image");
    this->dataPtr->imageWidth = elem->Get<double>("width",
        this->dataPtr->imageWidth).first;
    this->dataPtr->imageHeight = elem->Get<double>("height",
        this->dataPtr->imageHeight).first;
    std::string format = elem->Get<std::string>("format", "R8G8B8").first;
    if (format == "R8G8B8")
      this->dataPtr->pixelFormat = PixelFormatType::RGB_INT8;
    else if (format == "L8")
      this->dataPtr->pixelFormat = PixelFormatType::L_INT8;
    else if (format == "B8G8R8")
      this->dataPtr->pixelFormat = PixelFormatType::BGR_INT8;
    else if (format == "BAYER_RGGB8")
      this->dataPtr->pixelFormat = PixelFormatType::BAYER_RGGB8;
    else if (format == "BAYER_BGGR8")
      this->dataPtr->pixelFormat = PixelFormatType::BAYER_BGGR8;
    else if (format == "BAYER_GBRG8")
      this->dataPtr->pixelFormat = PixelFormatType::BAYER_GBRG8;
    else if (format == "BAYER_GRBG8")
      this->dataPtr->pixelFormat = PixelFormatType::BAYER_GRBG8;
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

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseFrame);

  // Load the lens values.
  if (_sdf->HasElement("lens"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("lens");

    this->dataPtr->lensType = elem->Get<std::string>("type",
        this->dataPtr->lensType).first;
    this->dataPtr->lensScaleToHfov = elem->Get<bool>("scale_to_hfov",
        this->dataPtr->lensScaleToHfov).first;
    this->dataPtr->lensCutoffAngle = elem->Get<double>("cutoff_angle",
        this->dataPtr->lensCutoffAngle).first;
    this->dataPtr->lensEnvTextureSize = elem->Get<int>("env_texture_size",
        this->dataPtr->lensEnvTextureSize).first;

    if (elem->HasElement("custom_function"))
    {
      sdf::ElementPtr func = elem->GetElement("custom_function");
      this->dataPtr->lensC1 = func->Get<double>("c1",
          this->dataPtr->lensC1).first;
      this->dataPtr->lensC2 = func->Get<double>("c2",
          this->dataPtr->lensC2).first;
      this->dataPtr->lensC3 = func->Get<double>("c3",
          this->dataPtr->lensC3).first;
      this->dataPtr->lensF = func->Get<double>("f",
          this->dataPtr->lensF).first;
      this->dataPtr->lensFun = func->Get<std::string>("fun",
          this->dataPtr->lensFun).first;
    }

    if (elem->HasElement("intrinsics"))
    {
      sdf::ElementPtr intrinsics = elem->GetElement("intrinsics");
      this->dataPtr->lensIntrinsicsFx = intrinsics->Get<double>("fx",
          this->dataPtr->lensIntrinsicsFx).first;
      this->dataPtr->lensIntrinsicsFy = intrinsics->Get<double>("fy",
          this->dataPtr->lensIntrinsicsFy).first;
      this->dataPtr->lensIntrinsicsCx = intrinsics->Get<double>("cx",
          this->dataPtr->lensIntrinsicsCx).first;
      this->dataPtr->lensIntrinsicsCy = intrinsics->Get<double>("cy",
          this->dataPtr->lensIntrinsicsCy).first;
      this->dataPtr->lensIntrinsicsS = intrinsics->Get<double>("s",
          this->dataPtr->lensIntrinsicsS).first;
    }
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
PixelFormatType Camera::PixelFormat() const
{
  return this->dataPtr->pixelFormat;
}

//////////////////////////////////////////////////
void Camera::SetPixelFormat(PixelFormatType _format)
{
  this->dataPtr->pixelFormat = _format;
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
Camera &Camera::operator=(Camera &&_camera) noexcept
{
  this->dataPtr = _camera.dataPtr;
  _camera.dataPtr = nullptr;
  return *this;
}

//////////////////////////////////////////////////
bool Camera::operator==(const Camera &_cam) const
{
  return this->Name() == _cam.Name() &&
    ignition::math::equal(this->HorizontalFov(), _cam.HorizontalFov()) &&
    this->ImageWidth() == _cam.ImageWidth() &&
    this->ImageHeight() == _cam.ImageHeight() &&
    this->PixelFormat() == _cam.PixelFormat() &&
    ignition::math::equal(this->NearClip(), _cam.NearClip()) &&
    ignition::math::equal(this->FarClip(), _cam.FarClip()) &&
    this->SaveFrames() == _cam.SaveFrames() &&
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
  this->dataPtr->imageNoise = _noise;
}

//////////////////////////////////////////////////
double Camera::DistortionK1() const
{
  return this->dataPtr->distortionK1;
}

//////////////////////////////////////////////////
void Camera::SetDistortionK1(double _k1)
{
  this->dataPtr->distortionK1 = _k1;
}

//////////////////////////////////////////////////
double Camera::DistortionK2() const
{
  return this->dataPtr->distortionK2;
}

//////////////////////////////////////////////////
void Camera::SetDistortionK2(double _k2)
{
  this->dataPtr->distortionK2 = _k2;
}

//////////////////////////////////////////////////
double Camera::DistortionK3() const
{
  return this->dataPtr->distortionK3;
}

//////////////////////////////////////////////////
void Camera::SetDistortionK3(double _k3)
{
  this->dataPtr->distortionK3 = _k3;
}

//////////////////////////////////////////////////
double Camera::DistortionP1() const
{
  return this->dataPtr->distortionP1;
}

//////////////////////////////////////////////////
void Camera::SetDistortionP1(double _p1)
{
  this->dataPtr->distortionP1 = _p1;
}

//////////////////////////////////////////////////
double Camera::DistortionP2() const
{
  return this->dataPtr->distortionP2;
}

//////////////////////////////////////////////////
void Camera::SetDistortionP2(double _p2)
{
  this->dataPtr->distortionP2 = _p2;
}

//////////////////////////////////////////////////
const ignition::math::Vector2d &Camera::DistortionCenter() const
{
  return this->dataPtr->distortionCenter;
}

//////////////////////////////////////////////////
void Camera::SetDistortionCenter(
                const ignition::math::Vector2d &_center) const
{
  this->dataPtr->distortionCenter = _center;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Camera::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Camera::PoseFrame() const
{
  return this->dataPtr->poseFrame;
}

/////////////////////////////////////////////////
void Camera::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Camera::SetPoseFrame(const std::string &_frame)
{
  this->dataPtr->poseFrame = _frame;
}

/////////////////////////////////////////////////
std::string Camera::LensType() const
{
  return this->dataPtr->lensType;
}

/////////////////////////////////////////////////
void Camera::SetLensType(const std::string &_type)
{
  this->dataPtr->lensType = _type;
}

/////////////////////////////////////////////////
bool Camera::LensScaleToHfov() const
{
  return this->dataPtr->lensScaleToHfov;
}

/////////////////////////////////////////////////
void Camera::SetLensScaleToHfov(bool _scale)
{
  this->dataPtr->lensScaleToHfov = _scale;
}

/////////////////////////////////////////////////
double Camera::LensC1() const
{
  return this->dataPtr->lensC1;
}

/////////////////////////////////////////////////
void Camera::SetLensC1(double _c1)
{
  this->dataPtr->lensC1 = _c1;
}

/////////////////////////////////////////////////
double Camera::LensC2() const
{
  return this->dataPtr->lensC2;
}

/////////////////////////////////////////////////
void Camera::SetLensC2(double _c2)
{
  this->dataPtr->lensC2 = _c2;
}

/////////////////////////////////////////////////
double Camera::LensC3() const
{
  return this->dataPtr->lensC3;
}

/////////////////////////////////////////////////
void Camera::SetLensC3(double _c3)
{
  this->dataPtr->lensC3 = _c3;
}

/////////////////////////////////////////////////
double Camera::LensFocalLength() const
{
  return this->dataPtr->lensF;
}

/////////////////////////////////////////////////
void Camera::SetLensFocalLength(double _f)
{
  this->dataPtr->lensF = _f;
}

/////////////////////////////////////////////////
const std::string &Camera::LensFun() const
{
  return this->dataPtr->lensFun;
}

/////////////////////////////////////////////////
void Camera::SetLensFun(const std::string &_fun)
{
  this->dataPtr->lensFun = _fun;
}

/////////////////////////////////////////////////
double Camera::LensCutoffAngle() const
{
  return this->dataPtr->lensCutoffAngle;
}

/////////////////////////////////////////////////
void Camera::SetLensCutoffAngle(double _angle)
{
  this->dataPtr->lensCutoffAngle = _angle;
}

/////////////////////////////////////////////////
int Camera::LensEnvironmentTextureSize() const
{
  return this->dataPtr->lensEnvTextureSize;
}

/////////////////////////////////////////////////
void Camera::SetLensEnvironmentTextureSize(int _size)
{
  this->dataPtr->lensEnvTextureSize = _size;
}

/////////////////////////////////////////////////
double Camera::LensIntrinsicsFx() const
{
  return this->dataPtr->lensIntrinsicsFx;
}

/////////////////////////////////////////////////
void Camera::SetLensIntrinsicsFx(double _fx)
{
  this->dataPtr->lensIntrinsicsFx = _fx;
}

/////////////////////////////////////////////////
double Camera::LensIntrinsicsFy() const
{
  return this->dataPtr->lensIntrinsicsFy;
}

/////////////////////////////////////////////////
void Camera::SetLensIntrinsicsFy(double _fy)
{
  this->dataPtr->lensIntrinsicsFy = _fy;
}

/////////////////////////////////////////////////
double Camera::LensIntrinsicsCx() const
{
  return this->dataPtr->lensIntrinsicsCx;
}

/////////////////////////////////////////////////
void Camera::SetLensIntrinsicsCx(double _cx)
{
  this->dataPtr->lensIntrinsicsCx = _cx;
}

/////////////////////////////////////////////////
double Camera::LensIntrinsicsCy() const
{
  return this->dataPtr->lensIntrinsicsCy;
}

/////////////////////////////////////////////////
void Camera::SetLensIntrinsicsCy(double _cy)
{
  this->dataPtr->lensIntrinsicsCy = _cy;
}

/////////////////////////////////////////////////
double Camera::LensIntrinsicsSkew() const
{
  return this->dataPtr->lensIntrinsicsS;
}

/////////////////////////////////////////////////
void Camera::SetLensIntrinsicsSkew(double _s)
{
  this->dataPtr->lensIntrinsicsS = _s;
}
