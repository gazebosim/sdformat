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
#include <array>
#include <string_view>

#include "sdf/Camera.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief String names for the pixel formats.
/// \sa Image::PixelFormat.
constexpr std::array<const std::string_view, 19> kPixelFormatNames =
{
  "UNKNOWN_PIXEL_FORMAT",
  "L_INT8",
  "L_INT16",
  "RGB_INT8",
  "RGBA_INT8",
  "BGRA_INT8",
  "RGB_INT16",
  "RGB_INT32",
  "BGR_INT8",
  "BGR_INT16",
  "BGR_INT32",
  "R_FLOAT16",
  "RGB_FLOAT16",
  "R_FLOAT32",
  "RGB_FLOAT32",
  "BAYER_RGGB8",
  "BAYER_BGGR8",
  "BAYER_GBRG8",
  "BAYER_GRBG8"
};

// Private data class
class sdf::Camera::Implementation
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Camera info topic.
  public: std::string cameraInfoTopic = "";

  /// \brief Name of the camera.
  public: std::string name = "";

  /// \brief True if the camera is triggered by a topic.
  public: bool triggered{false};

  /// \brief Camera trigger topic.
  public: std::string triggerTopic = "";

  /// \brief Horizontal field of view.
  public: gz::math::Angle hfov{1.047};

  /// \brief Image width.
  public: uint32_t imageWidth{320};

  /// \brief Image height.
  public: uint32_t imageHeight{240};

  /// \brief Image format.
  public: PixelFormatType pixelFormat{PixelFormatType::RGB_INT8};

  /// \brief Anti-aliasing value.
  public: uint32_t antiAliasingValue{4};

  /// \brief Near clip distance.
  public: double nearClip{0.1};

  /// \brief Far clip distance.
  public: double farClip{100};

  /// \brief Near clip distance for depth camera.
  public: double depthNearClip{0.1};

  /// \brief Far clip distance for depth camera.
  public: double depthFarClip{10.0};

  /// \brief Segmentation type for segmentation camera.
  public: std::string segmentationType{"semantic"};

  /// \brief Boundingbox type for boundingbox camera.
  public: std::string boundingBoxType{"2d"};

  /// \brief True indicates the depth camera was set.
  public: bool hasDepthCamera{false};

  /// \brief True indicates the depth camera far clip distance was set.
  public: bool hasDepthFarClip{false};

  /// \brief True indicates the depth camera near clip distance was set.
  public: bool hasDepthNearClip{false};

  /// \brief True indicates the segmentation type was set.
  public: bool hasSegmentationType{false};

  /// \brief True indicates the boundingobx type was set.
  public: bool hasBoundingBoxType{false};

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
  public: gz::math::Vector2d distortionCenter{0.5, 0.5};

  /// \brief Pose of the link
  public: gz::math::Pose3d pose = gz::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief Frame ID the camera_info message header is expressed.
  public: std::string opticalFrameId{""};

  /// \brief Lens type.
  public: std::string lensType{"stereographic"};

  /// \brief Lens scale to hfov.
  public: bool lensScaleToHfov{true};

  /// \brief Lens c1.
  public: double lensC1{1.0};

  /// \brief Lens c2.
  public: double lensC2{1.0};

  /// \brief Lens c3.
  public: double lensC3{0.0};

  /// \brief Lens F.
  public: double lensF{1.0};

  /// \brief Lens fun.
  public: std::string lensFun{"tan"};

  /// \brief Lens cutoff angle.
  public: gz::math::Angle lensCutoffAngle{GZ_PI_2};

  /// \brief lens environment texture size.
  public: int lensEnvTextureSize{256};

  /// \brief lens intrinsics fx.
  public: double lensIntrinsicsFx{277.0};

  /// \brief lens intrinsics fy.
  public: double lensIntrinsicsFy{277.0};

  /// \brief lens intrinsics cx.
  public: double lensIntrinsicsCx{160.0};

  /// \brief lens intrinsics cy.
  public: double lensIntrinsicsCy{120.0};

  /// \brief lens projection fx
  public: double lensProjectionFx{277.0};

  /// \brief lens projection fy
  public: double lensProjectionFy{277.0};

  /// \brief lens projection cx
  public: double lensProjectionCx{160.0};

  /// \brief lens projection cy
  public: double lensProjectionCy{120.0};

  /// \brief lens projection tx
  public: double lensProjectionTx{0.0};

  /// \brief lens projection ty
  public: double lensProjectionTy{0.0};

  /// \brief lens intrinsics s.
  public: double lensIntrinsicsS{0.0};

  /// \brief True if this camera has custom intrinsics values
  public: bool hasIntrinsics = false;

  /// \brief True if this camera has custom projection values
  public: bool hasProjection = false;

  /// \brief Visibility mask of a camera. Defaults to 0xFFFFFFFF
  public: uint32_t visibilityMask{UINT32_MAX};
};

/////////////////////////////////////////////////
Camera::Camera()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
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

  this->dataPtr->triggered = _sdf->Get<bool>("triggered",
      this->dataPtr->triggered).first;

  this->dataPtr->triggerTopic = _sdf->Get<std::string>("trigger_topic",
      this->dataPtr->triggerTopic).first;

  this->dataPtr->cameraInfoTopic = _sdf->Get<std::string>("camera_info_topic",
      this->dataPtr->cameraInfoTopic).first;
  if (this->dataPtr->cameraInfoTopic == "__default__")
    this->dataPtr->cameraInfoTopic = "";

  this->dataPtr->hfov = _sdf->Get<gz::math::Angle>("horizontal_fov",
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

    this->dataPtr->distortionCenter = elem->Get<gz::math::Vector2d>(
        "center", this->dataPtr->distortionCenter).first;
  }

  if (_sdf->HasElement("image"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("image");
    this->dataPtr->imageWidth = elem->Get<uint32_t>("width",
        this->dataPtr->imageWidth).first;
    this->dataPtr->imageHeight = elem->Get<uint32_t>("height",
        this->dataPtr->imageHeight).first;

    std::string format = elem->Get<std::string>("format", "R8G8B8").first;
    this->dataPtr->pixelFormat = ConvertPixelFormat(format);
    if (this->dataPtr->pixelFormat == PixelFormatType::UNKNOWN_PIXEL_FORMAT)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Camera sensor <image><format> has invalid value of " + format});
    }

    this->dataPtr->antiAliasingValue =
        elem->Get<uint32_t>("anti_aliasing",
            this->dataPtr->antiAliasingValue).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Camera sensor is missing an <image> element."});
  }

  if (_sdf->HasElement("depth_camera"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("depth_camera");
    this->dataPtr->hasDepthCamera = true;
    if (elem->HasElement("clip"))
    {
      sdf::ElementPtr func = elem->GetElement("clip");
      if (func->HasElement("near"))
      {
        this->SetDepthNearClip(func->Get<double>("near"));
      }
      if (func->HasElement("far"))
      {
        this->SetDepthFarClip(func->Get<double>("far"));
      }
    }
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

  if (_sdf->HasElement("segmentation_type"))
  {
    this->SetSegmentationType(_sdf->Get<std::string>("segmentation_type"));
  }

  if (_sdf->HasElement("box_type"))
  {
    this->SetBoundingBoxType(_sdf->Get<std::string>("box_type"));
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
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  // Load the optional optical_frame_id value.
  if (_sdf->HasElement("optical_frame_id"))
  {
    this->dataPtr->opticalFrameId = _sdf->Get<std::string>("optical_frame_id",
        this->dataPtr->opticalFrameId).first;
  }

  // Load the lens values.
  if (_sdf->HasElement("lens"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("lens");

    this->dataPtr->lensType = elem->Get<std::string>("type",
        this->dataPtr->lensType).first;
    this->dataPtr->lensScaleToHfov = elem->Get<bool>("scale_to_hfov",
        this->dataPtr->lensScaleToHfov).first;
    this->dataPtr->lensCutoffAngle = elem->Get<gz::math::Angle>(
        "cutoff_angle", this->dataPtr->lensCutoffAngle).first;
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
      this->dataPtr->hasIntrinsics = true;
    }

    if (elem->HasElement("projection")) {
      sdf::ElementPtr projection = elem->GetElement("projection");
      this->dataPtr->lensProjectionFx = projection->Get<double>("p_fx",
          this->dataPtr->lensProjectionFx).first;
      this->dataPtr->lensProjectionFy = projection->Get<double>("p_fy",
          this->dataPtr->lensProjectionFy).first;
      this->dataPtr->lensProjectionCx = projection->Get<double>("p_cx",
          this->dataPtr->lensProjectionCx).first;
      this->dataPtr->lensProjectionCy = projection->Get<double>("p_cy",
          this->dataPtr->lensProjectionCy).first;
      this->dataPtr->lensProjectionTx = projection->Get<double>("tx",
          this->dataPtr->lensProjectionTx).first;
      this->dataPtr->lensProjectionTy = projection->Get<double>("ty",
          this->dataPtr->lensProjectionTy).first;
      this->dataPtr->hasProjection = true;
    }
  }

  if (_sdf->HasElement("visibility_mask"))
  {
    this->dataPtr->visibilityMask = _sdf->Get<uint32_t>("visibility_mask",
        this->dataPtr->visibilityMask).first;
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Camera::CameraInfoTopic() const
{
  return this->dataPtr->cameraInfoTopic;
}

/////////////////////////////////////////////////
void Camera::SetCameraInfoTopic(const std::string &_cameraInfoTopic)
{
  this->dataPtr->cameraInfoTopic = _cameraInfoTopic;
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
bool Camera::Triggered() const
{
  return this->dataPtr->triggered;
}


/////////////////////////////////////////////////
void Camera::SetTriggered(bool _triggered)
{
  this->dataPtr->triggered = _triggered;
}

/////////////////////////////////////////////////
std::string Camera::TriggerTopic() const
{
  return this->dataPtr->triggerTopic;
}


/////////////////////////////////////////////////
void Camera::SetTriggerTopic(const std::string &_triggerTopic)
{
  this->dataPtr->triggerTopic = _triggerTopic;
}

/////////////////////////////////////////////////
gz::math::Angle Camera::HorizontalFov() const
{
  return this->dataPtr->hfov;
}

/////////////////////////////////////////////////
void Camera::SetHorizontalFov(const gz::math::Angle &_hfov)
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
std::string Camera::PixelFormatStr() const
{
  return ConvertPixelFormat(this->dataPtr->pixelFormat);
}

//////////////////////////////////////////////////
void Camera::SetPixelFormatStr(const std::string &_fmt)
{
  this->dataPtr->pixelFormat = ConvertPixelFormat(_fmt);
}

//////////////////////////////////////////////////
uint32_t Camera::AntiAliasingValue() const
{
  return this->dataPtr->antiAliasingValue;
}

//////////////////////////////////////////////////
void Camera::SetAntiAliasingValue(uint32_t _antiAliasingValue)
{
  this->dataPtr->antiAliasingValue = _antiAliasingValue;
}

//////////////////////////////////////////////////
double Camera::DepthNearClip() const
{
  return this->dataPtr->depthNearClip;
}

//////////////////////////////////////////////////
void Camera::SetDepthNearClip(double _near)
{
  this->dataPtr->hasDepthNearClip = true;
  this->dataPtr->depthNearClip = _near;
}

//////////////////////////////////////////////////
double Camera::DepthFarClip() const
{
  return this->dataPtr->depthFarClip;
}

//////////////////////////////////////////////////
void Camera::SetDepthFarClip(double _far)
{
  this->dataPtr->hasDepthFarClip = true;
  this->dataPtr->depthFarClip = _far;
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
const std::string &Camera::SegmentationType() const
{
  return this->dataPtr->segmentationType;
}

//////////////////////////////////////////////////
void Camera::SetSegmentationType(const std::string &_type)
{
  this->dataPtr->hasSegmentationType = true;
  this->dataPtr->segmentationType = _type;
}

//////////////////////////////////////////////////
void Camera::SetHasSegmentationType(bool _type)
{
  this->dataPtr->hasSegmentationType = _type;
}

//////////////////////////////////////////////////
bool Camera::HasSegmentationType() const
{
  return this->dataPtr->hasSegmentationType;
}

//////////////////////////////////////////////////
const std::string &Camera::BoundingBoxType() const
{
  return this->dataPtr->boundingBoxType;
}

//////////////////////////////////////////////////
void Camera::SetBoundingBoxType(const std::string &_type)
{
  this->dataPtr->hasBoundingBoxType = true;
  this->dataPtr->boundingBoxType = _type;
}

//////////////////////////////////////////////////
void Camera::SetHasBoundingBoxType(bool _type)
{
  this->dataPtr->hasBoundingBoxType = _type;
}

//////////////////////////////////////////////////
bool Camera::HasBoundingBoxType() const
{
  return this->dataPtr->hasBoundingBoxType;
}

//////////////////////////////////////////////////
void Camera::SetHasDepthCamera(bool _camera)
{
  this->dataPtr->hasDepthCamera = _camera;
}

//////////////////////////////////////////////////
bool Camera::HasDepthCamera() const
{
  return this->dataPtr->hasDepthCamera;
}

//////////////////////////////////////////////////
void Camera::SetHasDepthNearClip(bool _near)
{
  this->dataPtr->hasDepthNearClip = _near;
}

//////////////////////////////////////////////////
bool Camera::HasDepthNearClip() const
{
  return this->dataPtr->hasDepthNearClip;
}

//////////////////////////////////////////////////
void Camera::SetHasDepthFarClip(bool _far)
{
  this->dataPtr->hasDepthFarClip = _far;
}

//////////////////////////////////////////////////
bool Camera::HasDepthFarClip() const
{
  return this->dataPtr->hasDepthFarClip;
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
bool Camera::operator==(const Camera &_cam) const
{

  // \todo(iche033) Remove in sdformat16
  GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  return this->Name() == _cam.Name() &&
    this->HorizontalFov() == _cam.HorizontalFov() &&
    this->ImageWidth() == _cam.ImageWidth() &&
    this->ImageHeight() == _cam.ImageHeight() &&
    this->PixelFormat() == _cam.PixelFormat() &&
    gz::math::equal(this->NearClip(), _cam.NearClip()) &&
    gz::math::equal(this->FarClip(), _cam.FarClip()) &&
    this->SaveFrames() == _cam.SaveFrames() &&
    this->SaveFramesPath() == _cam.SaveFramesPath() &&
    this->ImageNoise() == _cam.ImageNoise() &&
    this->VisibilityMask() == _cam.VisibilityMask() &&
    this->OpticalFrameId() == _cam.OpticalFrameId();
  GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
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
const gz::math::Vector2d &Camera::DistortionCenter() const
{
  return this->dataPtr->distortionCenter;
}

//////////////////////////////////////////////////
void Camera::SetDistortionCenter(const gz::math::Vector2d &_center)
{
  this->dataPtr->distortionCenter = _center;
}

/////////////////////////////////////////////////
const gz::math::Pose3d &Camera::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Camera::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Camera::SetRawPose(const gz::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Camera::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
const std::string Camera::OpticalFrameId() const
{
  return this->dataPtr->opticalFrameId;
}

/////////////////////////////////////////////////
void Camera::SetOpticalFrameId(const std::string &_frame)
{
  this->dataPtr->opticalFrameId = _frame;
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
const std::string &Camera::LensFunction() const
{
  return this->dataPtr->lensFun;
}

/////////////////////////////////////////////////
void Camera::SetLensFunction(const std::string &_fun)
{
  this->dataPtr->lensFun = _fun;
}

/////////////////////////////////////////////////
gz::math::Angle Camera::LensCutoffAngle() const
{
  return this->dataPtr->lensCutoffAngle;
}

/////////////////////////////////////////////////
void Camera::SetLensCutoffAngle(const gz::math::Angle &_angle)
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
  this->dataPtr->hasIntrinsics = true;
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
  this->dataPtr->hasIntrinsics = true;
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
  this->dataPtr->hasIntrinsics = true;
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
  this->dataPtr->hasIntrinsics = true;
}

/////////////////////////////////////////////////
double Camera::LensProjectionFx() const
{
  return this->dataPtr->lensProjectionFx;
}

/////////////////////////////////////////////////
void Camera::SetLensProjectionFx(double _fx_p)
{
  this->dataPtr->lensProjectionFx = _fx_p;
  this->dataPtr->hasProjection = true;
}

/////////////////////////////////////////////////
double Camera::LensProjectionFy() const
{
  return this->dataPtr->lensProjectionFy;
}

/////////////////////////////////////////////////
void Camera::SetLensProjectionFy(double _fy_p)
{
  this->dataPtr->lensProjectionFy = _fy_p;
  this->dataPtr->hasProjection = true;
}

/////////////////////////////////////////////////
double Camera::LensProjectionCx() const
{
  return this->dataPtr->lensProjectionCx;
}

/////////////////////////////////////////////////
void Camera::SetLensProjectionCx(double _cx_p)
{
  this->dataPtr->lensProjectionCx = _cx_p;
  this->dataPtr->hasProjection = true;
}

/////////////////////////////////////////////////
double Camera::LensProjectionCy() const
{
  return this->dataPtr->lensProjectionCy;
}

/////////////////////////////////////////////////
void Camera::SetLensProjectionCy(double _cy_p)
{
  this->dataPtr->lensProjectionCy = _cy_p;
  this->dataPtr->hasProjection = true;
}

/////////////////////////////////////////////////
double Camera::LensProjectionTx() const
{
  return this->dataPtr->lensProjectionTx;
}

/////////////////////////////////////////////////
void Camera::SetLensProjectionTx(double _tx)
{
  this->dataPtr->lensProjectionTx = _tx;
  this->dataPtr->hasProjection = true;
}

/////////////////////////////////////////////////
double Camera::LensProjectionTy() const
{
  return this->dataPtr->lensProjectionTy;
}

/////////////////////////////////////////////////
void Camera::SetLensProjectionTy(double _ty)
{
  this->dataPtr->lensProjectionTy = _ty;
  this->dataPtr->hasProjection = true;
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
  this->dataPtr->hasIntrinsics = true;
}

/////////////////////////////////////////////////
std::string Camera::ConvertPixelFormat(PixelFormatType _type)
{
  unsigned int index = static_cast<int>(_type);
  if (index < kPixelFormatNames.size())
    return std::string(kPixelFormatNames[static_cast<int>(_type)]);

  return std::string(kPixelFormatNames[0]);
}

/////////////////////////////////////////////////
PixelFormatType Camera::ConvertPixelFormat(const std::string &_format)
{
  for (unsigned int i = 0; i < kPixelFormatNames.size(); ++i)
  {
    if (kPixelFormatNames[i] == _format)
    {
      return static_cast<PixelFormatType>(i);
    }
  }

  // Handle older formats
  if (_format == "R8G8B8")
    return PixelFormatType::RGB_INT8;
  else if (_format == "L8")
    return PixelFormatType::L_INT8;
  else if (_format == "L16")
    return PixelFormatType::L_INT16;
  else if (_format == "B8G8R8")
    return PixelFormatType::BGR_INT8;
  else if (_format == "BAYER_RGGB8")
    return PixelFormatType::BAYER_RGGB8;
  else if (_format == "BAYER_BGGR8")
    return PixelFormatType::BAYER_BGGR8;
  else if (_format == "BAYER_GBRG8")
    return PixelFormatType::BAYER_GBRG8;
  else if (_format == "BAYER_GRBG8")
    return PixelFormatType::BAYER_GRBG8;

  return PixelFormatType::UNKNOWN_PIXEL_FORMAT;
}

/////////////////////////////////////////////////
uint32_t Camera::VisibilityMask() const
{
  return this->dataPtr->visibilityMask;
}

/////////////////////////////////////////////////
void Camera::SetVisibilityMask(uint32_t _mask)
{
  this->dataPtr->visibilityMask = _mask;
}

/////////////////////////////////////////////////
bool Camera::HasLensIntrinsics() const
{
  return this->dataPtr->hasIntrinsics;
}

/////////////////////////////////////////////////
bool Camera::HasLensProjection() const
{
  return this->dataPtr->hasProjection;
}

/////////////////////////////////////////////////
sdf::ElementPtr Camera::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("camera.sdf", elem);

  elem->GetAttribute("name")->Set<std::string>(this->Name());
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<gz::math::Pose3d>(this->RawPose());
  elem->GetElement("horizontal_fov")->Set<double>(
      this->HorizontalFov().Radian());
  sdf::ElementPtr imageElem = elem->GetElement("image");
  imageElem->GetElement("width")->Set<double>(this->ImageWidth());
  imageElem->GetElement("height")->Set<double>(this->ImageHeight());
  imageElem->GetElement("format")->Set<std::string>(this->PixelFormatStr());
  imageElem->GetElement("anti_aliasing")->Set<uint32_t>(
      this->AntiAliasingValue());
  elem->GetElement("camera_info_topic")->Set<std::string>(
      this->CameraInfoTopic());
  elem->GetElement("trigger_topic")->Set<std::string>(
      this->TriggerTopic());
  elem->GetElement("triggered")->Set<bool>(
      this->Triggered());

  sdf::ElementPtr clipElem = elem->GetElement("clip");
  clipElem->GetElement("near")->Set<double>(this->NearClip());
  clipElem->GetElement("far")->Set<double>(this->FarClip());

  if (this->dataPtr->hasDepthCamera)
  {
    sdf::ElementPtr depthElem = elem->GetElement("depth_camera");
    sdf::ElementPtr depthClipElem = depthElem->GetElement("clip");
    depthClipElem->GetElement("near")->Set<double>(this->DepthNearClip());
    depthClipElem->GetElement("far")->Set<double>(this->DepthFarClip());
  }

  sdf::ElementPtr saveElem = elem->GetElement("save");
  saveElem->GetAttribute("enabled")->Set<bool>(this->SaveFrames());
  if (this->SaveFrames())
    saveElem->GetElement("path")->Set<std::string>(this->SaveFramesPath());
  elem->GetElement("visibility_mask")->Set<uint32_t>(
      this->VisibilityMask());

  sdf::ElementPtr noiseElem = elem->GetElement("noise");
  std::string noiseType;
  switch (this->dataPtr->imageNoise.Type())
  {
    case sdf::NoiseType::NONE:
      noiseType = "none";
      break;
    case sdf::NoiseType::GAUSSIAN:
      noiseType = "gaussian";
      break;
    case sdf::NoiseType::GAUSSIAN_QUANTIZED:
      noiseType = "gaussian_quantized";
      break;
    default:
      noiseType = "none";
  }

  // camera does not use noise.sdf description
  noiseElem->GetElement("type")->Set<std::string>(noiseType);
  noiseElem->GetElement("mean")->Set<double>(this->dataPtr->imageNoise.Mean());
  noiseElem->GetElement("stddev")->Set<double>(
      this->dataPtr->imageNoise.StdDev());

  sdf::ElementPtr distortionElem = elem->GetElement("distortion");
  distortionElem->GetElement("k1")->Set<double>(this->DistortionK1());
  distortionElem->GetElement("k2")->Set<double>(this->DistortionK2());
  distortionElem->GetElement("k3")->Set<double>(this->DistortionK3());
  distortionElem->GetElement("p1")->Set<double>(this->DistortionP1());
  distortionElem->GetElement("p2")->Set<double>(this->DistortionP2());
  distortionElem->GetElement("center")->Set<gz::math::Vector2d>(
      this->DistortionCenter());

  sdf::ElementPtr lensElem = elem->GetElement("lens");
  lensElem->GetElement("type")->Set<std::string>(this->LensType());
  lensElem->GetElement("scale_to_hfov")->Set<bool>(this->LensScaleToHfov());
  lensElem->GetElement("cutoff_angle")->Set<double>(
     this->LensCutoffAngle().Radian());
  lensElem->GetElement("env_texture_size")->Set<double>(
      this->LensEnvironmentTextureSize());
  if (this->LensType() == "custom")
  {
    sdf::ElementPtr customLensElem = lensElem->GetElement("custom_function");
    customLensElem->GetElement("c1")->Set<double>(this->LensC1());
    customLensElem->GetElement("c2")->Set<double>(this->LensC2());
    customLensElem->GetElement("c3")->Set<double>(this->LensC3());
    customLensElem->GetElement("f")->Set<double>(this->LensFocalLength());
    customLensElem->GetElement("fun")->Set<std::string>(
        this->LensFunction());
  }
  if (this->HasLensIntrinsics())
  {
    sdf::ElementPtr intrinsicsElem = lensElem->GetElement("intrinsics");
    intrinsicsElem->GetElement("fx")->Set<double>(this->LensIntrinsicsFx());
    intrinsicsElem->GetElement("fy")->Set<double>(this->LensIntrinsicsFy());
    intrinsicsElem->GetElement("cx")->Set<double>(this->LensIntrinsicsCx());
    intrinsicsElem->GetElement("cy")->Set<double>(this->LensIntrinsicsCy());
    intrinsicsElem->GetElement("s")->Set<double>(this->LensIntrinsicsSkew());
  }
  if (this->HasLensProjection())
  {
    sdf::ElementPtr projectionElem = lensElem->GetElement("projection");
    projectionElem->GetElement("p_fx")->Set<double>(this->LensProjectionFx());
    projectionElem->GetElement("p_fy")->Set<double>(this->LensProjectionFy());
    projectionElem->GetElement("p_cx")->Set<double>(this->LensProjectionCx());
    projectionElem->GetElement("p_cy")->Set<double>(this->LensProjectionCy());
    projectionElem->GetElement("tx")->Set<double>(this->LensProjectionTx());
    projectionElem->GetElement("ty")->Set<double>(this->LensProjectionTy());
  }

  if (this->HasSegmentationType())
  {
    elem->GetElement("segmentation_type")->Set<std::string>(
        this->SegmentationType());
  }

  // \todo(iche033) Remove in sdformat16
  GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
  elem->GetElement("optical_frame_id")->Set<std::string>(
      this->OpticalFrameId());
  GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION

  return elem;
}
