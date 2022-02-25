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
 *
*/

#include "sdf/usd/sdf_parser/Sensor.hh"

#include <string>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/xform.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Camera.hh"
#include "sdf/Lidar.hh"
#include "sdf/Sensor.hh"
#include "../UsdUtils.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  /// \brief Create a USD camera from a SDF camera object.
  /// \param[in] _sensor The SDF camera object
  /// \param[in] _stage The stage that contains the definition of the USD
  /// camera
  /// \param[in] _path The path where the USD representation of _sensor should
  /// be defined in _stage
  /// \return UsdErrors, which is a list of UsdError objects. An empty list
  /// means there were no issues defining the USD representation of _sensor in
  /// _stage
  UsdErrors ParseSdfCameraSensor(const sdf::Sensor &_sensor,
    pxr::UsdStageRefPtr &_stage, const pxr::SdfPath &_path)
  {
    UsdErrors errors;

    auto usdCamera = pxr::UsdGeomCamera::Define(_stage, _path);
    if (!usdCamera)
    {
      errors.push_back(UsdError(
          sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
          "Unable to define a USD camera at path [" + _path.GetString() + "]"));
      return errors;
    }

    const auto sdfCamera = _sensor.CameraSensor();

    // TODO(adlarkin) check units to make sure they match (no documented
    // units for SDF)
    // When then focal length is not defined in SDF, the default value is 1
    if (!ignition::math::equal(sdfCamera->LensFocalLength(), 1.0))
    {
      usdCamera.CreateFocalLengthAttr().Set(
          static_cast<float>(sdfCamera->LensFocalLength()));
    }
    else
    {
      // The default value in USD is 50, but something more
      // similar to ignition Gazebo is 40.
      usdCamera.CreateFocalLengthAttr().Set(
          static_cast<float>(40.0f));
    }
    usdCamera.CreateClippingRangeAttr().Set(pxr::GfVec2f(
          static_cast<float>(sdfCamera->NearClip()),
          static_cast<float>(sdfCamera->FarClip())));

    usdCamera.CreateHorizontalApertureAttr().Set(
      static_cast<float>(sdfCamera->HorizontalFov().Degree()));
    return errors;
  }

  /// \brief Create a USD lidar sensor from a SDF lidar sensor object.
  /// \param[in] _sensor The SDF lidar sensor object
  /// \param[in] _stage The stage that contains the definition of the USD
  /// lidar sensor
  /// \param[in] _path The path where the USD representation of _sensor should
  /// be defined in _stage
  /// \return UsdErrors, which is a list of UsdError objects. An empty list
  /// means there were no issues defining the USD representation of _sensor in
  /// _stage
  UsdErrors ParseSdfLidarSensor(const sdf::Sensor &_sensor,
    pxr::UsdStageRefPtr &_stage, const pxr::SdfPath &_path)
  {
    UsdErrors errors;

    pxr::UsdGeomXform::Define(_stage, _path);
    auto lidarPrim = _stage->GetPrimAtPath(_path);
    if (!lidarPrim)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
            "Unable to find a lidar sensor prim at path ["
            + _path.GetString() + "]"));
      return errors;
    }

    const auto sdfLidar = _sensor.LidarSensor();

    lidarPrim.SetTypeName(pxr::TfToken("Lidar"));
    lidarPrim.CreateAttribute(pxr::TfToken("minRange"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(sdfLidar->RangeMin()));
    lidarPrim.CreateAttribute(pxr::TfToken("maxRange"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(sdfLidar->RangeMax()));
    const auto horizontalFov = sdfLidar->HorizontalScanMaxAngle() -
      sdfLidar->HorizontalScanMinAngle();
    lidarPrim.CreateAttribute(pxr::TfToken("horizontalFov"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(horizontalFov.Degree()));
    const auto verticalFov = sdfLidar->VerticalScanMaxAngle() -
      sdfLidar->VerticalScanMinAngle();
    lidarPrim.CreateAttribute(pxr::TfToken("verticalFov"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(verticalFov.Degree()));
    lidarPrim.CreateAttribute(pxr::TfToken("horizontalResolution"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(sdfLidar->HorizontalScanResolution()));
    lidarPrim.CreateAttribute(pxr::TfToken("verticalResolution"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(sdfLidar->VerticalScanResolution()));

    // TODO(adlarkin) incorporate SDF lidar's horizontal/samples and
    // vertical/samples values somehow? There is a "rotationRate"
    // attribute for the USD sensor, which might be related

    return errors;
  }

  /// \brief Create a USD IMU sensor
  /// \param[in] _stage The stage that contains the definition of the USD
  /// IMU sensor
  /// \param[in] _path The path where the USD IMU should be defined in _stage
  /// \return UsdErrors, which is a list of UsdError objects. An empty list
  /// means there were no issues defining the USD IMU sensor in _stage
  UsdErrors ParseSdfImuSensor(pxr::UsdStageRefPtr &_stage, pxr::SdfPath &_path)
  {
    UsdErrors errors;

    // for now, IMUs are defined as a cube geometry named "imu"
    // (there will be an IMU schema released upstream in the future).
    // It should be noted that the Carter robot example from isaac sim sample
    // assets has its IMU prim labeled with a "kind = model", but
    // https://graphics.pixar.com/usd/release/glossary.html#usdglossary-kind
    // says, “'model' is considered an abstract type and should not be assigned
    // as any prim’s kind." So, "kind = model" is not applied to the IMU prim
    // here.
    // TODO(adlarkin) update this code when an IMU schema is released
    _path = _path.ReplaceName(pxr::TfToken("imu"));
    auto usdCube = pxr::UsdGeomCube::Define(_stage, pxr::SdfPath(_path));
    if (!usdCube)
    {
      errors.push_back(
          UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
            "Unable to define an IMU at path [" + _path.GetString() + "]"));
      return errors;
    }
    // Set the size of the box very small
    usdCube.CreateSizeAttr().Set(0.001);

    return errors;
  }

  UsdErrors ParseSdfSensor(const sdf::Sensor &_sensor,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;
    pxr::SdfPath sdfSensorPath(_path);

    switch (_sensor.Type())
    {
      case sdf::SensorType::CAMERA:
        errors = ParseSdfCameraSensor(_sensor, _stage, sdfSensorPath);
        break;
      case sdf::SensorType::LIDAR:
      case sdf::SensorType::GPU_LIDAR:
        errors = ParseSdfLidarSensor(_sensor, _stage, sdfSensorPath);
        break;
      case sdf::SensorType::IMU:
        errors = ParseSdfImuSensor(_stage, sdfSensorPath);
        break;
      case sdf::SensorType::CONTACT:
        // TODO(adlarkin) figure out how to convert contact sensor. I found the
        // following docs, but they seem to require isaac sim specific packages:
        //  https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.contact_sensor/docs/index.html
      default:
        errors.push_back(
          UsdError(sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "This type of sensor is not supported"));
    }

    if (errors.empty())
    {
      ignition::math::Pose3d pose;
      auto poseErrors = sdf::usd::PoseWrtParent(_sensor, pose);
      if (!poseErrors.empty())
      {
        errors.insert(errors.end(), poseErrors.begin(), poseErrors.end());
        return errors;
      }

      if (_sensor.Type() == sdf::SensorType::CAMERA)
      {
        // Camera sensors are upAxis equal to "Y", we need to rotate the camera
        // properly.
        const ignition::math::Pose3d poseCamera(0, 0, 0, 1.57, 0, -1.57);
        usd::SetPose(
          pose * poseCamera, _stage, sdfSensorPath);
      }
      else
      {
        usd::SetPose(pose, _stage, sdfSensorPath);
      }
    }
    return errors;
  }
}
}
}
