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

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
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
#include "sdf/usd/sdf_parser/Utils.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  bool ParseSdfCameraSensor(const sdf::Sensor &_sensor,
    pxr::UsdStageRefPtr &_stage, const pxr::SdfPath &_path)
  {
    auto usdCamera = pxr::UsdGeomCamera::Define(_stage, _path);

    auto sdfCamera = _sensor.CameraSensor();

    // TODO(adlarkin) check units to make sure they match (no documented
    // units for SDF)
    // When then focal length is not defined in the SDF the default value is 1,
    // The following condition adapt this value to USD.
    if (!ignition::math::equal(sdfCamera->LensFocalLength(), 1.0))
    {
      usdCamera.CreateFocalLengthAttr().Set(
          static_cast<float>(sdfCamera->LensFocalLength()));
    }
    else
    {
      // TODO(ahcorde): The default value in USD is 50, but something more
      // similar to ignition Gazebo is 40.
      usdCamera.CreateFocalLengthAttr().Set(
          static_cast<float>(40.0f));
    }
    usdCamera.CreateClippingRangeAttr().Set(pxr::GfVec2f(
          static_cast<float>(sdfCamera->NearClip()),
          static_cast<float>(sdfCamera->FarClip())));

    usdCamera.CreateHorizontalApertureAttr().Set(
      static_cast<float>(sdfCamera->HorizontalFov().Degree()));
    return true;
  }

  bool ParseSdfLidarSensor(const sdf::Sensor &_sensor,
    pxr::UsdStageRefPtr &_stage, const pxr::SdfPath &_path)
  {
    pxr::UsdGeomXform::Define(
      _stage,
      pxr::SdfPath(_path.GetString() + "/sensor"));
    auto lidarPrim = _stage->GetPrimAtPath(
      pxr::SdfPath(_path.GetString() + "/sensor"));

    auto sdfLidar = _sensor.LidarSensor();

    lidarPrim.SetTypeName(pxr::TfToken("Lidar"));
    lidarPrim.CreateAttribute(pxr::TfToken("minRange"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(sdfLidar->RangeMin()));
    lidarPrim.CreateAttribute(pxr::TfToken("maxRange"),
        pxr::SdfValueTypeNames->Float, false).Set(
          static_cast<float>(sdfLidar->RangeMax()));
    const auto horizontalFov = sdfLidar->HorizontalScanMaxAngle() -
      sdfLidar->HorizontalScanMinAngle();
    // TODO(adlarkin) double check if these FOV calculations are correct
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
    // attribute for the USD sensor, I wonder if these are all related somehow

    return true;
  }

  bool ParseSdfImuSensor(const sdf::Sensor &/*_sensor*/,
    pxr::UsdStageRefPtr &_stage, pxr::SdfPath &_path)
  {
    // for now, IMUs are defined as a cube geometry named "imu"
    // (there will be an IMU schema released upstream in the future).
    // It should be noted that the Carter robot example from isaac sim sample
    // assets has its IMU prim labeled with a "kind = model", but
    // https://graphics.pixar.com/usd/release/glossary.html#usdglossary-kind
    // says, “'model' is considered an abstract type and should not be assigned as
    // any prim’s kind." So, "kind = model" is not applied to the IMU prim here.
    // TODO(adlarkin) update this code when an IMU schema is released
    _path = _path.ReplaceName(pxr::TfToken("imu"));
    auto usdCube = pxr::UsdGeomCube::Define(_stage, pxr::SdfPath(_path));
    // Set the size of the box very small
    usdCube.CreateSizeAttr().Set(0.001);

    return true;
  }

  sdf::Errors ParseSdfSensor(const sdf::Sensor &_sensor,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    sdf::Errors errors;
    pxr::SdfPath sdfSensorPath(_path);

    bool typeParsed = false;
    switch (_sensor.Type())
    {
      case sdf::SensorType::CAMERA:
        typeParsed = ParseSdfCameraSensor(_sensor, _stage, sdfSensorPath);
        break;
      case sdf::SensorType::LIDAR:
      case sdf::SensorType::GPU_LIDAR:
        typeParsed = ParseSdfLidarSensor(_sensor, _stage, sdfSensorPath);
        break;
      case sdf::SensorType::IMU:
        typeParsed = ParseSdfImuSensor(_sensor, _stage, sdfSensorPath);
        break;
      case sdf::SensorType::CONTACT:
        // TODO(adlarkin) figure out how to convert contact sensor. I found the
        // following docs, but they seem to require isaac sim specific packages:
        //  https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.contact_sensor/docs/index.html
      default:
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
          "Sensor type is either invalid or not supported"));
    }

    if (typeParsed)
    {
      if (_sensor.Type() == sdf::SensorType::CAMERA)
      {
        // Camera sensors are upAxis equal to "Y", we need to rotate the camera
        // properly.
        ignition::math::Pose3d poseCamera(0, 0, 0, 1.57, 0, -1.57);
        usd::SetPose(
          usd::PoseWrtParent(_sensor) * poseCamera, _stage, sdfSensorPath);
      }
      else
      {
        usd::SetPose(usd::PoseWrtParent(_sensor), _stage, sdfSensorPath);
      }
    }
    return errors;
  }
}
}
}
