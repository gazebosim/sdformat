/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "sensors.hh"

#include "pxr/usd/usdGeom/camera.h"
#include "pxr/usd/usdGeom/gprim.h"

#include "sdf/Camera.hh"
#include "sdf/Lidar.hh"

#include "utils.hh"

namespace usd
{
  std::shared_ptr<sdf::Sensor> ParseSensors(
    const pxr::UsdPrim &_prim,
    USDData &_usdData,
    const std::string &_linkName)
  {
    std::shared_ptr<sdf::Sensor> sensor;
    sensor = std::make_shared<sdf::Sensor>();

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale(1, 1, 1);
    GetTransform(_prim, _usdData, pose, scale, _linkName);
    sensor->SetRawPose(pose);
    if(_prim.IsA<pxr::UsdGeomCamera>())
    {
      sensor->SetType(sdf::SensorType::CAMERA);

      sdf::Camera camera;

      auto variantCamera = pxr::UsdGeomCamera(_prim);
      float horizontalAperture = 20.955;
      float focalLength;
      pxr::GfVec2f clippingRange;
      // variantCamera.GetHorizontalApertureAttr().Get(&horizontalAperture);
      variantCamera.GetFocalLengthAttr().Get(&focalLength);
      variantCamera.GetClippingRangeAttr().Get(&clippingRange);

      sensor->SetName(_prim.GetPath().GetName());
      camera.SetName(_prim.GetPath().GetName());
      camera.SetHorizontalFov(horizontalAperture);
      camera.SetLensFocalLength(focalLength);
      ignition::math::Pose3d poseCamera(0, 0, 0, 1.57, 0, -1.57);
      camera.SetRawPose(pose * -poseCamera);
      camera.SetNearClip(clippingRange[0]);
      camera.SetFarClip(clippingRange[1]);
      camera.SetImageWidth(640);
      camera.SetImageHeight(480);
      camera.SetPixelFormat(sdf::PixelFormatType::RGB_INT8);
      sensor->SetCameraSensor(camera);
    }
    else if (std::string(_prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Lidar")
    {
      sensor->SetType(sdf::SensorType::LIDAR);

      sdf::Lidar lidar;
      sensor->SetName(_prim.GetPath().GetName());

      float hFOV;
      float hResolution;
      float vFOV;
      float vResolution;
      _prim.GetAttribute(pxr::TfToken("horizontalFov")).Get(&hFOV);
      _prim.GetAttribute(pxr::TfToken("horizontalResolution")).Get(&hResolution);
      _prim.GetAttribute(pxr::TfToken("verticalFov")).Get(&vFOV);
      _prim.GetAttribute(pxr::TfToken("verticalResolution")).Get(&vResolution);
      hResolution *= 3.1416/180;
      vResolution *= 3.1416/180;
      hFOV *= 3.1416/180;
      vFOV *= 3.1416/180;

      lidar.SetHorizontalScanMinAngle(ignition::math::Angle(-hFOV / 2));
      lidar.SetHorizontalScanMaxAngle(ignition::math::Angle(hFOV / 2));
      lidar.SetHorizontalScanResolution(1);
      lidar.SetHorizontalScanSamples(hFOV / hResolution);

      lidar.SetVerticalScanMinAngle(ignition::math::Angle(-vFOV / 2));
      lidar.SetVerticalScanMaxAngle(ignition::math::Angle(vFOV / 2));
      lidar.SetVerticalScanResolution(1);
      lidar.SetVerticalScanSamples(vFOV / vResolution);

      float minRange;
      float maxRange;
      _prim.GetAttribute(pxr::TfToken("minRange")).Get(&minRange);
      _prim.GetAttribute(pxr::TfToken("maxRange")).Get(&maxRange);

      lidar.SetRangeMin(minRange);
      lidar.SetRangeMax(maxRange);
      lidar.SetRangeResolution(0.1);

      sensor->SetLidarSensor(lidar);
    }
    return sensor;
  }
}
