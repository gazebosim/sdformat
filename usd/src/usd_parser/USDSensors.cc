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

#include "USDSensors.hh"

#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include "pxr/usd/usdGeom/camera.h"
#include "pxr/usd/usdGeom/gprim.h"
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDTransforms.hh"

#include "sdf/Camera.hh"
#include "sdf/Lidar.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
  sdf::Sensor ParseSensors(
    const pxr::UsdPrim &_prim,
    const USDData &_usdData)
  {
    sdf::Sensor sensor;

    const std::string primType =
      _prim.GetPrimTypeInfo().GetTypeName().GetText();

    gz::math::Pose3d pose;
    gz::math::Vector3d scale(1, 1, 1);
    GetTransform(
      _prim,
      _usdData,
      pose,
      scale,
      std::string(_prim.GetParent().GetPath().GetText()));

    if (_prim.IsA<pxr::UsdGeomCamera>())
    {
      sensor.SetType(sdf::SensorType::CAMERA);

      sdf::Camera camera;

      auto variantCamera = pxr::UsdGeomCamera(_prim);
      float focalLength;
      pxr::GfVec2f clippingRange;
      variantCamera.GetFocalLengthAttr().Get(&focalLength);
      variantCamera.GetClippingRangeAttr().Get(&clippingRange);

      sensor.SetName(_prim.GetPath().GetName());
      camera.SetName(_prim.GetPath().GetName());
      camera.SetHorizontalFov(20.955);
      camera.SetLensFocalLength(focalLength);
      // Camera is Y up axis, rotate the camera to match with Gazebo
      gz::math::Pose3d poseCamera(0, 0, 0, GZ_PI_2, 0, -GZ_PI_2);
      sensor.SetRawPose(pose * -poseCamera);
      camera.SetNearClip(clippingRange[0]);
      camera.SetFarClip(clippingRange[1]);
      camera.SetImageWidth(640);
      camera.SetImageHeight(480);
      camera.SetPixelFormat(sdf::PixelFormatType::RGB_INT8);
      sensor.SetCameraSensor(camera);
    }
    else if (primType == "Lidar")
    {
      sensor.SetType(sdf::SensorType::GPU_LIDAR);

      sdf::Lidar lidar;
      sensor.SetName(_prim.GetPath().GetName());
      sensor.SetRawPose(pose);

      float hFOV;
      float hResolution;
      float vFOV;
      float vResolution;
      _prim.GetAttribute(pxr::TfToken("horizontalFov")).Get(&hFOV);
      _prim.GetAttribute(
        pxr::TfToken("horizontalResolution")).Get(&hResolution);
      _prim.GetAttribute(pxr::TfToken("verticalFov")).Get(&vFOV);
      _prim.GetAttribute(pxr::TfToken("verticalResolution")).Get(&vResolution);
      hResolution = GZ_DTOR(hResolution);
      vResolution = GZ_DTOR(vResolution);
      hFOV = GZ_DTOR(hFOV);
      vFOV = GZ_DTOR(vFOV);

      lidar.SetHorizontalScanMinAngle(gz::math::Angle(-hFOV / 2));
      lidar.SetHorizontalScanMaxAngle(gz::math::Angle(hFOV / 2));
      lidar.SetHorizontalScanResolution(1);
      lidar.SetHorizontalScanSamples(hFOV / hResolution);

      lidar.SetVerticalScanMinAngle(gz::math::Angle(-vFOV / 2));
      lidar.SetVerticalScanMaxAngle(gz::math::Angle(vFOV / 2));
      lidar.SetVerticalScanResolution(1);
      lidar.SetVerticalScanSamples(vFOV / vResolution);

      float minRange;
      float maxRange;
      _prim.GetAttribute(pxr::TfToken("minRange")).Get(&minRange);
      _prim.GetAttribute(pxr::TfToken("maxRange")).Get(&maxRange);

      lidar.SetRangeMin(minRange);
      lidar.SetRangeMax(maxRange);
      lidar.SetRangeResolution(0.1);

      sensor.SetLidarSensor(lidar);
      sensor.SetUpdateRate(20.0);
    }
    return sensor;
  }
}
}
}
