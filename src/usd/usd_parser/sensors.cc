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

#include "utils.hh"

namespace usd
{
  std::shared_ptr<sdf::Sensor> ParseSensors(
    const pxr::UsdPrim &_prim,
    const double _metersPerUnit,
    const std::string &_linkName)
  {
    std::shared_ptr<sdf::Sensor> sensor;
    sensor = std::make_shared<sdf::Sensor>();

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

      ignition::math::Pose3d pose;
      ignition::math::Vector3d scale(1, 1, 1);
      GetTransform(_prim, _metersPerUnit, pose, scale, _linkName);

      sensor->SetName(_prim.GetPath().GetName());
      camera.SetName(_prim.GetPath().GetName());
      camera.SetHorizontalFov(horizontalAperture);
      camera.SetLensFocalLength(focalLength);
      camera.SetRawPose(pose);
      camera.SetNearClip(clippingRange[0]);
      camera.SetFarClip(clippingRange[1]);
      camera.SetImageWidth(640);
      camera.SetImageHeight(480);
      camera.SetPixelFormat(sdf::PixelFormatType::RGB_INT8);
      sensor->SetCameraSensor(camera);
    }
    return sensor;
  }
}
