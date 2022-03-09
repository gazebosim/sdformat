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

#include "Lights.hh"

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include "pxr/usd/usdLux/lightAPI.h"
#include "pxr/usd/usdLux/boundableLightBase.h"
#include "pxr/usd/usdLux/distantLight.h"
#include "pxr/usd/usdLux/diskLight.h"
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDTransforms.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  std::shared_ptr<sdf::Light> ParseLights(
    const pxr::UsdPrim &_prim,
    const std::string &_linkName)
  {
    std::shared_ptr<sdf::Light> light;
    light = std::make_shared<sdf::Light>();

    auto variantLight = pxr::UsdLuxBoundableLightBase(_prim);

    if (_prim.IsA<pxr::UsdLuxDistantLight>())
    {
      light->SetType(sdf::LightType::DIRECTIONAL);
    }
    else if (_prim.IsA<pxr::UsdLuxDiskLight>())
    {
      light->SetType(sdf::LightType::SPOT);
      light->SetSpotInnerAngle(0.1);
      light->SetSpotOuterAngle(0.5);
      light->SetSpotFalloff(0.8);
    }
    if (_prim.IsA<pxr::UsdLuxDistantLight>() ||
        _prim.IsA<pxr::UsdLuxDiskLight>())
    {
      float intensity;
      variantLight.GetIntensityAttr().Get(&intensity);

      ignition::math::Pose3d pose;
      ignition::math::Vector3d scale(1, 1, 1);
      GetTransform(_prim, _usdData, pose, scale, _linkName);

      if(_prim.IsA<pxr::UsdLuxDistantLight>() &&
        pose == ignition::math::Pose3d(0, 0, 0, IGN_PI_2, 0, 0))
      {
        pose = ignition::math::Pose3d(0, 0, 10, 0, 0, 0);
        light->SetDirection(ignition::math::Vector3d(-0.5, 0.1, -0.9));
      }

      light->SetName(_prim.GetPath().GetName());
      light->SetRawPose(pose);
      light->SetCastShadows(true);
      // TODO(anyone): review this
      light->SetIntensity(intensity/100000);
      light->SetDiffuse(ignition::math::Color(0.8, 0.8, 0.8, 1));
      light->SetSpecular(ignition::math::Color(0.2, 0.2, 0.2, 1));
      light->SetAttenuationRange(1000);
      light->SetLinearAttenuationFactor(0.01);
      light->SetConstantAttenuationFactor(0.9);
      light->SetQuadraticAttenuationFactor(0.001);
      // light->SetDirection(ignition::math::Vector3d(-0.5, 0.1, -0.9));
    }
    else
    {
      return nullptr;
    }
    return light;
  }
}
}
}
