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

#include "USDLights.hh"

#include <memory>

// TODO(ahcorde) This is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
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
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  std::optional<sdf::Light> ParseUSDLights(
    const pxr::UsdPrim &_prim,
    const USDData &_usdData,
    const std::string &_linkName)
  {
    sdf::Light lightSdf;
    std::optional<sdf::Light> light = lightSdf;

    auto variantLight = pxr::UsdLuxBoundableLightBase(_prim);

    gz::math::Pose3d pose;
    gz::math::Vector3d scale(1, 1, 1);
    GetTransform(_prim, _usdData, pose, scale, _linkName);

    light->SetName(_prim.GetPath().GetName());
    float intensity;
    variantLight.GetIntensityAttr().Get(&intensity);
    // This value was found trying to find a similar light intensity
    // between isaac sim and gazebo. Might be wrong
    // TODO(ahcorde): Convert the light intensity with an equation or unit
    // conversions
    light->SetIntensity(intensity / 10000);
    float diffuse;
    variantLight.GetDiffuseAttr().Get(&diffuse);
    light->SetDiffuse(gz::math::Color(diffuse, diffuse, diffuse, 1));
    float specular;
    variantLight.GetSpecularAttr().Get(&specular);
    light->SetSpecular(gz::math::Color(specular, specular, specular, 1));
    light->SetCastShadows(true);

    if (_prim.IsA<pxr::UsdLuxDistantLight>())
    {
      light->SetType(sdf::LightType::DIRECTIONAL);

      // DistantLight in USD does not define height. Added some height to the
      // light. The default sun light in gz-sim sdf world is 10.
      pose = gz::math::Pose3d(0, 0, 10, 0, 0, 0) * pose;
      // Light emitted from a distant source along the -Z axis
      // The pose should set the direction
      light->SetDirection(gz::math::Vector3d(0, 0, -1));
    }
    else if (_prim.IsA<pxr::UsdLuxDiskLight>())
    {
      light->SetType(sdf::LightType::SPOT);
      // These parameters are not defined in USD. Added some generic values.
      light->SetSpotInnerAngle(0.1);
      light->SetSpotOuterAngle(0.5);
      light->SetSpotFalloff(0.8);
    }
    else
    {
      return std::nullopt;
    }
    light->SetRawPose(pose);
    return light;
  }
}
}
}
