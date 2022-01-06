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

#include "sdf_usd_parser/light.hh"

#include <string>

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usdLux/diskLight.h>
#include <pxr/usd/usdLux/distantLight.h>
#include <pxr/usd/usdLux/sphereLight.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>

#include "sdf/Light.hh"
#include "sdf_usd_parser/utils.hh"

namespace usd
{
  bool ParseSdfPointLight(const sdf::Light &_light, pxr::UsdStageRefPtr &_stage,
      const pxr::SdfPath &_path)
  {
    auto pointLight = pxr::UsdLuxSphereLight::Define(_stage, _path);
    pointLight.CreateTreatAsPointAttr().Set(true);

    return true;
  }

  bool ParseSdfSpotLight(const sdf::Light &_light, pxr::UsdStageRefPtr &_stage,
      const pxr::SdfPath &_path)
  {
    // It may be more realistic to create a cone geometry with its opening pointing
    // downard, and apply a UsdLuxLightAPI to this geometry. This may gave the
    // "light emitting from a cone" effect (UsdLuxDiskLight is like light being
    // emitted from a circular end of a cylinder)
    pxr::UsdLuxDiskLight::Define(_stage, _path);

    return true;
  }

  bool ParseSdfDirectionalLight(const sdf::Light &_light,
      pxr::UsdStageRefPtr &_stage, const pxr::SdfPath &_path)
  {
    auto directionalLight = pxr::UsdLuxDistantLight::Define(_stage, _path);
    directionalLight.CreateIntensityAttr().Set(1000.0f);

    return true;
  }

  bool ParseSdfLight(const sdf::Light &_light, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const pxr::SdfPath sdfLightPath(_path);
    bool typeParsed = false;
    switch (_light.Type())
    {
      case sdf::LightType::POINT:
        typeParsed = ParseSdfPointLight(_light, _stage, sdfLightPath);
        break;
      case sdf::LightType::SPOT:
        typeParsed = ParseSdfSpotLight(_light, _stage, sdfLightPath);
        break;
      case sdf::LightType::DIRECTIONAL:
        // Directional lights in USD have their own intensity attribute
        // (along with the intensity from the base light class), which is high
        // to approximate the sun. So, intensity for directional lights
        // don't need to be increased any further
        typeParsed = ParseSdfDirectionalLight(_light, _stage, sdfLightPath);
        break;
      case sdf::LightType::INVALID:
      default:
        std::cerr << "Light type is either invalid or not supported\n";
    }

    if (typeParsed)
    {
      // TODO(adlarkin) figure out how an SDF light's <direction> effects the USD
      // light's pose. For example, a USD directional light with no rotation shines
      // a light in the -Z direction. So, does an SDF light's <direction> need to
      // be applied to a USD light somehow to ensure the light shines in the
      // appropriate direction? It seems like orientation of the pose of the SDF
      // light itself should be ignored, and direction should be used instead to
      // properly orient the USD light ... maybe I can take the <direction> vector,
      // turn it into a unit vector, extract RPY angle from that somehow, and use
      // that as the orientation of the light's pose (I'd probably need to flip the
      // z axis of the direction vector before applying computations on it though,
      // since USD has lights pointing in -Z by default)
      usd::SetPose(usd::PoseWrtParent(_light), _stage, sdfLightPath);


      // This is a workaround to set the light's intensity attribute. Using the
      // UsdLuxLightAPI sets the light's inputs:intensity attribute, but isaac
      // sim reads the light's intensity attribute
      auto lightPrim = _stage->GetPrimAtPath(sdfLightPath);
      lightPrim.CreateAttribute(pxr::TfToken("intensity"),
          pxr::SdfValueTypeNames->Float, false).Set(
            static_cast<float>(_light.Intensity()) * 100.0f);

      // TODO(adlarkin) Other things to look at (there may be more):
      // * exposure - I don't think SDF has this, but USD does
      //    (might be worth setting to 1 beause I think intensity is multiplied by
      //    this. See GetExposureAttr() docs for UsdLightAPI class)
      // * diffuse, specular - USD takes it as a scalar multiplier,
      //    SDF takes it as a RGB color vector
      // I think the things listed above can be handled by applying a material to the light
    }

    return typeParsed;
  }
}
