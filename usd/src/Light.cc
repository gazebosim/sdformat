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

#include "sdf/usd/Light.hh"

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
#include "sdf/usd/Utils.hh"

namespace usd
{
  bool ParseSdfLight(const sdf::Light &_light, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const pxr::SdfPath sdfLightPath(_path);
    bool typeParsed = true;
    switch (_light.Type())
    {
      case sdf::LightType::POINT:
        {
          auto pointLight =
            pxr::UsdLuxSphereLight::Define(_stage, sdfLightPath);
          pointLight.CreateTreatAsPointAttr().Set(true);
        }
        break;
      case sdf::LightType::SPOT:
        pxr::UsdLuxDiskLight::Define(_stage, sdfLightPath);
        break;
      case sdf::LightType::DIRECTIONAL:
        pxr::UsdLuxDistantLight::Define(_stage, sdfLightPath);
        break;
      case sdf::LightType::INVALID:
      default:
        std::cerr << "Light type is either invalid or not supported\n";
        typeParsed = false;
    }

    if (typeParsed)
    {
      // TODO(adlarkin) incorporate sdf::Light's <direction> somehow? According
      // to the USD API, things like UsdLuxDistantLight and UsdLuxDiskLight emit
      // light along the -Z axis, so I'm not sure if this can be changed.
      usd::SetPose(usd::PoseWrtParent(_light), _stage, sdfLightPath);

      // This is a workaround to set the light's intensity attribute. Using the
      // UsdLuxLightAPI sets the light's inputs:intensity attribute, but isaac
      // sim reads the light's intensity attribute
      auto lightPrim = _stage->GetPrimAtPath(sdfLightPath);
      lightPrim.CreateAttribute(pxr::TfToken("intensity"),
          pxr::SdfValueTypeNames->Float, false).Set(
            static_cast<float>(_light.Intensity()) * 100.0f);

      // TODO(adlarkin) Other things to look at (there may be more):
      // * exposure - I don't think SDF has this, but USD does. See the
      //    UsdLightAPI::GetExposureAttr method
      // * diffuse, specular - USD takes it as a scalar multiplier,
      //    SDF takes it as a RGB color vector. Perhaps this can be handled by
      //    applying a material to a light
    }

    return typeParsed;
  }
}
