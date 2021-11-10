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

#include <iostream>

#include <pxr/usd/usdShade/material.h>

#include "utils.hh"
#include <pxr/usd/usdShade/shader.h>
#include <pxr/usd/usdShade/input.h>

#include "sdf/Pbr.hh"

namespace usd
{
  sdf::Material ParseMaterial(const pxr::UsdPrim &_prim, int &_skip)
  {
    sdf::Material material;
    if(_prim.IsA<pxr::UsdGeomGprim>())
    {
      auto variant_geom = pxr::UsdGeomGprim(_prim);

      pxr::VtArray<pxr::GfVec3f> color {{0, 0, 0}};

      variant_geom.GetDisplayColorAttr().Get(&color);

      pxr::VtFloatArray displayOpacity;
      _prim.GetAttribute(pxr::TfToken("primvars:displayOpacity")).Get(&displayOpacity);

      variant_geom.GetDisplayColorAttr().Get(&color);
      double alpha = 1.0;
      if (displayOpacity.size() > 0)
      {
        alpha = 1 - displayOpacity[0];
      }
      material.SetAmbient(
        ignition::math::Color(
          ignition::math::clamp(color[0][2] / 0.4, 0.0, 1.0),
          ignition::math::clamp(color[0][1] / 0.4, 0.0, 1.0),
          ignition::math::clamp(color[0][0] / 0.4, 0.0, 1.0),
          alpha));
      material.SetDiffuse(
        ignition::math::Color(
          ignition::math::clamp(color[0][2] / 0.8, 0.0, 1.0),
          ignition::math::clamp(color[0][1] / 0.8, 0.0, 1.0),
          ignition::math::clamp(color[0][0] / 0.8, 0.0, 1.0),
          alpha));
    }
    else if (_prim.IsA<pxr::UsdShadeMaterial>())
    {
      auto variantMaterial = pxr::UsdShadeMaterial(_prim);
      std::cerr << "UsdShadeMaterial" << '\n';
      for (const auto & child : _prim.GetChildren())
      {
        std::cerr << "child " << pxr::TfStringify(child.GetPath()) << '\n';

        if (child.IsA<pxr::UsdShadeShader>())
        {
          auto variantshader = pxr::UsdShadeShader(child);

          pxr::GfVec3f diffuseColor {0, 0, 0};
          pxr::GfVec3f emissiveColor {0, 0, 0};
          bool enableEmission = false;

          bool isPBR = false;
          sdf::PbrWorkflow pbrWorkflow;
          ignition::math::Color emissiveColorCommon;

          std::vector<pxr::UsdShadeInput> inputs = variantshader.GetInputs();
          for (auto &input : inputs)
          {
            std::cerr << "GetFullName " << input.GetFullName() << " " << input.GetBaseName() << '\n';
            if (input.GetBaseName() == "diffuse_color_constant")
            {
              pxr::UsdShadeInput diffuseShaderInput =
                variantshader.GetInput(pxr::TfToken("diffuse_color_constant"));
              diffuseShaderInput.Get(&diffuseColor);

              material.SetDiffuse(
                ignition::math::Color(
                  diffuseColor[0],
                  diffuseColor[1],
                  diffuseColor[2]));
              std::cerr << "diffuse " << ignition::math::Color(
                diffuseColor[0],
                diffuseColor[1],
                diffuseColor[2]) << '\n';
            }
            else if (input.GetBaseName() == "metallic_constant")
            {
              pxr::UsdShadeInput metallicConstantShaderInput =
                variantshader.GetInput(pxr::TfToken("metallic_constant"));
              float metallicConstant;
              metallicConstantShaderInput.Get(&metallicConstant);
              pbrWorkflow.SetMetalness(metallicConstant);
              isPBR = true;
            }
            else if (input.GetBaseName() == "reflection_roughness_constant")
            {
              pxr::UsdShadeInput reflectionRoughnessConstantShaderInput =
                variantshader.GetInput(pxr::TfToken("reflection_roughness_constant"));
              float reflectionRoughnessConstant;
              reflectionRoughnessConstantShaderInput.Get(&reflectionRoughnessConstant);
              pbrWorkflow.SetRoughness(reflectionRoughnessConstant);
              isPBR = true;
            }
            else if (input.GetBaseName() == "enable_emission")
            {
              pxr::UsdShadeInput enableEmissiveShaderInput =
                variantshader.GetInput(pxr::TfToken("enable_emission"));
              enableEmissiveShaderInput.Get(&enableEmission);
              std::cerr << "enableEmission " << enableEmission << '\n';
            }
            else if (input.GetBaseName() == "emissive_color")
            {
              // if (enableEmission)
              // {
                pxr::UsdShadeInput emissiveColorShaderInput =
                  variantshader.GetInput(pxr::TfToken("emissive_color"));
                if (emissiveColorShaderInput.Get(&emissiveColor))
                {
                  std::cerr << "emissiveColor " << emissiveColor << '\n';

                  emissiveColorCommon = ignition::math::Color(
                    emissiveColor[0],
                    emissiveColor[1],
                    emissiveColor[2]);
                }
              // }
            }
          }

          if (enableEmission)
          {
            material.SetEmissive(emissiveColorCommon);
          }

          if (isPBR)
          {
            sdf::Pbr pbr;
            pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, pbrWorkflow);
            material.SetPbrMaterial(pbr);
          }

          // std::cerr << "diffuseColor " << diffuseColor << '\n';
          // std::cerr << "emissive_color " << emissiveColor << '\n';
          // std::cerr << "enableEmission " << enableEmission << '\n';

          ++_skip;
        }
        // exit(-1);
      }
    }
    return material;
  }

}
