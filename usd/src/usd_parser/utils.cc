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

#include "sdf/usd/usd_parser/utils.hh"

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <pxr/usd/usdShade/input.h>

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/shader.h>

#include "sdf/Pbr.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
  /////////////////////////////////////////////////
  std::string removeSubStr(const std::string &_str, const std::string &_substr)
  {
    std::string result = _str;
    size_t pos = std::string::npos;
    if ((pos = result.find(_substr) )!= std::string::npos)
    {
      result.erase(pos, _substr.length());
    }
    return result;
  }

  /////////////////////////////////////////////////
  sdf::Errors copyFile(const std::string &_ori, const std::string &_dest)
  {
    sdf::Errors errors;
    if (ignition::common::exists(_ori))
    {
      std::string baseName = ignition::common::basename(_dest);
      std::string pathDest = removeSubStr(_dest, baseName);
      ignition::common::createDirectories(pathDest);
      if (!ignition::common::copyFile(_ori, _dest))
      {
        errors.emplace_back(
            Error(ErrorCode::FILE_READ, "No able to copy the file " + _ori +
                  " in " + _dest));
      }
    }
    else
    {
      errors.emplace_back(
          Error(ErrorCode::FILE_READ, "File does not exists"));
    }
    return errors;
  }

  /////////////////////////////////////////////////
  sdf::Errors ParseMaterial(const pxr::UsdPrim &_prim, sdf::Material &_material)
  {
    sdf::Errors errors;
    // if the prim is a Geom then get the color values
    if(_prim.IsA<pxr::UsdGeomGprim>())
    {
      auto variant_geom = pxr::UsdGeomGprim(_prim);

      pxr::VtArray<pxr::GfVec3f> color {{0, 0, 0}};

      variant_geom.GetDisplayColorAttr().Get(&color);

      pxr::VtFloatArray displayOpacity;
      _prim.GetAttribute(
        pxr::TfToken("primvars:displayOpacity")).Get(&displayOpacity);

      variant_geom.GetDisplayColorAttr().Get(&color);
      double alpha = 1.0;
      if (displayOpacity.size() > 0)
      {
        alpha = 1 - displayOpacity[0];
      }
      // Refer to this comment in github to understand the ambient and diffuse
      // https://github.com/osrf/sdformat/pull/526#discussion_r623937715
      _material.SetAmbient(
        ignition::math::Color(
          ignition::math::clamp(color[0][2] / 0.4, 0.0, 1.0),
          ignition::math::clamp(color[0][1] / 0.4, 0.0, 1.0),
          ignition::math::clamp(color[0][0] / 0.4, 0.0, 1.0),
          alpha));
      _material.SetDiffuse(
        ignition::math::Color(
          ignition::math::clamp(color[0][2] / 0.8, 0.0, 1.0),
          ignition::math::clamp(color[0][1] / 0.8, 0.0, 1.0),
          ignition::math::clamp(color[0][0] / 0.8, 0.0, 1.0),
          alpha));
    }
    // if the prim is a shade Material then get the texture values
    else if (_prim.IsA<pxr::UsdShadeMaterial>())
    {
      auto variantMaterial = pxr::UsdShadeMaterial(_prim);
      for (const auto & child : _prim.GetChildren())
      {
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
            if (input.GetBaseName() == "diffuse_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput diffuseTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("diffuse_texture"));
              auto source = diffuseTextureShaderInput.GetConnectedSources();
              diffuseTextureShaderInput.Get(&materialPath);
              pbrWorkflow.SetAlbedoMap(materialPath.GetAssetPath());
              std::string fullAlbedoName =
                ignition::common::findFile(materialPath.GetAssetPath());
              sdf::Errors errorCopy = copyFile(
                fullAlbedoName, materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.emplace_back(
                  Error(ErrorCode::FILE_READ, "Failed to copy file"));
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "normalmap_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput normalTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("normalmap_texture"));
              auto source = normalTextureShaderInput.GetConnectedSources();
              normalTextureShaderInput.Get(&materialPath);
              pbrWorkflow.SetNormalMap(materialPath.GetAssetPath());
              std::string fullNormalName =
                ignition::common::findFile(materialPath.GetAssetPath());
              auto errorCopy = copyFile(fullNormalName,
                materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.emplace_back(
                  Error(ErrorCode::FILE_READ, "Failed to copy file"));
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "reflectionroughness_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput roughnessTextureShaderInput =
                variantshader.GetInput(
                  pxr::TfToken("reflectionroughness_texture"));
              auto source = roughnessTextureShaderInput.GetConnectedSources();
              roughnessTextureShaderInput.Get(&materialPath);
              pbrWorkflow.SetRoughnessMap(materialPath.GetAssetPath());
              std::string fullRoughnessName =
                ignition::common::findFile(materialPath.GetAssetPath());
              auto errorCopy = copyFile(
                fullRoughnessName, materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.emplace_back(
                  Error(ErrorCode::FILE_READ, "Failed to copy file"));
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "metallic_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput metallicTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("metallic_texture"));
              auto source = metallicTextureShaderInput.GetConnectedSources();
              metallicTextureShaderInput.Get(&materialPath);
              pbrWorkflow.SetMetalnessMap(materialPath.GetAssetPath());
              std::string fullMetalnessName =
                ignition::common::findFile(materialPath.GetAssetPath());
              auto errorCopy = copyFile(
                fullMetalnessName, materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.emplace_back(
                  Error(ErrorCode::FILE_READ, "Failed to copy file"));
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "diffuse_color_constant")
            {
              auto sourceInfoV = input.GetConnectedSources();
              if (sourceInfoV.size() > 0)
              {
                pxr::UsdShadeInput connectedInput =
                  sourceInfoV[0].source.GetInput(sourceInfoV[0].sourceName);

                const pxr::SdfPath& thisAttrPath =
                  connectedInput.GetAttr().GetPath();
                auto connectedPrim =
                  _prim.GetStage()->GetPrimAtPath(thisAttrPath.GetPrimPath());
                if(connectedPrim)
                  connectedPrim.GetAttribute(
                    pxr::TfToken("inputs:diffuse_color_constant")).
                      Get(&diffuseColor);
              }
              else
              {
                pxr::UsdShadeInput diffuseShaderInput =
                  variantshader.GetInput(
                    pxr::TfToken("diffuse_color_constant"));
                diffuseShaderInput.Get(&diffuseColor);
              }
              _material.SetDiffuse(
                ignition::math::Color(
                  diffuseColor[0],
                  diffuseColor[1],
                  diffuseColor[2]));
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
              auto sourceInfoV = input.GetConnectedSources();
              if (sourceInfoV.size() > 0)
              {
                pxr::UsdShadeInput connectedInput =
                  sourceInfoV[0].source.GetInput(sourceInfoV[0].sourceName);

                const pxr::SdfPath& thisAttrPath =
                  connectedInput.GetAttr().GetPath();
                auto connectedPrim =
                  _prim.GetStage()->GetPrimAtPath(thisAttrPath.GetPrimPath());
                if(connectedPrim)
                {
                  float reflectionRoughnessConstant;
                  connectedPrim.GetAttribute(
                    pxr::TfToken("inputs:reflection_roughness_constant")).
                      Get(&reflectionRoughnessConstant);
                  pbrWorkflow.SetRoughness(reflectionRoughnessConstant);
                  isPBR = true;
                }
              }
              else
              {
                pxr::UsdShadeInput reflectionRoughnessConstantShaderInput =
                  variantshader.GetInput(
                    pxr::TfToken("reflection_roughness_constant"));
                float reflectionRoughnessConstant;
                reflectionRoughnessConstantShaderInput.
                  Get(&reflectionRoughnessConstant);
                pbrWorkflow.SetRoughness(reflectionRoughnessConstant);
                isPBR = true;
              }
            }
            else if (input.GetBaseName() == "enable_emission")
            {
              pxr::UsdShadeInput enableEmissiveShaderInput =
                variantshader.GetInput(pxr::TfToken("enable_emission"));
              enableEmissiveShaderInput.Get(&enableEmission);
            }
            else if (input.GetBaseName() == "emissive_color")
            {
                pxr::UsdShadeInput emissiveColorShaderInput =
                  variantshader.GetInput(pxr::TfToken("emissive_color"));
                if (emissiveColorShaderInput.Get(&emissiveColor))
                {
                  emissiveColorCommon = ignition::math::Color(
                    emissiveColor[0],
                    emissiveColor[1],
                    emissiveColor[2]);
                }
            }
          }

          if (enableEmission)
          {
            _material.SetEmissive(emissiveColorCommon);
          }

          if (isPBR)
          {
            sdf::Pbr pbr;
            pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, pbrWorkflow);
            _material.SetPbrMaterial(pbr);
          }
        }
      }
    }
    return errors;
  }
}
}
}
