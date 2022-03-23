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

#include <string>
#include "USDMaterial.hh"

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <pxr/usd/usdShade/input.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/shader.h>

#include "sdf/Pbr.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
  /////////////////////////////////////////////////
  /// \brief Copy a file from one destination to another
  /// \param[in] _ori The original file to copy
  /// \param[in] _dest The destination for the copy of _ori
  /// \return A list of UsdErrors. An empty list means no errors occurred when
  /// copying _ori to _dest
  UsdErrors copyFile(const std::string &_ori, const std::string &_dest)
  {
    UsdErrors errors;
    if (ignition::common::exists(_ori))
    {
      // If the file exists then we should remove it and we will copy the
      // file below
      if (ignition::common::exists(_dest))
      {
        ignition::common::removeFile(_dest);
      }

      std::string baseName = ignition::common::basename(_dest);
      std::string pathDest = ignition::common::replaceAll(_dest, baseName, "");
      ignition::common::createDirectories(pathDest);
      if (!ignition::common::copyFile(_ori, _dest))
      {
        errors.emplace_back(
            Error(ErrorCode::FILE_READ, "Unable to copy the file [" + _ori +
                  "] to [" + _dest + "]"));
      }
    }
    else
    {
      errors.emplace_back(
          Error(ErrorCode::FILE_READ, "File [" + _ori + "] does not exist"));
    }
    return errors;
  }

  /// \brief Helper method for getting an asset path
  /// \param[in] _tokenName Name of the asset
  /// \param[in] _shader Shader that holds the desired asset
  /// \return The pxr::SdfAssetPath object that contains the asset identified by
  /// _tokenName in _shader
  pxr::SdfAssetPath assetPath(const pxr::TfToken &_tokenName,
      const pxr::UsdShadeShader &_shader)
  {
    pxr::SdfAssetPath assetPath;
    pxr::UsdShadeInput shadeInput = _shader.GetInput(_tokenName);
    shadeInput.Get(&assetPath);
    return assetPath;
  }

  /////////////////////////////////////////////////
  UsdErrors ParseMaterial(const pxr::UsdPrim &_prim, sdf::Material &_material)
  {
    UsdErrors errors;
    // if the prim is a Geom then get the color values
    if (_prim.IsA<pxr::UsdGeomGprim>())
    {
      auto variantGeom = pxr::UsdGeomGprim(_prim);

      pxr::VtArray<pxr::GfVec3f> color(0, 0, 0);
      variantGeom.GetDisplayColorAttr().Get(&color);

      pxr::VtFloatArray displayOpacity;
      const std::string displayOpacityToken = "primvars:displayOpacity";
      auto opacityAttr = _prim.GetAttribute(pxr::TfToken(displayOpacityToken));
      if (!opacityAttr)
      {
        errors.push_back(UsdError(UsdErrorCode::PRIM_MISSING_ATTRIBUTE,
              "Prim at path [" + _prim.GetPath().GetString()
              + "] does not have an attribute with a pxr::TfToken of ["
              + displayOpacityToken + "]"));
        return errors;
      }
      opacityAttr.Get(&displayOpacity);

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
      for (const auto &child : _prim.GetChildren())
      {
        if (child.IsA<pxr::UsdShadeShader>())
        {
          auto variantShader = pxr::UsdShadeShader(child);

          bool enableEmission = false;
          bool isPBR = false;
          sdf::PbrWorkflow pbrWorkflow;
          ignition::math::Color emissiveColorCommon;

          for (const auto &input : variantShader.GetInputs())
          {
            if (input.GetBaseName() == "diffuse_texture")
            {
              pxr::SdfAssetPath materialPath =
                assetPath(pxr::TfToken("diffuse_texture"), variantShader);
              pbrWorkflow.SetAlbedoMap(materialPath.GetAssetPath());
              std::string fullAlbedoName =
                ignition::common::findFile(ignition::common::basename(
                  materialPath.GetAssetPath()), false);
              UsdErrors errorCopy = copyFile(
                fullAlbedoName, materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }

              // We need to set diffuse and specular to (1, 1, 1) otherwise
              // the texture is completely black
              _material.SetDiffuse(ignition::math::Color(1, 1, 1));
              _material.SetSpecular(ignition::math::Color(1, 1, 1));

              isPBR = true;
            }
            else if (input.GetBaseName() == "normalmap_texture")
            {
              pxr::SdfAssetPath materialPath =
                assetPath(pxr::TfToken("normalmap_texture"), variantShader);
              pbrWorkflow.SetNormalMap(materialPath.GetAssetPath());
              std::string fullNormalName =
                ignition::common::findFile(ignition::common::basename(
                  materialPath.GetAssetPath()), false);
              auto errorCopy = copyFile(fullNormalName,
                materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "reflectionroughness_texture")
            {
              pxr::SdfAssetPath materialPath = assetPath(
                  pxr::TfToken("reflectionroughness_texture"), variantShader);
              pbrWorkflow.SetRoughnessMap(materialPath.GetAssetPath());
              std::string fullRoughnessName =
                ignition::common::findFile(ignition::common::basename(
                  materialPath.GetAssetPath()), false);
              auto errorCopy = copyFile(
                fullRoughnessName, materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "metallic_texture")
            {
              pxr::SdfAssetPath materialPath = assetPath(
                  pxr::TfToken("metallic_texture"), variantShader);
              pbrWorkflow.SetMetalnessMap(materialPath.GetAssetPath());
              std::string fullMetalnessName =
                ignition::common::findFile(ignition::common::basename(
                  materialPath.GetAssetPath()), false);
              auto errorCopy = copyFile(
                fullMetalnessName, materialPath.GetAssetPath());
              if (!errorCopy.empty())
              {
                errors.insert(errors.end(), errorCopy.begin(), errorCopy.end());
                return errors;
              }
              isPBR = true;
            }
            else if (input.GetBaseName() == "diffuse_color_constant")
            {
              pxr::GfVec3f diffuseColor(0, 0, 0);
              auto sourceInfoV = input.GetConnectedSources();
              if (sourceInfoV.size() > 0)
              {
                pxr::UsdShadeInput connectedInput =
                  sourceInfoV[0].source.GetInput(sourceInfoV[0].sourceName);

                const pxr::SdfPath& thisAttrPath =
                  connectedInput.GetAttr().GetPath();
                auto connectedPrim =
                  _prim.GetStage()->GetPrimAtPath(thisAttrPath.GetPrimPath());
                if (connectedPrim)
                {
                  connectedPrim.GetAttribute(
                    pxr::TfToken("inputs:diffuse_color_constant")).
                      Get(&diffuseColor);
                }
              }
              else
              {
                pxr::UsdShadeInput diffuseShaderInput = variantShader.GetInput(
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
                variantShader.GetInput(pxr::TfToken("metallic_constant"));
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
                  variantShader.GetInput(
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
                variantShader.GetInput(pxr::TfToken("enable_emission"));
              enableEmissiveShaderInput.Get(&enableEmission);
            }
            else if (input.GetBaseName() == "emissive_color")
            {
              pxr::GfVec3f emissiveColor(0, 0, 0);
              pxr::UsdShadeInput emissiveColorShaderInput =
                variantShader.GetInput(pxr::TfToken("emissive_color"));
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
            pbrWorkflow.SetType(sdf::PbrWorkflowType::METAL);
            pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, pbrWorkflow);
            _material.SetPbrMaterial(pbr);
          }
        }
      }
    }
    else
    {
      errors.push_back(UsdError(UsdErrorCode::PRIM_INCORRECT_SCHEMA_TYPE,
            "Prim at path [" + _prim.GetPath().GetString()
            + "is not a pxr::UsdGeomGprim or a pxr::UsdShadeMaterial."));
    }

    return errors;
  }
}
}
}
