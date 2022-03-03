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

#include "sdf/usd/sdf_parser/Material.hh"

#include <map>
#include <string>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/URI.hh>

#include <ignition/math/Color.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/tf/stringUtils.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdShade/connectableAPI.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Pbr.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  /// \brief Copy a file in a directory
  /// \param[in] _path path where the copy will be located
  /// \param[in] _fullPath name of the file to copy
  /// \return True if the file at _fullPath was copied to _path. False otherwise
  bool copyMaterial(const std::string &_path, const std::string &_fullPath)
  {
    if (!_path.empty() && !_fullPath.empty())
    {
      auto fileName = ignition::common::basename(_path);
      auto filePathIndex = _path.rfind(fileName);
      auto filePath = _path.substr(0, filePathIndex);
      if (!filePath.empty())
      {
        ignition::common::createDirectories(filePath);
      }
      return ignition::common::copyFile(_fullPath, _path);
    }
    return false;
  }

  /// \brief Get the path to copy the material to
  /// \param[in] _uri full path of the file to copy
  /// \return A relative path to save the material. The path looks like:
  /// materials/textures/<filename with extension>
  std::string getMaterialCopyPath(const std::string &_uri)
  {
    return ignition::common::joinPaths(
      "materials",
      "textures",
      ignition::common::basename(_uri));
  }

  /// \brief Fill Material shader attributes and properties
  /// \param[in] _prim USD primitive
  /// \param[in] _name Name of the field attribute or property
  /// \param[in] _vType Type of the field
  /// \param[in] _value Value of the field
  /// \param[in] _customData Custom data to set the field
  /// \param[in] _displayName Display name
  /// \param[in] _displayGroup Display group
  /// \param[in] _doc Documentation of the field
  /// \param[in] _colorSpace if the material is a texture, we can specify the
  /// color space of the image
  /// \return UsdErrors, which is a list of UsdError objects. This list is empty
  /// if no errors occurred when creating the material input.
  template<typename T>
  UsdErrors CreateMaterialInput(
    const pxr::UsdPrim &_prim,
    const std::string &_name,
    const pxr::SdfValueTypeName &_vType,
    const T &_value,
    const std::map<pxr::TfToken, pxr::VtValue> &_customData,
    const pxr::TfToken &_displayName,
    const pxr::TfToken &_displayGroup,
    const std::string &_doc,
    const pxr::TfToken &_colorSpace = pxr::TfToken(""))
  {
    UsdErrors errors;
    auto shader = pxr::UsdShadeShader(_prim);
    if (!shader)
    {
      errors.emplace_back(UsdError(
        sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
        "Unable to convert the prim to a UsdShadeShader"));
      return errors;
    }

    auto existingInput = shader.GetInput(pxr::TfToken(_name));
    pxr::SdfValueTypeName vTypeName;
    if (_vType.IsScalar())
    {
      vTypeName = _vType.GetScalarType();
    }
    else if (_vType.IsArray())
    {
      vTypeName = _vType.GetArrayType();
    }
    auto surfaceInput = shader.CreateInput(
      pxr::TfToken(_name), vTypeName);
    surfaceInput.Set(_value);
    auto attr = surfaceInput.GetAttr();

    for (const auto &[key, customValue] : _customData)
    {
      attr.SetCustomDataByKey(key, customValue);
    }
    if (!_displayName.GetString().empty())
    {
      attr.SetDisplayName(_displayName);
    }
    if (!_displayGroup.GetString().empty())
    {
      attr.SetDisplayGroup(_displayGroup);
    }
    if (!_doc.empty())
    {
      attr.SetDocumentation(_doc);
    }
    if (!_colorSpace.GetString().empty())
    {
      attr.SetColorSpace(_colorSpace);
    }
    return errors;
  }

  UsdErrors ParseSdfMaterial(const sdf::Material *_materialSdf,
      pxr::UsdStageRefPtr &_stage, pxr::SdfPath &_materialPath)
  {
    UsdErrors errors;

    const auto looksPath = pxr::SdfPath("/Looks");
    auto looksPrim = _stage->GetPrimAtPath(looksPath);
    if (!looksPrim)
    {
      looksPrim = _stage->DefinePrim(looksPath, pxr::TfToken("Scope"));
    }

    // This variable will increase with every new material to avoid collision
    // with the names of the materials
    static int i = 0;

    _materialPath = pxr::SdfPath("/Looks/Material_" + std::to_string(i));
    i++;

    pxr::UsdShadeMaterial materialUsd;
    auto usdMaterialPrim = _stage->GetPrimAtPath(_materialPath);
    if (!usdMaterialPrim)
    {
      materialUsd = pxr::UsdShadeMaterial::Define(_stage, _materialPath);
    }
    else
    {
      materialUsd = pxr::UsdShadeMaterial(usdMaterialPrim);
    }

    const auto shaderPath = pxr::SdfPath(_materialPath.GetString() + "/Shader");
    auto usdShader = pxr::UsdShadeShader::Define(_stage, shaderPath);
    auto shaderPrim = _stage->GetPrimAtPath(shaderPath);
    if (!shaderPrim)
    {
      errors.emplace_back(UsdError(
        sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
        "Not able to cast the UsdShadeShader at path [" + shaderPath.GetString()
        + "] to a Prim"));
    }

    auto shaderOut = pxr::UsdShadeConnectableAPI(shaderPrim).CreateOutput(
      pxr::TfToken("out"), pxr::SdfValueTypeNames->Token);
    const auto mdlToken = pxr::TfToken("mdl");
    materialUsd.CreateSurfaceOutput(mdlToken).ConnectToSource(shaderOut);
    materialUsd.CreateVolumeOutput(mdlToken).ConnectToSource(shaderOut);
    materialUsd.CreateDisplacementOutput(mdlToken).ConnectToSource(shaderOut);
    usdShader.GetImplementationSourceAttr().Set(
      pxr::UsdShadeTokens->sourceAsset);
    usdShader.SetSourceAsset(pxr::SdfAssetPath("OmniPBR.mdl"), mdlToken);
    usdShader.SetSourceAssetSubIdentifier(pxr::TfToken("OmniPBR"), mdlToken);

    const std::map<pxr::TfToken, pxr::VtValue> customDataDiffuse =
    {
      {pxr::TfToken("default"), pxr::VtValue(pxr::GfVec3f(0.2, 0.2, 0.2))},
      {pxr::TfToken("range:max"),
       pxr::VtValue(pxr::GfVec3f(100000, 100000, 100000))},
      {pxr::TfToken("range:min"), pxr::VtValue(pxr::GfVec3f(0, 0, 0))}
    };
    const ignition::math::Color diffuse = _materialSdf->Diffuse();
    auto errorsMaterialDiffuseColorConstant = CreateMaterialInput<pxr::GfVec3f>(
      shaderPrim,
      "diffuse_color_constant",
      pxr::SdfValueTypeNames->Color3f,
      pxr::GfVec3f(diffuse.R(), diffuse.G(), diffuse.B()),
      customDataDiffuse,
      pxr::TfToken("Base Color"),
      pxr::TfToken("Albedo"),
      "This is the base color");

    if (!errorsMaterialDiffuseColorConstant.empty())
    {
      errors.insert(
        errors.end(),
        errorsMaterialDiffuseColorConstant.begin(),
        errorsMaterialDiffuseColorConstant.end());
      errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
            "Unable to set the base color of the material at path ["
            + _materialPath.GetString() + "]"));
      return errors;
    }

    const std::map<pxr::TfToken, pxr::VtValue> customDataEmissive =
    {
      {pxr::TfToken("default"), pxr::VtValue(pxr::GfVec3f(1, 0.1, 0.1))},
      {pxr::TfToken("range:max"),
       pxr::VtValue(pxr::GfVec3f(100000, 100000, 100000))},
      {pxr::TfToken("range:min"), pxr::VtValue(pxr::GfVec3f(0, 0, 0))}
    };
    ignition::math::Color emissive = _materialSdf->Emissive();
    auto errorsMaterialEmissiveColor = CreateMaterialInput<pxr::GfVec3f>(
      shaderPrim,
      "emissive_color",
      pxr::SdfValueTypeNames->Color3f,
      pxr::GfVec3f(emissive.R(), emissive.G(), emissive.B()),
      customDataEmissive,
      pxr::TfToken("Emissive Color"),
      pxr::TfToken("Emissive"),
      "The emission color");

    if (!errorsMaterialEmissiveColor.empty())
    {
      errors.insert(
        errors.end(),
        errorsMaterialEmissiveColor.begin(),
        errorsMaterialEmissiveColor.end());
      errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
            "Unable to set the emission color of the material at path ["
            + _materialPath.GetString() + "]"));
      return errors;
    }

    const std::map<pxr::TfToken, pxr::VtValue> customDataEnableEmission =
    {
      {pxr::TfToken("default"), pxr::VtValue(0)}
    };

    auto errorsMaterialEnableEmission = CreateMaterialInput<bool>(
      shaderPrim,
      "enable_emission",
      pxr::SdfValueTypeNames->Bool,
      emissive.A() > 0,
      customDataEnableEmission,
      pxr::TfToken("Enable Emissive"),
      pxr::TfToken("Emissive"),
      "Enables the emission of light from the material");

    if (!errorsMaterialEnableEmission.empty())
    {
      errors.insert(
        errors.end(),
        errorsMaterialEnableEmission.begin(),
        errorsMaterialEnableEmission.end());
      errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
            "Unable to set the emissive enabled propery of the material at path"
            " [" + _materialPath.GetString() + "]"));
      return errors;
    }

    const std::map<pxr::TfToken, pxr::VtValue> customDataIntensity =
    {
      {pxr::TfToken("default"), pxr::VtValue(40)},
      {pxr::TfToken("range:max"), pxr::VtValue(100000)},
      {pxr::TfToken("range:min"), pxr::VtValue(0)}
    };
    auto errorsMaterialEmissiveIntensity = CreateMaterialInput<float>(
      shaderPrim,
      "emissive_intensity",
      pxr::SdfValueTypeNames->Float,
      emissive.A(),
      customDataIntensity,
      pxr::TfToken("Emissive Intensity"),
      pxr::TfToken("Emissive"),
      "Intensity of the emission");

    if (!errorsMaterialEmissiveIntensity.empty())
    {
      errors.insert(
        errors.end(),
        errorsMaterialEmissiveIntensity.begin(),
        errorsMaterialEmissiveIntensity.end());
      errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
            "Unable to set the emissive intensity of the material at path ["
            + _materialPath.GetString() + "]"));
      return errors;
    }

    const sdf::Pbr * pbr = _materialSdf->PbrMaterial();
    if (pbr)
    {
      const sdf::PbrWorkflow * pbrWorkflow =
        pbr->Workflow(sdf::PbrWorkflowType::METAL);
      if (!pbrWorkflow)
      {
        pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
      }

      if (pbrWorkflow)
      {
        const std::map<pxr::TfToken, pxr::VtValue> customDataMetallicConstant =
        {
          {pxr::TfToken("default"), pxr::VtValue(0.5)},
          {pxr::TfToken("range:max"), pxr::VtValue(1)},
          {pxr::TfToken("range:min"), pxr::VtValue(0)}
        };
        auto errorsMaterialMetallicConstant = CreateMaterialInput<float>(
          shaderPrim,
          "metallic_constant",
          pxr::SdfValueTypeNames->Float,
          pbrWorkflow->Metalness(),
          customDataMetallicConstant,
          pxr::TfToken("Metallic Amount"),
          pxr::TfToken("Reflectivity"),
          "Metallic Material");
        if (!errorsMaterialMetallicConstant.empty())
        {
          errors.insert(
            errors.end(),
            errorsMaterialMetallicConstant.begin(),
            errorsMaterialMetallicConstant.end());
          errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
            "Unable to set the metallic constant of the material at path ["
            + _materialPath.GetString() + "]"));
          return errors;
        }
        const std::map<pxr::TfToken, pxr::VtValue> customDataRoughnessConstant =
        {
          {pxr::TfToken("default"), pxr::VtValue(0.5)},
          {pxr::TfToken("range:max"), pxr::VtValue(1)},
          {pxr::TfToken("range:min"), pxr::VtValue(0)}
        };
        auto errorsMaterialReflectionRoughnessConstant =
          CreateMaterialInput<float>(
            shaderPrim,
            "reflection_roughness_constant",
            pxr::SdfValueTypeNames->Float,
            pbrWorkflow->Roughness(),
            customDataRoughnessConstant,
            pxr::TfToken("Roughness Amount"),
            pxr::TfToken("Reflectivity"),
            "Higher roughness values lead to more blurry reflections");
        if (!errorsMaterialReflectionRoughnessConstant.empty())
        {
          errors.insert(
            errors.end(),
            errorsMaterialReflectionRoughnessConstant.begin(),
            errorsMaterialReflectionRoughnessConstant.end());
          errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
            "Unable to set the roughness constant of the material at path ["
            + _materialPath.GetString() + "]"));
          return errors;
        }

        const std::map<pxr::TfToken, pxr::VtValue> customDefaultSdfAssetPath =
        {
          {pxr::TfToken("default"), pxr::VtValue(pxr::SdfAssetPath())},
        };

        if (!pbrWorkflow->AlbedoMap().empty())
        {
          std::string copyPath = getMaterialCopyPath(pbrWorkflow->AlbedoMap());

          std::string fullnameAlbedoMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->AlbedoMap()));

          if (fullnameAlbedoMap.empty())
          {
            fullnameAlbedoMap = pbrWorkflow->AlbedoMap();
          }

          copyMaterial(copyPath, fullnameAlbedoMap);

          auto errorsMaterialDiffuseTexture =
            CreateMaterialInput<pxr::SdfAssetPath>(
              shaderPrim,
              "diffuse_texture",
              pxr::SdfValueTypeNames->Asset,
              pxr::SdfAssetPath(copyPath),
              customDefaultSdfAssetPath,
              pxr::TfToken("Base Map"),
              pxr::TfToken("Albedo"),
              "",
              pxr::TfToken("auto"));
          if (!errorsMaterialDiffuseTexture.empty())
          {
            errors.insert(
              errors.end(),
              errorsMaterialDiffuseTexture.begin(),
              errorsMaterialDiffuseTexture.end());
            errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
              "Unable to set the albedo of the material at path ["
              + _materialPath.GetString() + "]"));
            return errors;
          }
        }
        if (!pbrWorkflow->MetalnessMap().empty())
        {
          std::string copyPath =
            getMaterialCopyPath(pbrWorkflow->MetalnessMap());

          std::string fullnameMetallnessMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->MetalnessMap()));

          if (fullnameMetallnessMap.empty())
          {
            fullnameMetallnessMap = pbrWorkflow->MetalnessMap();
          }

          copyMaterial(copyPath, fullnameMetallnessMap);

          auto errorsMaterialMetallicTexture =
            CreateMaterialInput<pxr::SdfAssetPath>(
              shaderPrim,
              "metallic_texture",
              pxr::SdfValueTypeNames->Asset,
              pxr::SdfAssetPath(copyPath),
              customDefaultSdfAssetPath,
              pxr::TfToken("Metallic Map"),
              pxr::TfToken("Reflectivity"),
              "",
              pxr::TfToken("raw"));
          if (!errorsMaterialMetallicTexture.empty())
          {
            errors.insert(
              errors.end(),
              errorsMaterialMetallicTexture.begin(),
              errorsMaterialMetallicTexture.end());
            errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
              "Unable to set the reflectivity of the material at path ["
              + _materialPath.GetString() + "]"));
            return errors;
          }
        }
        if (!pbrWorkflow->NormalMap().empty())
        {
          std::string copyPath = getMaterialCopyPath(pbrWorkflow->NormalMap());

          std::string fullnameNormalMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->NormalMap()));

          if (fullnameNormalMap.empty())
          {
            fullnameNormalMap = pbrWorkflow->NormalMap();
          }

          copyMaterial(copyPath, fullnameNormalMap);

          auto errorsMaterialNormalMapTexture =
            CreateMaterialInput<pxr::SdfAssetPath>(
              shaderPrim,
              "normalmap_texture",
              pxr::SdfValueTypeNames->Asset,
              pxr::SdfAssetPath(copyPath),
              customDefaultSdfAssetPath,
              pxr::TfToken("Normal Map"),
              pxr::TfToken("Normal"),
              "",
              pxr::TfToken("raw"));
          if (!errorsMaterialNormalMapTexture.empty())
          {
            errors.insert(
              errors.end(),
              errorsMaterialNormalMapTexture.begin(),
              errorsMaterialNormalMapTexture.end());
            errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
              "Unable to set the normal map of the material at path ["
              + _materialPath.GetString() + "]"));
            return errors;
          }
        }
        if (!pbrWorkflow->RoughnessMap().empty())
        {
          std::string copyPath =
            getMaterialCopyPath(pbrWorkflow->RoughnessMap());

          std::string fullnameRoughnessMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->RoughnessMap()));

          if (fullnameRoughnessMap.empty())
          {
            fullnameRoughnessMap = pbrWorkflow->RoughnessMap();
          }

          copyMaterial(copyPath, fullnameRoughnessMap);

          auto errorsMaterialReflectionRoughnessTexture =
            CreateMaterialInput<pxr::SdfAssetPath>(
              shaderPrim,
              "reflectionroughness_texture",
              pxr::SdfValueTypeNames->Asset,
              pxr::SdfAssetPath(copyPath),
              customDefaultSdfAssetPath,
              pxr::TfToken("RoughnessMap Map"),
              pxr::TfToken("RoughnessMap"),
              "",
              pxr::TfToken("raw"));
          if (!errorsMaterialReflectionRoughnessTexture.empty())
          {
            errors.insert(
              errors.end(),
              errorsMaterialReflectionRoughnessTexture.begin(),
              errorsMaterialReflectionRoughnessTexture.end());
            errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
              "Unable to set the roughness map of the material at path ["
              + _materialPath.GetString() + "]"));
            return errors;
          }

          const std::map<pxr::TfToken, pxr::VtValue>
            customDataRoughnessTextureInfluence =
          {
            {pxr::TfToken("default"), pxr::VtValue(0)},
            {pxr::TfToken("range:max"), pxr::VtValue(1)},
            {pxr::TfToken("range:min"), pxr::VtValue(0)}
          };

          auto errorsMaterialReflectionRoughnessTextureInfluence =
            CreateMaterialInput<bool>(
              shaderPrim,
              "reflection_roughness_texture_influence",
              pxr::SdfValueTypeNames->Bool,
              true,
              customDataRoughnessTextureInfluence,
              pxr::TfToken("Roughness Map Influence"),
              pxr::TfToken("Reflectivity"),
              "",
              pxr::TfToken("raw"));
          if (!errorsMaterialReflectionRoughnessTextureInfluence.empty())
          {
            errors.insert(
              errors.end(),
              errorsMaterialReflectionRoughnessTextureInfluence.begin(),
              errorsMaterialReflectionRoughnessTextureInfluence.end());
            errors.push_back(UsdError(UsdErrorCode::INVALID_MATERIAL,
              "Unable to set the reflectivity of the material at path ["
              + _materialPath.GetString() + "]"));
            return errors;
          }
        }
      }
    }

    return errors;
  }
}
}
}
