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

#include "sdf_usd_parser/material.hh"

#include <map>

#include <ignition/common/URI.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#include <ignition/math/Color.hh>

#include <pxr/base/tf/stringUtils.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdShade/connectableAPI.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "sdf/Pbr.hh"

namespace usd
{
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

  std::string getCopyPath(const std::string &_uri)
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
  /// colorSpace of the image
  template<typename T>
  void CreateMaterialInput(
    pxr::UsdPrim &_prim,
    std::string _name,
    pxr::SdfValueTypeName _vType,
    T _value,
    std::map<pxr::TfToken, pxr::VtValue> &_customData,
    pxr::TfToken _displayName = pxr::TfToken(""),
    pxr::TfToken _displayGroup = pxr::TfToken(""),
    std::string _doc = "",
    pxr::TfToken _colorSpace = pxr::TfToken(""))
  {
    auto shader = pxr::UsdShadeShader(_prim);
    if (shader)
    {
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
    }
    else
    {
      std::cerr << "Not able to convert the prim to a UsdShadeShader\n";
    }
  }

  pxr::UsdShadeMaterial ParseSdfMaterial(
    const sdf::Material *_material, pxr::UsdStageRefPtr &_stage)
  {
    auto looksPrim = _stage->GetPrimAtPath(pxr::SdfPath("/Looks"));
    if (!looksPrim)
    {
      looksPrim = _stage->DefinePrim(
        pxr::SdfPath("/Looks"), pxr::TfToken("Scope"));
    }

    // This variable will increase with every new material to avoid collision
    // with the names of the materials
    static int i = 0;

    const std::string mtl_path = "/Looks/Material_" + std::to_string(i);

    auto usdMaterialPrim = _stage->GetPrimAtPath(pxr::SdfPath(mtl_path));
    pxr::UsdShadeMaterial material;
    if (!usdMaterialPrim)
    {
      material = pxr::UsdShadeMaterial::Define(_stage, pxr::SdfPath(mtl_path));
    }
    else
    {
      material = pxr::UsdShadeMaterial(usdMaterialPrim);
    }

    auto usdShader = pxr::UsdShadeShader::Define(
      _stage,
      pxr::SdfPath(mtl_path + "/Shader"));
    auto shaderPrim = _stage->GetPrimAtPath(pxr::SdfPath(mtl_path + "/Shader"));
    if (!shaderPrim)
    {
      std::cerr << "Not able to cast the UsdShadeShader to a Prim" << '\n';
      return material;
    }

    auto shader_out = pxr::UsdShadeConnectableAPI(shaderPrim).CreateOutput(
      pxr::TfToken("out"), pxr::SdfValueTypeNames->Token);
    material.CreateSurfaceOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    material.CreateVolumeOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    material.CreateDisplacementOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    usdShader.GetImplementationSourceAttr().Set(
      pxr::UsdShadeTokens->sourceAsset);
    usdShader.SetSourceAsset(
      pxr::SdfAssetPath("OmniPBR.mdl"), pxr::TfToken("mdl"));
    usdShader.SetSourceAssetSubIdentifier(
      pxr::TfToken("OmniPBR"), pxr::TfToken("mdl"));

    std::map<pxr::TfToken, pxr::VtValue> customDataDiffuse =
    {
      {pxr::TfToken("default"), pxr::VtValue(pxr::GfVec3f(0.2, 0.2, 0.2))},
      {pxr::TfToken("range:max"),
       pxr::VtValue(pxr::GfVec3f(100000, 100000, 100000))},
      {pxr::TfToken("range:min"), pxr::VtValue(pxr::GfVec3f(0, 0, 0))}
    };
    ignition::math::Color diffuse = _material->Diffuse();
    CreateMaterialInput<pxr::GfVec3f>(
      shaderPrim,
      "diffuse_color_constant",
      pxr::SdfValueTypeNames->Color3f,
      pxr::GfVec3f(diffuse.R(), diffuse.G(), diffuse.B()),
      customDataDiffuse,
      pxr::TfToken("Base Color"),
      pxr::TfToken("Albedo"),
      "This is the base color");

    std::map<pxr::TfToken, pxr::VtValue> customDataEmissive =
    {
      {pxr::TfToken("default"), pxr::VtValue(pxr::GfVec3f(1, 0.1, 0.1))},
      {pxr::TfToken("range:max"),
       pxr::VtValue(pxr::GfVec3f(100000, 100000, 100000))},
      {pxr::TfToken("range:min"), pxr::VtValue(pxr::GfVec3f(0, 0, 0))}
    };
    ignition::math::Color emissive = _material->Emissive();
    CreateMaterialInput<pxr::GfVec3f>(
      shaderPrim,
      "emissive_color",
      pxr::SdfValueTypeNames->Color3f,
      pxr::GfVec3f(emissive.R(), emissive.G(), emissive.B()),
      customDataEmissive,
      pxr::TfToken("Emissive Color"),
      pxr::TfToken("Emissive"),
      "The emission color");

    std::map<pxr::TfToken, pxr::VtValue> customDataEnableEmission =
    {
      {pxr::TfToken("default"), pxr::VtValue(0)}
    };

    CreateMaterialInput<bool>(
      shaderPrim,
      "enable_emission",
      pxr::SdfValueTypeNames->Bool,
      emissive.A() > 0,
      customDataEnableEmission,
      pxr::TfToken("Enable Emissive"),
      pxr::TfToken("Emissive"),
      "Enables the emission of light from the material");

    std::map<pxr::TfToken, pxr::VtValue> customDataIntensity =
    {
      {pxr::TfToken("default"), pxr::VtValue(40)},
      {pxr::TfToken("range:max"), pxr::VtValue(100000)},
      {pxr::TfToken("range:min"), pxr::VtValue(0)}
    };
    CreateMaterialInput<float>(
      shaderPrim,
      "emissive_intensity",
      pxr::SdfValueTypeNames->Float,
      emissive.A(),
      customDataIntensity,
      pxr::TfToken("Emissive Intensity"),
      pxr::TfToken("Emissive"),
      "Intensity of the emission");

    const sdf::Pbr * pbr = _material->PbrMaterial();
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
        std::map<pxr::TfToken, pxr::VtValue> customDataMetallicConstant =
        {
          {pxr::TfToken("default"), pxr::VtValue(0.5)},
          {pxr::TfToken("range:max"), pxr::VtValue(1)},
          {pxr::TfToken("range:min"), pxr::VtValue(0)}
        };
        CreateMaterialInput<float>(
          shaderPrim,
          "metallic_constant",
          pxr::SdfValueTypeNames->Float,
          pbrWorkflow->Metalness(),
          customDataMetallicConstant,
          pxr::TfToken("Metallic Amount"),
          pxr::TfToken("Reflectivity"),
          "Metallic Material");
        std::map<pxr::TfToken, pxr::VtValue> customDataRoughnessConstant =
        {
          {pxr::TfToken("default"), pxr::VtValue(0.5)},
          {pxr::TfToken("range:max"), pxr::VtValue(1)},
          {pxr::TfToken("range:min"), pxr::VtValue(0)}
        };
        CreateMaterialInput<float>(
          shaderPrim,
          "reflection_roughness_constant",
          pxr::SdfValueTypeNames->Float,
          pbrWorkflow->Roughness(),
          customDataRoughnessConstant,
          pxr::TfToken("Roughness Amount"),
          pxr::TfToken("Reflectivity"),
          "Higher roughness values lead to more blurry reflections");
        if (!pbrWorkflow->AlbedoMap().empty())
        {
          std::map<pxr::TfToken, pxr::VtValue> customDataDiffuseTexture =
          {
            {pxr::TfToken("default"), pxr::VtValue(pxr::SdfAssetPath())},
          };

          std::string copyPath = getCopyPath(pbrWorkflow->AlbedoMap());

          std::string fullnameAlbedoMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->AlbedoMap()));

          if (fullnameAlbedoMap.empty())
          {
            fullnameAlbedoMap = pbrWorkflow->AlbedoMap();
          }

          copyMaterial(copyPath, fullnameAlbedoMap);

          CreateMaterialInput<pxr::SdfAssetPath>(
            shaderPrim,
            "diffuse_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(copyPath),
            customDataDiffuseTexture,
            pxr::TfToken("Base Map"),
            pxr::TfToken("Albedo"),
            "",
            pxr::TfToken("auto"));
        }
        if (!pbrWorkflow->MetalnessMap().empty())
        {
          std::map<pxr::TfToken, pxr::VtValue> customDataMetallnessTexture =
          {
            {pxr::TfToken("default"), pxr::VtValue(pxr::SdfAssetPath())},
          };

          std::string copyPath = getCopyPath(pbrWorkflow->MetalnessMap());

          std::string fullnameMetallnessMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->MetalnessMap()));

          if (fullnameMetallnessMap.empty())
          {
            fullnameMetallnessMap = pbrWorkflow->MetalnessMap();
          }

          copyMaterial(copyPath, fullnameMetallnessMap);

          CreateMaterialInput<pxr::SdfAssetPath>(
            shaderPrim,
            "metallic_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(copyPath),
            customDataMetallnessTexture,
            pxr::TfToken("Metallic Map"),
            pxr::TfToken("Reflectivity"),
            "",
            pxr::TfToken("raw"));
        }
        if (!pbrWorkflow->NormalMap().empty())
        {
          std::map<pxr::TfToken, pxr::VtValue> customDataNormalTexture =
          {
            {pxr::TfToken("default"), pxr::VtValue(pxr::SdfAssetPath())},
          };

          std::string copyPath = getCopyPath(pbrWorkflow->NormalMap());

          std::string fullnameNormalMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->NormalMap()));

          if (fullnameNormalMap.empty())
          {
            fullnameNormalMap = pbrWorkflow->NormalMap();
          }

          copyMaterial(copyPath, fullnameNormalMap);

          CreateMaterialInput<pxr::SdfAssetPath>(
            shaderPrim,
            "normalmap_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(copyPath),
            customDataNormalTexture,
            pxr::TfToken("Normal Map"),
            pxr::TfToken("Normal"),
            "",
            pxr::TfToken("raw"));
        }
        if (!pbrWorkflow->RoughnessMap().empty())
        {
          std::map<pxr::TfToken, pxr::VtValue> customDataRoughnessTexture =
          {
            {pxr::TfToken("default"), pxr::VtValue(pxr::SdfAssetPath())},
          };

          std::string copyPath = getCopyPath(pbrWorkflow->RoughnessMap());

          std::string fullnameRoughnessMap =
            ignition::common::findFile(
              ignition::common::basename(pbrWorkflow->RoughnessMap()));

          if (fullnameRoughnessMap.empty())
          {
            fullnameRoughnessMap = pbrWorkflow->RoughnessMap();
          }

          copyMaterial(copyPath, fullnameRoughnessMap);

          CreateMaterialInput<pxr::SdfAssetPath>(
            shaderPrim,
            "reflectionroughness_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(copyPath),
            customDataRoughnessTexture,
            pxr::TfToken("RoughnessMap Map"),
            pxr::TfToken("RoughnessMap"),
            "",
            pxr::TfToken("raw"));

          std::map<pxr::TfToken, pxr::VtValue>
            customDataRoughnessTextureInfluence =
          {
            {pxr::TfToken("default"), pxr::VtValue(0)},
            {pxr::TfToken("range:max"), pxr::VtValue(1)},
            {pxr::TfToken("range:min"), pxr::VtValue(0)}
          };

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
        }
      }
    }

    i++;

    return material;
  }
}
