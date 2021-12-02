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

#include <map>

#include "sdf_usd_parser/material.hh"

#include "pxr/usd/usdShade/connectableAPI.h"
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/sdf/types.h>

#include <pxr/usd/usd/prim.h>
#include <pxr/base/tf/stringUtils.h>

#include <ignition/math/Color.hh>

#include "sdf/Pbr.hh"

namespace usd
{

  void remove_property(
    pxr::UsdStageRefPtr &_stage,
    pxr::SdfPath prim_path,
    pxr::TfToken property_name)
  {
    for (auto &layer : _stage->GetLayerStack())
    {
      auto primSpec = layer->GetPrimAtPath(prim_path);
      if (primSpec)
      {
        auto propertySpec =
          layer->GetPropertyAtPath(
            prim_path.AppendProperty(property_name));
        if (propertySpec)
        {
          // primSpec.RemoveProperty(propertySpec);
        }
      }
    }
  }

  template<typename T>
  void create_material_input(
    pxr::UsdPrim &_prim, std::string name, pxr::SdfValueTypeName vType, T value,
    std::map<pxr::TfToken, pxr::VtValue> &_customData, pxr::TfToken displayName = pxr::TfToken(""),
    pxr::TfToken displayGroup = pxr::TfToken(""), std::string doc = "",
    pxr::TfToken colorSpace = pxr::TfToken(""))
  {
    auto shader = pxr::UsdShadeShader(_prim);
    if (shader)
    {
      auto existingInput = shader.GetInput(pxr::TfToken(name));
      pxr::SdfValueTypeName vTypeName;
      if (vType.IsScalar())
      {
        vTypeName = vType.GetScalarType();
      } else if (vType.IsArray())
      {
        vTypeName = vType.GetArrayType();
      }
      if (existingInput &&
          existingInput.GetTypeName() != vTypeName)
      {
        // remove_property(shader.GetPrim().GetPath(), existingInput.GetFullName());
      }
      auto surfaceInput = shader.CreateInput(
        pxr::TfToken(name), vTypeName);
      surfaceInput.Set(value);
      auto attr = surfaceInput.GetAttr();

      for (auto& [key, customValue]: _customData)
      {
        attr.SetCustomDataByKey(key, customValue);//pxr::TfToken("default"), pxr::VtValue(pxr::GfVec3f(0.2, 0.2, 0.2)));
      }
      if (!displayName.GetString().empty())
      {
        attr.SetDisplayName(displayName);
      }
      if (!displayGroup.GetString().empty())
      {
        attr.SetDisplayGroup(displayGroup);
      }
      if (!doc.empty())
      {
        attr.SetDocumentation(doc);
      }
      if (!colorSpace.GetString().empty())
      {
        attr.SetColorSpace(colorSpace);
      }
    }
  }

  pxr::UsdShadeMaterial ParseSdfMaterial(const sdf::Material *_material, pxr::UsdStageRefPtr &_stage)
  {
    auto looksPrim = _stage->GetPrimAtPath(pxr::SdfPath("/Looks"));
    if (!looksPrim)
    {
      looksPrim = _stage->DefinePrim(pxr::SdfPath("/Looks"), pxr::TfToken("Scope"));
    }

    static int i = 0;

    auto usdMaterialPrim = _stage->GetPrimAtPath(
      pxr::SdfPath(std::string("/Looks/Material_") + std::to_string(i)));
    pxr::UsdShadeMaterial material;
    if (!usdMaterialPrim)
    {
      std::cerr << "create Material xform" << '\n';
      material = pxr::UsdShadeMaterial::Define(
        _stage,
        pxr::SdfPath(std::string("/Looks/Material_") + std::to_string(i)));
    }
    else
    {
      material = pxr::UsdShadeMaterial(usdMaterialPrim);
    }

    std::string mtl_path =
      "/Looks/" + pxr::TfMakeValidIdentifier("Material_") + std::to_string(i);

    auto shaderPrim = _stage->DefinePrim(
      pxr::SdfPath(mtl_path + "/Shader"), pxr::TfToken("Shader"));

    auto shader_out = pxr::UsdShadeConnectableAPI(shaderPrim).CreateOutput(
      pxr::TfToken("out"), pxr::SdfValueTypeNames->Token);
    material.CreateSurfaceOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    material.CreateSurfaceOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    material.CreateVolumeOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    material.CreateDisplacementOutput(
      pxr::TfToken("mdl")).ConnectToSource(shader_out);
    pxr::UsdShadeShader(shaderPrim).GetImplementationSourceAttr().Set(
      pxr::UsdShadeTokens->sourceAsset);
    pxr::UsdShadeShader(shaderPrim).SetSourceAsset(
      pxr::SdfAssetPath("OmniPBR.mdl"), pxr::TfToken("mdl"));
    pxr::UsdShadeShader(shaderPrim).SetSourceAssetSubIdentifier(
      pxr::TfToken("OmniPBR"), pxr::TfToken("mdl"));

    std::map<pxr::TfToken, pxr::VtValue> customDataDiffuse =
    {
      {pxr::TfToken("default"), pxr::VtValue(pxr::GfVec3f(0.2, 0.2, 0.2))},
      {pxr::TfToken("range:max"), pxr::VtValue(pxr::GfVec3f(100000, 100000, 100000))},
      {pxr::TfToken("range:min"), pxr::VtValue(pxr::GfVec3f(0, 0, 0))}
    };
    ignition::math::Color diffuse = _material->Diffuse();
    create_material_input<pxr::GfVec3f>(
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
      {pxr::TfToken("range:max"), pxr::VtValue(pxr::GfVec3f(100000, 100000, 100000))},
      {pxr::TfToken("range:min"), pxr::VtValue(pxr::GfVec3f(0, 0, 0))}
    };
    ignition::math::Color emissive = _material->Emissive();
    create_material_input<pxr::GfVec3f>(
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

    create_material_input<bool>(
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
    create_material_input<float>(
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
        create_material_input<float>(
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
        create_material_input<float>(
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
          std::cerr << "pbrWorkflow->AlbedoMap() "<< pbrWorkflow->AlbedoMap() << '\n';
          std::map<pxr::TfToken, pxr::VtValue> customDataDiffuseTexture =
          {
            {pxr::TfToken("default"), pxr::VtValue(pxr::SdfAssetPath())},
          };
          create_material_input<pxr::SdfAssetPath>(
            shaderPrim,
            "diffuse_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(pbrWorkflow->AlbedoMap()),
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
          create_material_input<pxr::SdfAssetPath>(
            shaderPrim,
            "metallic_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(pbrWorkflow->MetalnessMap()),
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
          create_material_input<pxr::SdfAssetPath>(
            shaderPrim,
            "normalmap_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(pbrWorkflow->NormalMap()),
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
          create_material_input<pxr::SdfAssetPath>(
            shaderPrim,
            "reflectionroughness_texture",
            pxr::SdfValueTypeNames->Asset,
            pxr::SdfAssetPath(pbrWorkflow->RoughnessMap()),
            customDataRoughnessTexture,
            pxr::TfToken("RoughnessMap Map"),
            pxr::TfToken("RoughnessMap"),
            "",
            pxr::TfToken("raw"));

          std::map<pxr::TfToken, pxr::VtValue> customDataRoughnessTextureInfluence =
          {
            {pxr::TfToken("default"), pxr::VtValue(0)},
            {pxr::TfToken("range:max"), pxr::VtValue(1)},
            {pxr::TfToken("range:min"), pxr::VtValue(0)}
          };

          create_material_input<bool>(
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
