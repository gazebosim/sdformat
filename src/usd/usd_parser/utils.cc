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

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Helpers.hh>

#include "sdf/Pbr.hh"

namespace usd
{
  std::string directoryFromUSDPath(std::string &_primPath)
  {
    std::vector<std::string> tokensChild = ignition::common::split(_primPath, "/");
    std::string directoryMesh;
    if (tokensChild.size() > 1)
    {
      directoryMesh = tokensChild[0];
      for (unsigned int i = 1; i < tokensChild.size() - 1; ++i)
      {
          directoryMesh = ignition::common::joinPaths(directoryMesh, tokensChild[i+1]);
      }
    }
    return directoryMesh;
  }

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

  sdf::Material ParseMaterial(const pxr::UsdPrim &_prim)
  {
    sdf::Material material;
    if(_prim.IsA<pxr::UsdGeomGprim>())
    {
      std::cerr << "Normal Color" << '\n';
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
      for (const auto & child : _prim.GetChildren())
      {
        std::cerr << "\tchild " << pxr::TfStringify(child.GetPath()) << '\n';

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
            std::cerr << "\tGetFullName " << input.GetFullName() << " " << input.GetBaseName() << '\n';
            if (input.GetBaseName() == "diffuse_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput diffuseTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("diffuse_texture"));
              auto source = diffuseTextureShaderInput.GetConnectedSources();
              diffuseTextureShaderInput.Get(&materialPath);
              std::cerr << "\tAlbedoMap " << materialPath.GetAssetPath() << '\n';
              pbrWorkflow.SetAlbedoMap(materialPath.GetAssetPath());
              isPBR = true;
            }
            else if (input.GetBaseName() == "normalmap_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput normalTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("normalmap_texture"));
              auto source = normalTextureShaderInput.GetConnectedSources();
              normalTextureShaderInput.Get(&materialPath);
              std::cerr << "\tNormalMap " << materialPath.GetAssetPath() << '\n';
              pbrWorkflow.SetNormalMap(materialPath.GetAssetPath());
              isPBR = true;
            }
            else if (input.GetBaseName() == "reflectionroughness_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput roughnessTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("reflectionroughness_texture"));
              auto source = roughnessTextureShaderInput.GetConnectedSources();
              roughnessTextureShaderInput.Get(&materialPath);
              std::cerr << "\troughnessMap " << materialPath.GetAssetPath() << '\n';
              pbrWorkflow.SetRoughnessMap(materialPath.GetAssetPath());
              isPBR = true;
            }
            else if (input.GetBaseName() == "metallic_texture")
            {
              pxr::SdfAssetPath materialPath;
              pxr::UsdShadeInput metallicTextureShaderInput =
                variantshader.GetInput(pxr::TfToken("metallic_texture"));
              auto source = metallicTextureShaderInput.GetConnectedSources();
              metallicTextureShaderInput.Get(&materialPath);
              std::cerr << "\tMetalnessMap " << materialPath.GetAssetPath() << '\n';
              pbrWorkflow.SetMetalnessMap(materialPath.GetAssetPath());
              isPBR = true;
            }
            else if (input.GetBaseName() == "diffuse_color_constant")
            {
              auto sourceInfoV = input.GetConnectedSources();
              if (sourceInfoV.size() > 0)
              {
                pxr::UsdShadeInput connectedInput =
                  sourceInfoV[0].source.GetInput(sourceInfoV[0].sourceName);

                const pxr::SdfPath& thisAttrPath = connectedInput.GetAttr().GetPath();
                auto connectedPrim = _prim.GetStage()->GetPrimAtPath(thisAttrPath.GetPrimPath());
                if(connectedPrim)
                  connectedPrim.GetAttribute(pxr::TfToken("inputs:diffuse_color_constant")).Get(&diffuseColor);

              }
              else
              {
                pxr::UsdShadeInput diffuseShaderInput =
                  variantshader.GetInput(pxr::TfToken("diffuse_color_constant"));
                diffuseShaderInput.Get(&diffuseColor);
              }
              material.SetDiffuse(
                ignition::math::Color(
                  diffuseColor[0],
                  diffuseColor[1],
                  diffuseColor[2]));
              std::cerr << "\tdiffuse " << ignition::math::Color(
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
              auto sourceInfoV = input.GetConnectedSources();
              if (sourceInfoV.size() > 0)
              {
                pxr::UsdShadeInput connectedInput =
                  sourceInfoV[0].source.GetInput(sourceInfoV[0].sourceName);

                const pxr::SdfPath& thisAttrPath = connectedInput.GetAttr().GetPath();
                auto connectedPrim = _prim.GetStage()->GetPrimAtPath(thisAttrPath.GetPrimPath());
                if(connectedPrim)
                {
                  float reflectionRoughnessConstant;
                  connectedPrim.GetAttribute(pxr::TfToken("inputs:reflection_roughness_constant")).Get(&reflectionRoughnessConstant);
                  pbrWorkflow.SetRoughness(reflectionRoughnessConstant);
                  isPBR = true;
                }
              }
              else
              {
                pxr::UsdShadeInput reflectionRoughnessConstantShaderInput =
                  variantshader.GetInput(pxr::TfToken("reflection_roughness_constant"));
                float reflectionRoughnessConstant;
                reflectionRoughnessConstantShaderInput.Get(&reflectionRoughnessConstant);
                pbrWorkflow.SetRoughness(reflectionRoughnessConstant);
                isPBR = true;
              }
            }
            else if (input.GetBaseName() == "enable_emission")
            {
              pxr::UsdShadeInput enableEmissiveShaderInput =
                variantshader.GetInput(pxr::TfToken("enable_emission"));
              enableEmissiveShaderInput.Get(&enableEmission);
              std::cerr << "\tenableEmission " << enableEmission << '\n';
            }
            else if (input.GetBaseName() == "emissive_color")
            {
                pxr::UsdShadeInput emissiveColorShaderInput =
                  variantshader.GetInput(pxr::TfToken("emissive_color"));
                if (emissiveColorShaderInput.Get(&emissiveColor))
                {
                  std::cerr << "\temissiveColor " << emissiveColor << '\n';

                  emissiveColorCommon = ignition::math::Color(
                    emissiveColor[0],
                    emissiveColor[1],
                    emissiveColor[2]);
                }
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
        }
      }
    }
    return material;
  }

  void GetAllTransforms(
    const pxr::UsdPrim &_prim,
    USDData &_usdData,
    std::vector<ignition::math::Pose3d> &_tfs,
    ignition::math::Vector3d &_scale,
    const std::string &_name)
  {
    pxr::UsdPrim parent = _prim;
    double metersPerUnit = 1.0;
    while(parent)
    {
      if (pxr::TfStringify(parent.GetPath()) == _name)
      {
        return;
      }

      Transforms t = ParseTransform(parent);

      ignition::math::Pose3d pose;
      _scale *= t.scale;

      std::cout << "parent.GetPath().GetName() " << parent.GetPath().GetName() << '\n';

      std::pair<std::string, std::shared_ptr<USDStage>> data =
        _usdData.findStage(parent.GetPath().GetName());

      if (data.second != nullptr)
      {
        metersPerUnit = data.second->_metersPerUnit;
        // std::string upAxis = data.second->_upAxis;
        // std::cerr << "upAxis " << upAxis << '\n';
        // if(upAxis == "Y")
        // {
        //   ignition::math::Matrix4d m(
        //     1, 0, 0, 0,
        //     0, 0, -1, 0,
        //     0, 1, 0, 0,
        //     0, 0, 0, 1);
        //   t.translate = m * t.translate;
        //
        //   ignition::math::Pose3d poseAxisUpY = ignition::math::Pose3d(
        //     ignition::math::Vector3d(0 ,0 ,0),
        //     ignition::math::Quaterniond(0.707, 0.707, 0, 0));
        //
        //   _tfs.push_back(poseAxisUpY);
        //
        //   // std::cerr << "m.Rotation() " << m.Rotation() << '\n';
        //   // exit(-1);
        //   // // ignition::math::Pose3d translateUpAxisY = ignition::math::Pose3d(
        //   // //   t.translate,
        //   // //   ignition::math::Quaterniond(0, 0, 0));
        //   // t.translate = ignition::math::Vector3d(t.translate[0], -t.translate[2], t.translate[1]);
        //   // if (!t.isRotationZYX)
        //   // {
        //   //   if (t.isRotation)
        //   //   {
        //   //     auto eulerAngles = t.q[0].Euler();
        //   //     t.q[0] = ignition::math::Quaterniond(eulerAngles[0], eulerAngles[2], eulerAngles[1]);
        //   //   }
        //   // }
        //   // else
        //   // {
        //   //   auto qtemp = t.q[2];
        //   //   t.q[2] = t.q[1];
        //   //   t.q[1] = qtemp;
        //   // }
        // }
      }

      pose.Pos() = t.translate * metersPerUnit;

      if (!t.isRotationZYX)
      {
        if (t.isRotation)
        {
          pose.Rot() = t.q[0];
        }
        _tfs.push_back(pose);
      }
      else
      {
        ignition::math::Pose3d poseZ = ignition::math::Pose3d(
          ignition::math::Vector3d(0 ,0 ,0), t.q[2]);
        ignition::math::Pose3d poseY = ignition::math::Pose3d(
          ignition::math::Vector3d(0 ,0 ,0), t.q[1]);
        ignition::math::Pose3d poseX = ignition::math::Pose3d(
          ignition::math::Vector3d(0 ,0 ,0), t.q[0]);

        ignition::math::Pose3d poseT = ignition::math::Pose3d(
          t.translate * metersPerUnit,
          ignition::math::Quaterniond(1, 0, 0, 0));
        if (data.second != nullptr)
        {
          std::string upAxis = data.second->_upAxis;
          if (upAxis == "Z")
          {
            _tfs.push_back(poseZ);
            _tfs.push_back(poseY);
          }
          else
          {
            ignition::math::Pose3d poseAxisUpY = ignition::math::Pose3d(
              ignition::math::Vector3d(0 ,0 ,0),
              ignition::math::Quaterniond(0.707, -0.707, 0, 0));
            _tfs.push_back(poseAxisUpY);
            _tfs.push_back(poseY);
            _tfs.push_back(poseZ);
          }
        }

        _tfs.push_back(poseX);
        _tfs.push_back(poseT);
      }
      parent = parent.GetParent();
    }
  }

  void GetTransform(
    const pxr::UsdPrim &_prim,
    USDData &_usdData,
    ignition::math::Pose3d &_pose,
    ignition::math::Vector3d &_scale,
    const std::string &_name)
  {
    std::vector<ignition::math::Pose3d> tfs;

    GetAllTransforms(_prim, _usdData, tfs, _scale, _name);
    std::cerr << "pose " << _pose << '\n';
    for (auto & rt : tfs)
    {
      _pose = rt * _pose;
      std::cerr << "pose " << _pose << '\n';
    }
  }

  Transforms ParseTransform(const pxr::UsdPrim &_prim)
  {
    auto variant_geom = pxr::UsdGeomGprim(_prim);
    auto transforms = variant_geom.GetXformOpOrderAttr();

    pxr::GfVec3f scale(1, 1, 1);
    pxr::GfVec3f translate(0, 0, 0);
    pxr::GfQuatf rotationQuad(1, 0, 0, 0);

    Transforms t;

    pxr::VtTokenArray xformOpOrder;
    transforms.Get(&xformOpOrder);
    for (auto & op: xformOpOrder)
    {
      std::cerr << "xformOpOrder " << op << '\n';
      std::string s = op;
      if (op == "xformOp:scale")
      {
        auto attribute = _prim.GetAttribute(pxr::TfToken("xformOp:scale"));
        if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3f")
        {
          attribute.Get(&scale);
        }
        else if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3d")
        {
          pxr::GfVec3d scaleTmp(1, 1, 1);
          attribute.Get(&scaleTmp);
          scale[0] = scaleTmp[0];
          scale[1] = scaleTmp[1];
          scale[2] = scaleTmp[2];
        }
        t.scale = ignition::math::Vector3d(scale[0], scale[1], scale[2]);
        std::cerr << "scale "<< scale << '\n';
      }
      else if (op == "xformOp:rotateZYX")
      {
        pxr::GfVec3f rotationEuler(0, 0, 0);
        auto attribute = _prim.GetAttribute(pxr::TfToken("xformOp:rotateZYX"));
        if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3f")
        {
          attribute.Get(&rotationEuler);
        }
        else if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3d")
        {
          pxr::GfVec3f rotationEulerTmp(0, 0, 0);
          attribute.Get(&rotationEulerTmp);
          rotationEuler[0] = rotationEulerTmp[0];
          rotationEuler[1] = rotationEulerTmp[1];
          rotationEuler[2] = rotationEulerTmp[2];
        }

        ignition::math::Quaterniond qX, qY, qZ;

        ignition::math::Angle angleX(rotationEuler[0] * 3.1416 / 180.0);
        ignition::math::Angle angleY(rotationEuler[1] * 3.1416 / 180.0);
        ignition::math::Angle angleZ(rotationEuler[2] * 3.1416 / 180.0);
        qX = ignition::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
        qY = ignition::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
        qZ = ignition::math::Quaterniond(0, 0, angleZ.Normalized().Radian());
        t.q.push_back(qX);
        t.q.push_back(qY);
        t.q.push_back(qZ);
        t.isRotationZYX = true;
        t.isRotation = true;
        // std::cerr << "euler rot " << rotationEuler << " q: " << q << " | " << q.W() << " " << q.X() << " " << q.Y() << " " << q.Z() << '\n';
        std::cerr << "euler rot " << rotationEuler << " qX: " << qX << " | " << qX.W() << " " << qX.X() << " " << qX.Y() << " " << qX.Z() << '\n';
        std::cerr << "euler rot " << rotationEuler << " qY: " << qY << " | " << qY.W() << " " << qY.X() << " " << qY.Y() << " " << qY.Z() << '\n';
        std::cerr << "euler rot " << rotationEuler << " qZ: " << qZ << " | " << qZ.W() << " " << qZ.X() << " " << qZ.Y() << " " << qZ.Z() << '\n';
      }
      else if (op == "xformOp:translate")
      {
        auto attribute = _prim.GetAttribute(pxr::TfToken("xformOp:translate"));
        if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3f")
        {
          attribute.Get(&translate);
        }
        else if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3d")
        {
          pxr::GfVec3d translateTmp(0, 0, 0);
          attribute.Get(&translateTmp);
          translate[0] = translateTmp[0];
          translate[1] = translateTmp[1];
          translate[2] = translateTmp[2];
        }
        t.translate = ignition::math::Vector3d(translate[0], translate[1], translate[2]);
        std::cerr << "translate " << translate << '\n';
        t.isTranslate = true;
      }
      else if (op == "xformOp:orient")
      {
        auto attribute = _prim.GetAttribute(pxr::TfToken("xformOp:orient"));
        if (attribute.GetTypeName().GetCPPTypeName() == "GfQuatf")
        {
          attribute.Get(&rotationQuad);
        }
        else if (attribute.GetTypeName().GetCPPTypeName() == "GfQuatd")
        {
          pxr::GfQuatd rotationQuadTmp;
          attribute.Get(&rotationQuadTmp);
          rotationQuad.SetImaginary(
            rotationQuadTmp.GetImaginary()[0],
            rotationQuadTmp.GetImaginary()[1],
            rotationQuadTmp.GetImaginary()[2]);
          rotationQuad.SetReal(rotationQuadTmp.GetReal());
        }
        ignition::math::Quaterniond q(
          rotationQuad.GetReal(),
          rotationQuad.GetImaginary()[0],
          rotationQuad.GetImaginary()[1],
          rotationQuad.GetImaginary()[2]);
        t.q.push_back(q);
        std::cerr << "rotationQuad " << rotationQuad << '\n';
        t.isRotation = true;
      }

      if (op == "xformOp:transform")
      {
        pxr::GfMatrix4d transform;
        _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
        pxr::GfVec3d translateMatrix = transform.ExtractTranslation();
        pxr::GfQuatd rotation_quadMatrix = transform.ExtractRotationQuat();

        ignition::math::Matrix4d m(
          transform[0][0], transform[0][1], transform[0][2], transform[0][3],
          transform[1][0], transform[1][1], transform[1][2], transform[1][3],
          transform[2][0], transform[2][1], transform[2][2], transform[2][3],
          transform[3][0], transform[3][1], transform[3][2], transform[3][3]
        );
        std::cerr << "m " << m << '\n';
        ignition::math::Vector3d eulerAngles = m.EulerRotation(true);
        std::cerr << "eulerAngles " << eulerAngles << '\n';
        ignition::math::Matrix4d inverseRX(ignition::math::Pose3d(
          ignition::math::Vector3d(0, 0, 0),
          ignition::math::Quaterniond(-eulerAngles[0], 0, 0)));
        ignition::math::Matrix4d inverseRY(ignition::math::Pose3d(
          ignition::math::Vector3d(0, 0, 0),
          ignition::math::Quaterniond(0, -eulerAngles[1], 0)));
        ignition::math::Matrix4d inverseRZ(ignition::math::Pose3d(
          ignition::math::Vector3d(0, 0, 0),
          ignition::math::Quaterniond(0, 0, -eulerAngles[2])));

        pxr::GfMatrix4d inverseR2X(
          inverseRX(0, 0), inverseRX(0, 1), inverseRX(0, 2), inverseRX(0, 3),
          inverseRX(1, 0), inverseRX(1, 1), inverseRX(1, 2), inverseRX(1, 3),
          inverseRX(2, 0), inverseRX(2, 1), inverseRX(2, 2), inverseRX(2, 3),
          inverseRX(3, 0), inverseRX(3, 1), inverseRX(3, 2), inverseRX(3, 3));
        pxr::GfMatrix4d inverseR2Y(
          inverseRY(0, 0), inverseRY(0, 1), inverseRY(0, 2), inverseRY(0, 3),
          inverseRY(1, 0), inverseRY(1, 1), inverseRY(1, 2), inverseRY(1, 3),
          inverseRY(2, 0), inverseRY(2, 1), inverseRY(2, 2), inverseRY(2, 3),
          inverseRY(3, 0), inverseRY(3, 1), inverseRY(3, 2), inverseRY(3, 3));
        pxr::GfMatrix4d inverseR2Z(
          inverseRZ(0, 0), inverseRZ(0, 1), inverseRZ(0, 2), inverseRZ(0, 3),
          inverseRZ(1, 0), inverseRZ(1, 1), inverseRZ(1, 2), inverseRZ(1, 3),
          inverseRZ(2, 0), inverseRZ(2, 1), inverseRZ(2, 2), inverseRZ(2, 3),
          inverseRZ(3, 0), inverseRZ(3, 1), inverseRZ(3, 2), inverseRZ(3, 3));

        m = inverseRX * (inverseRY * (inverseRZ * m));
        transform = inverseR2X * (inverseR2Y * (inverseR2Z * transform));
        // ignition::math::Vector3d t = m.Translation();
        // ignition::math::Vector3d euler = m.EulerRotation(true);
        // ignition::math::Quaternion r(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
        std::cerr << "m " << m << '\n';

        // std::cerr << "m " << m << '\n';
        // std::cerr << "transform " << transform << '\n';

        t.scale[0] = transform[0][0];
        t.scale[1] = transform[1][1];
        t.scale[2] = transform[2][2];

        pxr::GfVec3d translateVector = transform.ExtractTranslation();
        pxr::GfQuatd rotationInversed = transform.ExtractRotationQuat();
        t.translate = ignition::math::Vector3d(
          translateVector[0], translateVector[1], translateVector[2]);
        ignition::math::Quaterniond q(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
        t.q.push_back(q);
        // translate[0] = translateVector[0];
        // translate[1] = translateVector[1];
        // translate[2] = translateVector[2];
        // rotationQuad.SetImaginary(r.X(), r.Y(), r.Z());
        // rotationQuad.SetReal(r.W());

        // isTranslate = true;
        // isRotation = true;
        // isScale = true;
        t.isTranslate = true;
        t.isRotation = true;

        std::cerr << "translate " << t.translate << '\n';
        std::cerr << "rotation_quad " << q << '\n';
        std::cerr << "scale " << t.scale << '\n';
      }
    }
    return t;
  }
}
