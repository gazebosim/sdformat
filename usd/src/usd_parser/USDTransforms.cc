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

#include "sdf/usd/usd_parser/USDTransforms.hh"
#include "sdf/usd/usd_parser/USDData.hh"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {

  void GetAllTransforms(
    const pxr::UsdPrim &_prim,
    USDData &_usdData,
    std::vector<ignition::math::Pose3d> &_tfs,
    ignition::math::Vector3d &_scale,
    const std::string &_schemaToStop)
  {
    pxr::UsdPrim parent = _prim;
    double metersPerUnit = 1.0;
    std::string upAxis = "Y";

    // this assumes that there can only be one stage
    auto stageData = _usdData.FindStage(parent.GetPath().GetName());
    if (stageData.second != nullptr) {
      metersPerUnit = stageData.second->MetersPerUnit();
      upAxis = stageData.second->UpAxis();
    }

    while (parent)
    {
      if (pxr::TfStringify(parent.GetPath()) == _schemaToStop)
      {
        return;
      }

      UDSTransforms t = ParseUSDTransform(parent);

      ignition::math::Pose3d pose;
      _scale *= t.scale;

      pose.Pos() = t.translate * metersPerUnit;
      // scaling is lost when we convert to pose, so we pre-scale the translation
      // to make them match the scaled values.
      if (!_tfs.empty()) {
        auto& child = _tfs.back();
        child.Pos().Set(
          child.Pos().X() * t.scale[0],
          child.Pos().Y() * t.scale[1],
          child.Pos().Z() * t.scale[2]);
      }

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

        _tfs.push_back(poseZ);
        _tfs.push_back(poseY);
        _tfs.push_back(poseX);
        _tfs.push_back(poseT);
      }
      parent = parent.GetParent();
    }

    if (upAxis == "Y")
    {
      ignition::math::Pose3d poseUpAxis = ignition::math::Pose3d(
        ignition::math::Vector3d(0 ,0 ,0),
        ignition::math::Quaterniond(IGN_PI_2, 0, 0));
      _tfs.push_back(poseUpAxis);
    }
  }

  void GetTransform(
    const pxr::UsdPrim &_prim,
    USDData &_usdData,
    ignition::math::Pose3d &_pose,
    ignition::math::Vector3d &_scale,
    const std::string &_schemaToStop)
  {
    std::vector<ignition::math::Pose3d> tfs;
    GetAllTransforms(_prim, _usdData, tfs, _scale, _schemaToStop);
    for (auto & rt : tfs)
    {
      _pose = rt * _pose;
    }
  }

  UDSTransforms ParseUSDTransform(const pxr::UsdPrim &_prim)
  {
    auto variant_geom = pxr::UsdGeomGprim(_prim);
    auto transforms = variant_geom.GetXformOpOrderAttr();

    pxr::GfVec3f scale(1, 1, 1);
    pxr::GfVec3f translate(0, 0, 0);
    pxr::GfQuatf rotationQuad(1, 0, 0, 0);

    UDSTransforms t;

    pxr::VtTokenArray xformOpOrder;
    transforms.Get(&xformOpOrder);
    for (auto & op: xformOpOrder)
    {
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
      }
      else if (op == "xformOp:rotateZYX" || op == "xformOp:rotateXYZ")
      {
        pxr::GfVec3f rotationEuler(0, 0, 0);
        auto attribute = _prim.GetAttribute(pxr::TfToken("xformOp:rotateZYX"));
        if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3f")
        {
          attribute.Get(&rotationEuler);
        }
        else if (attribute.GetTypeName().GetCPPTypeName() == "GfVec3d")
        {
          pxr::GfVec3d rotationEulerTmp(0, 0, 0);
          attribute.Get(&rotationEulerTmp);
          rotationEuler[0] = rotationEulerTmp[0];
          rotationEuler[1] = rotationEulerTmp[1];
          rotationEuler[2] = rotationEulerTmp[2];
        }
        ignition::math::Quaterniond qX, qY, qZ;
        ignition::math::Angle angleX(rotationEuler[0] * IGN_PI / 180.0);
        ignition::math::Angle angleY(rotationEuler[1] * IGN_PI / 180.0);
        ignition::math::Angle angleZ(rotationEuler[2] * IGN_PI / 180.0);
        qX = ignition::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
        qY = ignition::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
        qZ = ignition::math::Quaterniond(0, 0, angleZ.Normalized().Radian());

        t.q.push_back(qX);
        t.q.push_back(qY);
        t.q.push_back(qZ);
        t.isRotationZYX = true;
        t.isRotation = true;
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
        t.isRotation = true;
      }

      if (op == "xformOp:transform")
      {
        // FIXME: Shear is lost (does sdformat support it?).
        pxr::GfMatrix4d transform;
        _prim.GetAttribute(pxr::TfToken("xformOp:transform")).Get(&transform);
        const auto rot = transform.RemoveScaleShear();
        const auto scaleShear = transform * rot.GetInverse();

        t.scale[0] = scaleShear[0][0];
        t.scale[1] = scaleShear[1][1];
        t.scale[2] = scaleShear[2][2];

        const auto rotQuat = rot.ExtractRotationQuat();
        t.translate = ignition::math::Vector3d(transform[3][0], transform[3][1], transform[3][2]);
        ignition::math::Quaterniond q(
          rotQuat.GetReal(),
          rotQuat.GetImaginary()[0],
          rotQuat.GetImaginary()[1],
          rotQuat.GetImaginary()[2]
        );
        t.q.push_back(q);
        t.isTranslate = true;
        t.isRotation = true;
      }
    }
    return t;
  }
}
}
}
