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

const char kXFormOpTranslate[] = {"xformOp:translate"};
const char kXFormOpOrient[] = {"xformOp:orient"};
const char kXFormOpTransform[] = {"xformOp:transform"};
const char kXFormOpScale[] = {"xformOp:scale"};
const char kXFormOpRotateXYZ[] = {"xformOp:rotateXYZ"};
const char kXFormOpRotateZYX[] = {"xformOp:rotateZYX"};

const char kGfVec3fString[] = {"GfVec3f"};
const char kGfVec3dString[] = {"GfVec3d"};
const char kGfQuatfString[] = {"GfQuatf"};
const char kGfQuatdString[] = {"GfQuatd"};

/// \brief Private altimeter data.
class UDSTransforms::Implementation
{
  /// \brief Scale of the schema
  public: ignition::math::Vector3d scale{1, 1, 1};

  /// \brief Rotation of the schema
  public: std::vector<ignition::math::Quaterniond> q;

  /// \brief Translatio of the schema
  public: ignition::math::Vector3d translate{0, 0, 0};

  /// \brief True if there is a rotation ZYX defined or false otherwise
  public: bool isRotationZYX = false;

  /// \brief True if there is a rotation XYZ defined or false otherwise
  public: bool isRotationXYZ = false;

  /// \brief True if there is a rotation (as a quaterion) defined
  /// or false otherwise
  public: bool isRotation = false;
};

/////////////////////////////////////////////////
UDSTransforms::UDSTransforms()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
ignition::math::Vector3d UDSTransforms::Translate()
{
  return this->dataPtr->translate;
}

//////////////////////////////////////////////////
ignition::math::Vector3d UDSTransforms::Scale()
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
std::vector<ignition::math::Quaterniond>
  UDSTransforms::Rotations()
{
  return this->dataPtr->q;
}

//////////////////////////////////////////////////
void UDSTransforms::SetTranslate(
  const ignition::math::Vector3d &_translate)
{
  this->dataPtr->translate = _translate;
}

//////////////////////////////////////////////////
void UDSTransforms::SetScale(
  const ignition::math::Vector3d &_scale)
{
  this->dataPtr->scale = _scale;
}

//////////////////////////////////////////////////
void UDSTransforms::AddRotation(
  const ignition::math::Quaterniond &_q)
{
  this->dataPtr->q.push_back(_q);
}

//////////////////////////////////////////////////
bool UDSTransforms::RotationZYX()
{
  return this->dataPtr->isRotationZYX;
}

//////////////////////////////////////////////////
bool UDSTransforms::RotationXYZ()
{
  return this->dataPtr->isRotationXYZ;
}

//////////////////////////////////////////////////
bool UDSTransforms::Rotation()
{
  return this->dataPtr->isRotation;
}

//////////////////////////////////////////////////
void UDSTransforms::SetRotationZYX(bool _rotationZYX)
{
  this->dataPtr->isRotationZYX = _rotationZYX;
}

//////////////////////////////////////////////////
void UDSTransforms::SetRotationXYZ(bool _rotationXYZ)
{
  this->dataPtr->isRotationXYZ = _rotationXYZ;
}

//////////////////////////////////////////////////
void UDSTransforms::SetRotation(bool _rotation)
{
  this->dataPtr->isRotation = _rotation;
}

//////////////////////////////////////////////////
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
    _scale *= t.Scale();

    pose.Pos() = t.Translate() * metersPerUnit;
    // scaling is lost when we convert to pose, so we pre-scale the
    // translation to make them match the scaled values.
    if (!_tfs.empty()) {
      auto& child = _tfs.back();
      child.Pos().Set(
        child.Pos().X() * t.Scale()[0],
        child.Pos().Y() * t.Scale()[1],
        child.Pos().Z() * t.Scale()[2]);
    }

    if (!t.RotationZYX() && !t.RotationXYZ())
    {
      if (t.Rotation())
      {
        pose.Rot() = t.Rotations()[0];
      }
      _tfs.push_back(pose);
    }
    else
    {
      ignition::math::Pose3d poseZ = ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0 ,0), t.Rotations()[2]);
      ignition::math::Pose3d poseY = ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0 ,0), t.Rotations()[1]);
      ignition::math::Pose3d poseX = ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0 ,0), t.Rotations()[0]);

      ignition::math::Pose3d poseT = ignition::math::Pose3d(
        t.Translate() * metersPerUnit,
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
      ignition::math::Vector3d(0, 0 ,0),
      ignition::math::Quaterniond(IGN_PI_2, 0, 0));
    _tfs.push_back(poseUpAxis);
  }
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
UDSTransforms ParseUSDTransform(const pxr::UsdPrim &_prim)
{
  auto variantGeom = pxr::UsdGeomGprim(_prim);
  auto transforms = variantGeom.GetXformOpOrderAttr();

  pxr::GfVec3f scale(1, 1, 1);
  pxr::GfVec3f translate(0, 0, 0);
  pxr::GfQuatf rotationQuad(1, 0, 0, 0);

  UDSTransforms t;

  pxr::VtTokenArray xformOpOrder;
  transforms.Get(&xformOpOrder);
  for (auto & op : xformOpOrder)
  {
    if (op == kXFormOpScale)
    {
      auto attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpScale));
      if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3fString)
      {
        attribute.Get(&scale);
      }
      else if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3dString)
      {
        pxr::GfVec3d scaleTmp(1, 1, 1);
        attribute.Get(&scaleTmp);
        scale[0] = scaleTmp[0];
        scale[1] = scaleTmp[1];
        scale[2] = scaleTmp[2];
      }
      t.SetScale(ignition::math::Vector3d(scale[0], scale[1], scale[2]));
    }
    else if (op == kXFormOpRotateZYX || op == kXFormOpRotateXYZ)
    {
      pxr::GfVec3f rotationEuler(0, 0, 0);
      pxr::UsdAttribute attribute;
      if (op == kXFormOpRotateZYX)
      {
        attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpRotateZYX));
        t.SetRotationZYX(true);
      }
      else
      {
        attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpRotateXYZ));
        t.SetRotationXYZ(true);
      }
      if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3fString)
      {
        attribute.Get(&rotationEuler);
      }
      else if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3dString)
      {
        pxr::GfVec3d rotationEulerTmp(0, 0, 0);
        attribute.Get(&rotationEulerTmp);
        rotationEuler[0] = rotationEulerTmp[0];
        rotationEuler[1] = rotationEulerTmp[1];
        rotationEuler[2] = rotationEulerTmp[2];
      }
      ignition::math::Quaterniond qX, qY, qZ;
      ignition::math::Angle angleX(IGN_DTOR(rotationEuler[0]));
      ignition::math::Angle angleY(IGN_DTOR(rotationEuler[1]));
      ignition::math::Angle angleZ(IGN_DTOR(rotationEuler[2]));
      qX = ignition::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
      qY = ignition::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
      qZ = ignition::math::Quaterniond(0, 0, angleZ.Normalized().Radian());

      t.AddRotation(qX);
      t.AddRotation(qY);
      t.AddRotation(qZ);
      t.SetRotation(true);
    }
    else if (op == kXFormOpTranslate)
    {
      auto attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpTranslate));
      if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3fString)
      {
        attribute.Get(&translate);
      }
      else if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3dString)
      {
        pxr::GfVec3d translateTmp(0, 0, 0);
        attribute.Get(&translateTmp);
        translate[0] = translateTmp[0];
        translate[1] = translateTmp[1];
        translate[2] = translateTmp[2];
      }
      t.SetTranslate(ignition::math::Vector3d(
        translate[0],
        translate[1],
        translate[2]));
    }
    else if (op == kXFormOpOrient)
    {
      auto attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpOrient));
      if (attribute.GetTypeName().GetCPPTypeName() == kGfQuatfString)
      {
        attribute.Get(&rotationQuad);
      }
      else if (attribute.GetTypeName().GetCPPTypeName() == kGfQuatfString)
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
      t.AddRotation(q);
      t.SetRotation(true);
    }

    if (op == kXFormOpTransform)
    {
      // FIXME: Shear is lost (does sdformat support it?).
      pxr::GfMatrix4d transform;
      _prim.GetAttribute(pxr::TfToken(kXFormOpTransform)).Get(&transform);
      const auto rot = transform.RemoveScaleShear();
      const auto scaleShear = transform * rot.GetInverse();

      t.SetScale(ignition::math::Vector3d(
        scaleShear[0][0],
        scaleShear[1][1],
        scaleShear[2][2]));

      const auto rotQuat = rot.ExtractRotationQuat();
      t.SetTranslate(ignition::math::Vector3d(
        transform[3][0],
        transform[3][1],
        transform[3][2]));
      ignition::math::Quaterniond q(
        rotQuat.GetReal(),
        rotQuat.GetImaginary()[0],
        rotQuat.GetImaginary()[1],
        rotQuat.GetImaginary()[2]
      );
      t.AddRotation(q);
      t.SetRotation(true);
    }
  }
  return t;
}
}
}
}
