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

#include <optional>
#include <utility>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "sdf/usd/usd_parser/USDData.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
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
  public: gz::math::Vector3d scale{1, 1, 1};

  /// \brief Rotation of the schema
  public: std::optional<gz::math::Quaterniond> q = std::nullopt;

  /// \brief Translation of the schema
  public: gz::math::Vector3d translate{0, 0, 0};
};

/////////////////////////////////////////////////
UDSTransforms::UDSTransforms()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
const gz::math::Vector3d UDSTransforms::Translation() const
{
  return this->dataPtr->translate;
}

//////////////////////////////////////////////////
const gz::math::Vector3d UDSTransforms::Scale() const
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
const std::optional<gz::math::Quaterniond> UDSTransforms::Rotation() const
{
  return this->dataPtr->q;
}

//////////////////////////////////////////////////
void UDSTransforms::SetTranslation(
  const gz::math::Vector3d &_translate)
{
  this->dataPtr->translate = _translate;
}

//////////////////////////////////////////////////
void UDSTransforms::SetScale(
  const gz::math::Vector3d &_scale)
{
  this->dataPtr->scale = _scale;
}

//////////////////////////////////////////////////
void UDSTransforms::SetRotation(
  const gz::math::Quaterniond &_q)
{
  this->dataPtr->q = _q;
}

//////////////////////////////////////////////////
/// \brief This function will parse all the parent transforms of a prim.
/// \param[in] _prim Initial prim to read the transform
/// \param[in] _usdData USDData structure to get info about the prim, for
/// example: metersperunit
/// \param[out] _tfs A vector with all the transforms
/// \param[out] _scale The scale of the prims
/// \param[in] _schemaToStop Name of the prim where the loop will stop
/// reading transforms
void GetAllTransforms(
  const pxr::UsdPrim &_prim,
  const USDData &_usdData,
  std::vector<gz::math::Pose3d> &_tfs,
  gz::math::Vector3d &_scale,
  const std::string &_schemaToStop)
{
  pxr::UsdPrim parent = _prim;
  double metersPerUnit = 1.0;
  std::string upAxis = "Y";

  // this assumes that there can only be one stage
  const auto stageData = _usdData.FindStage(parent.GetPath().GetName());
  if (stageData.second) {
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

    gz::math::Pose3d pose;
    _scale *= t.Scale();

    pose.Pos() = t.Translation() * metersPerUnit;
    // scaling is lost when we convert to pose, so we pre-scale the
    // translation to make them match the scaled values.
    if (!_tfs.empty()) {
      auto& child = _tfs.back();
      child.Pos().Set(
        child.Pos().X() * t.Scale()[0],
        child.Pos().Y() * t.Scale()[1],
        child.Pos().Z() * t.Scale()[2]);
    }

    if (t.Rotation())
    {
      pose.Rot() = t.Rotation().value();
    }
    _tfs.push_back(pose);
    parent = parent.GetParent();
  }

  if (upAxis == "Y")
  {
    // Add additional rotation to match with Z up Axis.
    // TODO(anyone) handle upAxis == "X". This is a case that is rarely
    // used by other renderers
    gz::math::Pose3d poseUpAxis = gz::math::Pose3d(
      gz::math::Vector3d(0, 0, 0),
      gz::math::Quaterniond(GZ_PI_2, 0, 0));
    _tfs.push_back(poseUpAxis);
  }
}

//////////////////////////////////////////////////
void GetTransform(
  const pxr::UsdPrim &_prim,
  const USDData &_usdData,
  gz::math::Pose3d &_pose,
  gz::math::Vector3d &_scale,
  const std::string &_schemaToStop)
{
  std::vector<gz::math::Pose3d> tfs;
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
  for (const auto &op : xformOpOrder)
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
        scale[0] = static_cast<float>(scaleTmp[0]);
        scale[1] = static_cast<float>(scaleTmp[1]);
        scale[2] = static_cast<float>(scaleTmp[2]);
      }
      t.SetScale(gz::math::Vector3d(scale[0], scale[1], scale[2]));
    }
    else if (op == kXFormOpRotateZYX || op == kXFormOpRotateXYZ)
    {
      pxr::GfVec3f rotationEuler(0, 0, 0);
      pxr::UsdAttribute attribute;
      if (op == kXFormOpRotateZYX)
      {
        attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpRotateZYX));
      }
      else
      {
        attribute = _prim.GetAttribute(pxr::TfToken(kXFormOpRotateXYZ));
      }
      if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3fString)
      {
        attribute.Get(&rotationEuler);
      }
      else if (attribute.GetTypeName().GetCPPTypeName() == kGfVec3dString)
      {
        pxr::GfVec3d rotationEulerTmp(0, 0, 0);
        attribute.Get(&rotationEulerTmp);
        rotationEuler[0] = static_cast<float>(rotationEulerTmp[0]);
        rotationEuler[1] = static_cast<float>(rotationEulerTmp[1]);
        rotationEuler[2] = static_cast<float>(rotationEulerTmp[2]);
      }
      gz::math::Quaterniond qX, qY, qZ;
      gz::math::Angle angleX(GZ_DTOR(rotationEuler[0]));
      gz::math::Angle angleY(GZ_DTOR(rotationEuler[1]));
      gz::math::Angle angleZ(GZ_DTOR(rotationEuler[2]));
      qX = gz::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
      qY = gz::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
      qZ = gz::math::Quaterniond(0, 0, angleZ.Normalized().Radian());

      // TODO(ahcorde) This part should be reviewed, revisit how rotateXYZ
      // and rotateZYX are handle.
      // Related issue https://github.com/gazebosim/sdformat/issues/926
      // if (op == kXFormOpRotateZYX)
      // {
      //   std::swap(angleX, angleZ);
      // }
      t.SetRotation((qX * qY) * qZ);
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
        translate[0] = static_cast<float>(translateTmp[0]);
        translate[1] = static_cast<float>(translateTmp[1]);
        translate[2] = static_cast<float>(translateTmp[2]);
      }
      t.SetTranslation(gz::math::Vector3d(
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
      else if (attribute.GetTypeName().GetCPPTypeName() == kGfQuatdString)
      {
        pxr::GfQuatd rotationQuadTmp;
        attribute.Get(&rotationQuadTmp);
        rotationQuad.SetImaginary(
          rotationQuadTmp.GetImaginary()[0],
          rotationQuadTmp.GetImaginary()[1],
          rotationQuadTmp.GetImaginary()[2]);
        rotationQuad.SetReal(rotationQuadTmp.GetReal());
      }
      gz::math::Quaterniond q(
        rotationQuad.GetReal(),
        rotationQuad.GetImaginary()[0],
        rotationQuad.GetImaginary()[1],
        rotationQuad.GetImaginary()[2]);
      t.SetRotation(q);
    }

    if (op == kXFormOpTransform)
    {
      // TODO(koonpeng) Shear is lost (does sdformat support it?).
      pxr::GfMatrix4d transform;
      _prim.GetAttribute(pxr::TfToken(kXFormOpTransform)).Get(&transform);
      const auto rot = transform.RemoveScaleShear();
      const auto scaleShear = transform * rot.GetInverse();

      t.SetScale(gz::math::Vector3d(
        scaleShear[0][0],
        scaleShear[1][1],
        scaleShear[2][2]));

      const auto rotQuat = rot.ExtractRotationQuat();
      t.SetTranslation(gz::math::Vector3d(
        transform[3][0],
        transform[3][1],
        transform[3][2]));
      gz::math::Quaterniond q(
        rotQuat.GetReal(),
        rotQuat.GetImaginary()[0],
        rotQuat.GetImaginary()[1],
        rotQuat.GetImaginary()[2]
      );
      t.SetRotation(q);
    }
  }
  return t;
}
}
}
}
