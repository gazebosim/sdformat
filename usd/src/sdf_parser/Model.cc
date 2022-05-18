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

#include "Model.hh"

#include <string>
#include <unordered_map>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Model.hh"
#include "../UsdUtils.hh"
#include "Joint.hh"
#include "Link.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  UsdErrors ParseSdfModel(const sdf::Model &_model, pxr::UsdStageRefPtr &_stage,
      const std::string &_path, const pxr::SdfPath &_worldPath)
  {
    UsdErrors errors;

    if (_model.ModelCount())
    {
      errors.push_back(UsdError(
            sdf::Error(sdf::ErrorCode::NESTED_MODELS_UNSUPPORTED,
              "Nested models currently aren't supported.")));
      return errors;
    }

    const pxr::SdfPath sdfModelPath(_path);
    auto usdModelXform = pxr::UsdGeomXform::Define(_stage, sdfModelPath);
    // since USD does not have a plane yet, planes are being represented as a
    // wide, thin box. The plane/box pose needs to be offset according to the
    // plane thickness to ensure that the top of the plane is at the correct
    // height. This pose offset workaround will no longer be needed when a
    // pxr::USDGeomPlane class is created:
    // https://graphics.pixar.com/usd/release/wp_rigid_body_physics.html#plane-shapes
    if (usd::IsPlane(_model))
    {
      gz::math::Vector3d planePosition(
          _model.RawPose().X(),
          _model.RawPose().Y(),
          _model.RawPose().Z() - (0.5 * kPlaneThickness));
      const auto poseErrors = usd::SetPose(
          gz::math::Pose3d(planePosition, _model.RawPose().Rot()),
          _stage, sdfModelPath);
      if (!poseErrors.empty())
      {
        for (const auto &e : poseErrors)
          errors.push_back(e);
        errors.push_back(UsdError(UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
              "Unable to set the pose of the USD ground plane prim named ["
              + _model.Name() + "]"));
        return errors;
      }
    }
    else
    {
      gz::math::Pose3d pose;
      auto poseErrors = usd::PoseWrtParent(_model, pose);
      if (!poseErrors.empty())
      {
        for (const auto &e : poseErrors)
          errors.push_back(e);
        return errors;
      }

      poseErrors = usd::SetPose(pose, _stage, sdfModelPath);
      if (!poseErrors.empty())
      {
        for (const auto &e : poseErrors)
          errors.push_back(e);
        errors.push_back(UsdError(UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
              "Unable to set the pose of the model prim corresponding to the "
              "SDF model named [" + _model.Name() + "]"));
        return errors;
      }
    }

    // Parse all of the model's links and convert them to USD.
    // Map a link's SDF name to its USD path so that USD joints know which
    // USD links to connect to.
    std::unordered_map<std::string, pxr::SdfPath> sdfLinkToUSDPath;
    for (uint64_t i = 0; i < _model.LinkCount(); ++i)
    {
      const auto link = *(_model.LinkByIndex(i));
      auto linkPath = std::string(_path + "/" + link.Name());
      linkPath = sdf::usd::validPath(linkPath);
      sdfLinkToUSDPath[link.Name()] = pxr::SdfPath(linkPath);
      UsdErrors linkErrors = ParseSdfLink(
        link, _stage, linkPath, !_model.Static());
      if (!linkErrors.empty())
      {
        errors.push_back(
          UsdError(sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
          "Error parsing link [" + link.Name() + "]"));
        errors.insert(errors.end(), linkErrors.begin(), linkErrors.end());
        return errors;
      }
    }

    // Parse all of the model's joints and convert them to USD.
    auto modelPrim = _stage->GetPrimAtPath(sdfModelPath);
    if (!modelPrim)
    {
      errors.push_back(UsdError(
            sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
            "Internal error: unable to find prim at path [" + _path
            + "], but a prim should exist at this path."));
      return errors;
    }
    bool markedArticulationRoot = false;
    for (uint64_t i = 0; i < _model.JointCount(); ++i)
    {
      const auto joint = *(_model.JointByIndex(i));
      const auto jointPath = std::string(_path + "/" + joint.Name());
      const auto jointErrors = ParseSdfJoint(joint, _stage, jointPath, _model,
            sdfLinkToUSDPath, _worldPath);
      if (!jointErrors.empty())
      {
        errors.push_back(UsdError(
              sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
              "Error parsing joint [" + joint.Name() + "]."));
        errors.insert(errors.end(), jointErrors.begin(), jointErrors.end());
        return errors;
      }

      if (!markedArticulationRoot && joint.Type() == sdf::JointType::REVOLUTE)
      {
        if (!pxr::UsdPhysicsArticulationRootAPI::Apply(modelPrim))
        {
          errors.push_back(UsdError(
                sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
                "Unable to mark Xform at path [" + _path +
                "] as a pxr::UsdPhysicsArticulationRootAPI. "
                "Some features might not work."));
        }
        else
        {
          markedArticulationRoot = true;
        }
      }
    }

    if (!markedArticulationRoot && !_model.Static())
    {
      if (!pxr::UsdPhysicsRigidBodyAPI::Apply(modelPrim))
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
              "Internal error: unable to mark model at path [" +
              modelPrim.GetPath().GetString() + "] as a rigid body."));
        return errors;
      }
    }

    return errors;
  }
}
}
}
