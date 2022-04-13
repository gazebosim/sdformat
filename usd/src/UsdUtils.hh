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

#ifndef SDF_USD_SDF_PARSER_UTILS_HH_
#define SDF_USD_SDF_PARSER_UTILS_HH_

#include <string>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

// TODO(adlarkin) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3d.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Util.hh>

#include "sdf/Collision.hh"
#include "sdf/Error.hh"
#include "sdf/Geometry.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Visual.hh"
#include "sdf/system_util.hh"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Return a valid path
    /// Path must not:
    ///   - start with a digit
    ///   - Contain spaces
    ///   - Contain dots
    /// \param[in] _path Path to check
    /// \return A valid path
    inline std::string validPath(const std::string &_path)
    {
      std::string result;
      if (_path.empty())
      {
        return result;
      }
      result = ignition::common::replaceAll(_path, " ", "");
      result = ignition::common::replaceAll(result, ".", "_");
      if (std::isdigit(result[0]))
      {
        result = "_" + result;
      }
      return result;
    }


    /// \brief Get an object's pose w.r.t. its parent.
    /// \param[in] _obj The object whose pose should be computed/retrieved.
    /// \param[out] _pose The pose of _obj w.r.t. its parent.
    /// \tparam T An object that has the following method signatures:
    ///   sdf::SemanticPose SemanticPose();
    /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
    /// includes an error code and message. An empty vector indicates no error.
    template <typename T>
    inline UsdErrors PoseWrtParent(const T &_obj, ignition::math::Pose3d &_pose)
    {
      UsdErrors errors;
      const auto poseResolutionErrors = _obj.SemanticPose().Resolve(_pose, "");
      if (!poseResolutionErrors.empty())
      {
        for (const auto &e : poseResolutionErrors)
          errors.push_back(UsdError(e));

        sdf::Error poseError(sdf::ErrorCode::POSE_RELATIVE_TO_INVALID,
            "Unable to resolve the pose of [" + _obj.Name()
            + "] w.r.t its parent");
        errors.push_back(UsdError(poseError));
      }
      return errors;
    }

    /// \brief Set the pose of a USD prim.
    /// \param[in] _pose The pose to set.
    /// \param[in] _stage The stage that contains the USD prim at path _usdPath.
    /// \param[in] _usdPath The path to the USD prim that should have its
    /// pose modified to match _pose.
    /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
    /// includes an error code and message. An empty vector indicates no error.
    inline UsdErrors SetPose(const ignition::math::Pose3d &_pose,
        pxr::UsdStageRefPtr &_stage,
        const pxr::SdfPath &_usdPath)
    {
      UsdErrors errors;

      const auto prim = _stage->GetPrimAtPath(_usdPath);
      if (!prim)
      {
        errors.push_back(UsdError(UsdErrorCode::INVALID_PRIM_PATH,
              "No USD prim exists at path [" + _usdPath.GetString() + "]"));
        return errors;
      }

      pxr::UsdGeomXformCommonAPI geomXformAPI(_stage->GetPrimAtPath(_usdPath));
      if (!geomXformAPI)
      {
        errors.push_back(UsdError(UsdErrorCode::FAILED_PRIM_API_APPLY,
              "Unable to apply apxr::UsdGeomXformCommonAPI to prim at path ["
              + _usdPath.GetString() + "]"));
        return errors;
      }

      const auto &position = _pose.Pos();
      geomXformAPI.SetTranslate(pxr::GfVec3d(
            position.X(), position.Y(), position.Z()));

      const auto &rotation = _pose.Rot();
      // roll/pitch/yaw from ignition::math::Pose3d are in radians, but this API
      // call expects degrees
      geomXformAPI.SetRotate(pxr::GfVec3f(
            ignition::math::Angle(rotation.Roll()).Degree(),
            ignition::math::Angle(rotation.Pitch()).Degree(),
            ignition::math::Angle(rotation.Yaw()).Degree()));

      return errors;
    }

    /// \brief Check if an sdf model is static and contains a single link that
    /// contains a single visual and single collision that both have a plane
    /// geometry.
    /// \param[in] _model The sdf model to check
    /// \return True if _model is static and has only one link with one visual
    /// and one collision that have a plane geometry. False otherwise
    /// \note This method will no longer be needed when a pxr::USDGeomPlane
    /// class is created
    inline bool IsPlane(const sdf::Model &_model)
    {
      if (!_model.Static() || _model.LinkCount() != 1u)
        return false;

      const auto &link = _model.LinkByIndex(0u);
      if ((link->VisualCount() != 1u) || (link->CollisionCount() != 1u))
        return false;

      const auto &visual = link->VisualByIndex(0u);
      if (visual->Geom()->Type() != sdf::GeometryType::PLANE)
        return false;

      const auto &collision = link->CollisionByIndex(0u);
      return collision->Geom()->Type() == sdf::GeometryType::PLANE;
    }

    /// \brief Pre-defined USD plane thickness. This is a temporary variable
    /// that will no longer be needed once USD supports their own plane class
    static const double kPlaneThickness = 0.25;
  }
  }
}

#endif
