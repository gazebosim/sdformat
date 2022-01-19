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

#ifndef SDF_USD_UTILS_HH_
#define SDF_USD_UTILS_HH_

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <pxr/base/gf/vec3d.h>
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include "sdf/SemanticPose.hh"
#include "sdf/config.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Get an object's pose w.r.t. its parent.
    /// \param[in] _obj The object whose pose should be computed/retrieved.
    /// \tparam T An object that has the following method signatures:
    ///   sdf::SemanticPose SemanticPose();
    /// \return _obj's pose w.r.t. its parent. If there was an error computing
    /// this pose, the pose's position will be NaNs.
    template <typename T>
    inline ignition::math::Pose3d SDFORMAT_VISIBLE PoseWrtParent(const T &_obj)
    {
      ignition::math::Pose3d pose(ignition::math::Vector3d::NaN,
          ignition::math::Quaterniond::Identity);
      auto errors = _obj.SemanticPose().Resolve(pose, "");
      if (!errors.empty())
      {
        std::cerr << "Errors occurred when resolving the pose of ["
                  << _obj.Name() << "] w.r.t its parent:\n\t" << errors;
      }
      return pose;
    }

    /// \brief Set the pose of a USD prim.
    /// \param[in] _pose The pose to set.
    /// \param[in] _stage The stage that contains the USD prim at path _usdPath.
    /// \param[in] _usdPath The path to the USD prim that should have its
    /// pose modified to match _pose.
    inline void SDFORMAT_VISIBLE SetPose(const ignition::math::Pose3d &_pose,
        pxr::UsdStageRefPtr &_stage, const pxr::SdfPath &_usdPath)
    {
      pxr::UsdGeomXformCommonAPI geomXformAPI(_stage->GetPrimAtPath(_usdPath));

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
    }
  }
  }
}

#endif
