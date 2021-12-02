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

#ifndef SDF_PARSER_UTILS_HH_
#define SDF_PARSER_UTILS_HH_

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include "sdf/Geometry.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Visual.hh"
#include "sdf/sdf_config.h"

namespace usd
{
  /// \brief Set the pose of a pxr::UsdGeomXform object.
  /// \param[in] _pose The pose to set.
  /// \param[in] _geomXform The object that should have a pose of _pose.
  // TODO(adlarkin) handle a <pose> element's <relative_to> attribute.
  // Either this API will need to change to take the <relative_to> data, or
  // the code that calls this method elsewhere will need to update _pose to be
  // relative to the <relative_to> attribute data before calling this method
  // (probably best to update the API here to take the <relative_to> data.
  // If it's empty, then _pose doesn't need to be modified)
  inline void SDFORMAT_VISIBLE SetPose(const ignition::math::Pose3d &_pose,
      pxr::UsdGeomXform &_geomXform)
  {
    pxr::UsdGeomXformCommonAPI geomXformAPI(_geomXform);

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

  /// \brief Thickness of an sdf plane
  /// \note This will no longer be needed when a pxr::USDGeomPlane class
  /// is created (see the notes in the usd::ParseSdfPlaneGeometry method in
  /// the geometry.cc file for more information)
  static const double kPlaneThickness = 0.25;

  /// \brief Check if an sdf model is a plane.
  /// \param[in] _model The sdf model to check
  /// \return True if _model is a plane. False otherwise
  /// \note This method will no longer be needed when a pxr::USDGeomPlane class
  /// is created (see the notes in the usd::ParseSdfPlaneGeometry method in
  /// the geometry.cc file for more information)
  inline bool SDFORMAT_VISIBLE IsPlane(const sdf::Model &_model)
  {
    if (_model.LinkCount() != 1u)
      return false;

    const auto &link = _model.LinkByIndex(0u);
    if (link->VisualCount() != 1u)
      return false;

    const auto &visual = link->VisualByIndex(0u);
    return visual->Geom()->Type() == sdf::GeometryType::PLANE;
  }
}

#endif
