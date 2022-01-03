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

#include "sdf_usd_parser/model.hh"

#include <iostream>
#include <string>
#include <unordered_map>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>

#include "sdf/Model.hh"
#include "sdf_usd_parser/joint.hh"
#include "sdf_usd_parser/link.hh"
#include "sdf_usd_parser/utils.hh"

namespace usd
{
  bool ParseSdfModel(const sdf::Model &_model, pxr::UsdStageRefPtr &_stage,
      const std::string &_path, const pxr::SdfPath &_worldPath)
  {
    if (_model.ModelCount())
    {
      std::cerr << "Nested models currently aren't supported\n";
      return false;
    }

    const pxr::SdfPath sdfModelPath(_path);
    auto usdModelXform = pxr::UsdGeomXform::Define(_stage, sdfModelPath);
    // since USD does not have a plane yet, planes are being represented as a
    // wide, thin box. The plane/box pose needs to be offset according to the
    // thickness to ensure that the top of the plane is at the correct height.
    // This pose offset workaround will no longer be needed when a pxr::USDGeomPlane
    // class is created (see the notes in the usd::ParseSdfPlaneGeometry method in
    // the geometry.cc file for more information)
    if (usd::IsPlane(_model))
    {
      // TODO(adlarkin) does this computation need to be updated if the plane isn't
      // flat (i.e., if _model.RawPose().Rot() - which is used in the SetPose call
      // below - isn't an identity quaternion)?
      ignition::math::Vector3d planePosition(
          _model.RawPose().X(),
          _model.RawPose().Y(),
          _model.RawPose().Z() - (0.5 * usd::kPlaneThickness));
      usd::SetPose(ignition::math::Pose3d(planePosition, _model.RawPose().Rot()),
          _stage, sdfModelPath);
    }
    else
    {
      usd::SetPose(usd::PoseWrtParent(_model), _stage, sdfModelPath);
    }

    // Parse all of the model's links and convert them to USD.
    // Map a link's SDF name to its USD path so that USD joints know which
    // USD links to connect to.
    std::unordered_map<std::string, pxr::SdfPath> sdfLinkToUSDPath;
    for (uint64_t i = 0; i < _model.LinkCount(); ++i)
    {
      const auto link = *(_model.LinkByIndex(i));
      const auto linkPath = std::string(_path + "/" + link.Name());
      sdfLinkToUSDPath[link.Name()] = pxr::SdfPath(linkPath);
      if (!ParseSdfLink(link, _stage, linkPath, !_model.Static()))
      {
        std::cerr << "Error parsing link [" << link.Name() << "]\n";
        return false;
      }
    }

    // Parse all of the model's joints and convert them to USD.
    for (uint64_t i = 0; i < _model.JointCount(); ++i)
    {
      const auto joint = *(_model.JointByIndex(i));
      const auto jointPath = std::string(_path + "/" + joint.Name());
      if (!ParseSdfJoint(joint, _stage, jointPath, _model,
            sdfLinkToUSDPath, _worldPath))
      {
        std::cerr << "Error parsing joint [" << joint.Name() << "]\n";
        return false;
      }
    }

    // TODO(adlarkin) finish parsing model

    return true;
  }
}
