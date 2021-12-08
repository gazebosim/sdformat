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

#include "sdf_usd_parser/link.hh"

#include <string>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/massAPI.h>

#include "sdf/Link.hh"
#include "sdf_usd_parser/visual.hh"
#include "sdf_usd_parser/utils.hh"

namespace usd
{
  bool ParseSdfLink(const sdf::Link &_link, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const pxr::SdfPath sdfLinkPath(_path);

    auto usdLinkXform = pxr::UsdGeomXform::Define(_stage, sdfLinkPath);
    usd::SetPose(_link.RawPose(), usdLinkXform);

    // apply a mass to this link
    // TODO(adlarkin) don't apply mass to static links? I don't think it
    // matters because static bodies in USD (i.e., bodies that either
    // have or are children of a PhysicsRigidBodyAPI) are implicitly given
    // an infinite mass. So, even if the generated USD file shows a static
    // link having a non-infinite mass (default mass in SDF is 1), the link
    // should still behave as a static body. But, maybe it would be worth
    // skipping applying masses to static links to avoid confusion in the
    // resulting USD file
    auto massAPI =
      pxr::UsdPhysicsMassAPI::Apply(_stage->GetPrimAtPath(sdfLinkPath));
    if (!massAPI)
    {
      std::cerr << "Unable to attach mass properties to link ["
                << _link.Name() << "]\n";
      return false;
    }
    massAPI.CreateMassAttr().Set(
        static_cast<float>(_link.Inertial().MassMatrix().Mass()));

    // TODO(adlarkin) finish parsing link. It will look something like this
    // (this does not cover all elements of a link that need to be parsed):
    //  * ParseSdfVisual
    //  * ParseSdfCollision
    //  * ParseSdfSensor

    // parse all of the link's visuals and convert them to USD
    for (uint64_t i = 0; i < _link.VisualCount(); ++i)
    {
      const auto visual = *(_link.VisualByIndex(i));
      const auto visualPath = std::string(_path + "/" + visual.Name());
      if (!ParseSdfVisual(visual, _stage, visualPath))
      {
        std::cerr << "Error parsing visual [" << visual.Name() << "]\n";
        return false;
      }
    }
    return true;
  }
}
