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
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#include "sdf/Link.hh"
#include "sdf_usd_parser/sensor.hh"
#include "sdf_usd_parser/utils.hh"
#include "sdf_usd_parser/visual.hh"

namespace usd
{
  bool ParseSdfLink(const sdf::Link &_link, pxr::UsdStageRefPtr &_stage,
      const std::string &_path, const bool _rigidBody)
  {
    const pxr::SdfPath sdfLinkPath(_path);

    auto usdLinkXform = pxr::UsdGeomXform::Define(_stage, sdfLinkPath);
    usd::SetPose(_link.RawPose(), _stage, sdfLinkPath);

    if (_rigidBody)
    {
      auto linkPrim = _stage->GetPrimAtPath(sdfLinkPath);
      if (!linkPrim)
      {
        std::cerr << "Internal error: unable to get prim at path ["
                  << _path << "], but a link prim should exist at this path\n";
        return false;
      }

      if (!pxr::UsdPhysicsRigidBodyAPI::Apply(linkPrim))
      {
        std::cerr << "Internal error: unable to mark link at path ["
                  << _path << "] as a rigid body\n";
        return false;
      }

      auto massAPI =
        pxr::UsdPhysicsMassAPI::Apply(linkPrim);
      if (!massAPI)
      {
        std::cerr << "Unable to attach mass properties to link ["
                  << _link.Name() << "]\n";
        return false;
      }
      massAPI.CreateMassAttr().Set(
          static_cast<float>(_link.Inertial().MassMatrix().Mass()));
    }

    // TODO(adlarkin) finish parsing link. It will look something like this
    // (this does not cover all elements of a link that need to be parsed):
    //  * ParseSdfVisual
    //  * ParseSdfCollision
    //  * ParseSdfSensor
    //  * ParseSdfLight (look at world.cc for how this is being done)

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

    // convert the link's sensors
    for (uint64_t i = 0; i < _link.SensorCount(); ++i)
    {
      const auto sensor = *(_link.SensorByIndex(i));
      const auto sensorPath = std::string(_path + "/" + sensor.Name());
      if (!ParseSdfSensor(sensor, _stage, sensorPath))
      {
        std::cerr << "Error parsing sensor [" << sensor.Name() << "]\n";
        return false;
      }
    }

    return true;
  }
}
