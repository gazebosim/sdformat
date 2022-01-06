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

#include "sdf_usd_parser/visual.hh"

#include <iostream>
#include <string>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "sdf/Visual.hh"
#include "sdf_usd_parser/geometry.hh"
#include "sdf_usd_parser/material.hh"
#include "sdf_usd_parser/utils.hh"

namespace usd
{
  bool ParseSdfVisual(const sdf::Visual &_visual, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    const pxr::SdfPath sdfVisualPath(_path);
    auto usdVisualXform = pxr::UsdGeomXform::Define(_stage, sdfVisualPath);
    usd::SetPose(usd::PoseWrtParent(_visual), _stage, sdfVisualPath);

    const auto geometry = *(_visual.Geom());
    const auto geometryPath = std::string(_path + "/geometry");
    if (!ParseSdfGeometry(geometry, _stage, geometryPath))
    {
      std::cerr << "Error parsing geometry attached to visual ["
                << _visual.Name() << "]\n";
      return false;
    }

    auto geomPrim = _stage->GetPrimAtPath(pxr::SdfPath(geometryPath));
    if (geomPrim)
    {
        const auto material = _visual.Material();
        pxr::UsdShadeMaterial materialUSD;

        if (material)
        {
          materialUSD = usd::ParseSdfMaterial(material, _stage);
          if (!materialUSD)
          {
            std::cerr << "Error parsing material attached to visual ["
                      << _visual.Name() << "]\n";
            return false;
          }
          pxr::UsdShadeMaterialBindingAPI(geomPrim).Bind(materialUSD);
        }
    }
    else
    {
      std::cerr << "Internal error: no geometry prim exists at path ["
                << geometryPath << "]\n";
      return false;
    }

    return true;
  }
}
