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

#include <iostream>
#include <set>
#include <string>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDStage.hh"
#include "sdf/usd/usd_parser/utils.hh"

namespace sdf {
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  /// \brief USDStage private data.
  class USDStage::Implementation
  {
    public:
      /// \brief Up Axis this must be "Z" or "Y".
      std::string upAxis = "Z";

      /// \brief Meter per unit
      double metersPerUnit = 1.0;

      /// \brief All USD paths available in the file
      std::set<std::string> paths;

      std::string refFileName;
  };

  /////////////////////////////////////////////////
  USDStage::USDStage(const std::string &_refFileName)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
  {
    this->dataPtr->refFileName = _refFileName;
  }

  /////////////////////////////////////////////////
  sdf::Errors USDStage::Init()
  {
    sdf::Errors errors;

    // Open the stage
    auto referencee = pxr::UsdStage::Open(this->dataPtr->refFileName);
    if (!referencee)
    {
      errors.emplace_back(
        Error(ErrorCode::FILE_READ, "Failed to load usd file"));
      return errors;
    }

    // Get meters per unit
    this->dataPtr->metersPerUnit =
      pxr::UsdGeomGetStageMetersPerUnit(referencee);

    // is it up axis define, if so, read the value, if the value is
    // not 'Y' or 'Z' throw an exception
    if (referencee->HasAuthoredMetadata(pxr::UsdGeomTokens->upAxis))
    {
      pxr::TfToken axis;
      referencee->GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
      this->dataPtr->upAxis = axis.GetText();

      if (this->dataPtr->upAxis != "Y" && this->dataPtr->upAxis != "Z")
      {
        errors.emplace_back(
          Error(ErrorCode::ELEMENT_INVALID, "Up axis should be 'Y' or 'Z'"));
        return errors;
      }
    }

    // Keep all the USD paths
    auto range = pxr::UsdPrimRange::Stage(referencee);
    for (auto const &_prim : range)
    {
      if (_prim.IsA<pxr::UsdShadeMaterial>())
      {
        continue;
      }
      if (_prim.IsA<pxr::UsdShadeShader>())
      {
        continue;
      }

      this->dataPtr->paths.insert(_prim.GetPath().GetName());
    }
    return errors;
  }

  /////////////////////////////////////////////////
  const std::string &USDStage::GetUpAxis() const
  {
    return this->dataPtr->upAxis;
  }

  /////////////////////////////////////////////////
  double USDStage::GetMetersPerUnit() const
  {
    return this->dataPtr->metersPerUnit;
  }

  /////////////////////////////////////////////////
  const std::set<std::string> &USDStage::GetUSDPaths() const
  {
    return this->dataPtr->paths;
  }
}  // usd
}  // sdf version namespace
}  // sdf
