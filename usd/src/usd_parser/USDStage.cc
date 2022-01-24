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
#include <string>
#include <set>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd_parser/USDStage.hh"
#include "sdf/usd_parser/utils.hh"

namespace sdf
{
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
  };

  /////////////////////////////////////////////////
  USDStage::USDStage(const std::string &_refFileName)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
  {
    // Open the stage
    auto referencee = pxr::UsdStage::Open(_refFileName);
    if (!referencee)
    {
      // if the file doesn't exists throw and exception
      throw std::invalid_argument("Stage file does not exists");
    }

    // Get meters per unit
    referencee->GetMetadata<double>(
      pxr::TfToken("metersPerUnit"), &this->dataPtr->metersPerUnit);

    // is it up axis define, if so, read the value, if the value is
    // not 'Y' or 'Z' throw an exception
    if (referencee->HasAuthoredMetadata(pxr::UsdGeomTokens->upAxis))
    {
      pxr::TfToken axis;
      referencee->GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
      this->dataPtr->upAxis = axis.GetText();

      if (this->dataPtr->upAxis != "Y" && this->dataPtr->upAxis != "Z")
      {
        throw std::range_error("Up axis should be 'Y' or 'Z'");
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
}  // sdf
