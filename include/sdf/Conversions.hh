/*
 * Copyright 2021 Open Source Robotics Foundation
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


#ifndef SDF_CONVERSIONS_HH_
#define SDF_CONVERSIONS_HH_

#include <memory>

#include <ignition/common/Material.hh>
#include <sdf/Material.hh>
#include "sdf/sdf_config.h"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  /// \brief Specialized conversion from an SDF material to a Ignition Common
  /// material.
  /// \param[in] _in Ignition Common Material.
  /// \return Geometry material.
  SDFORMAT_VISIBLE
  sdf::Material convert(std::shared_ptr<ignition::common::Material> &_in);

  /// \brief Specialized conversion from an Ignition Common Material
  /// to a SDF material
  /// \param[in] _in SDF material.
  /// \return Ignition Common Material.
  SDFORMAT_VISIBLE
  std::shared_ptr<ignition::common::Material> convert(sdf::Material &_in);
  }
}

#endif
