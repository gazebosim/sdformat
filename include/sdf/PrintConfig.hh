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
#ifndef SDF_PRINTCONFIG_HH_
#define SDF_PRINTCONFIG_HH_

#include <ignition/utils/ImplPtr.hh>

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
/// \enum PrintConfig
/// \brief The different types of available printing configuration.
enum class PrintConfig
{
  /// \brief Default config
  DEFAULT,

  /// \brief Displays rotations in poses in degrees.
  ROTATION_IN_DEGREES,

  /// \brief Displays rotations in poses in degrees as well as snaps to the 16
  /// commonly used angles lying on the axes in intervals of 45 degrees with
  /// a tolerance of 0.01 degrees, with the exception of singularities.
  ROTATION_SNAP_TO_DEGREES
};
}
}

#endif
