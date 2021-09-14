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

/// This class contains configuration options for printing elements.
class SDFORMAT_VISIBLE PrintConfig
{
  /// \brief Default constructor. All options are set to false by default.
  public: PrintConfig();

  /// \brief Sets the option for printing pose rotations in degrees if true,
  /// otherwise they will be printed as radians by default.
  /// \param[in] _value Whether to print pose rotations in degrees.
  public: void SetRotationInDegrees(bool _value);

  /// \brief Gets the current option of whether pose rotations are printed in
  /// degrees.
  /// \return True if pose rotations are printed in degrees, false otherwise.
  public: bool GetRotationInDegrees() const;

  /// \brief Sets the option for printing pose rotation in degrees as well as
  /// snapping the rotation to commonly used angles of 45 degrees along axes
  /// with a tolerance of 0.01 degrees, with the exception of singularities,
  /// if set to true, otherwise they will be printed as radians by default.
  public: void SetRotationSnapToDegrees(bool _value);

  /// \brief Gets the current option of whether pose rotations are printed in
  /// degrees as well as snapping them to commonly used angles.
  /// \return True if pose rotations are to be printed in degrees, as well as
  /// snapping to commonly used angles, false otherwise.
  public: bool GetRotationSnapToDegrees() const;

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};

}
}

#endif
