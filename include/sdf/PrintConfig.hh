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

#include <optional>
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
  /// snapping the rotation to the desired interval, with the provided
  /// tolerance.
  /// \param[in] _interval Degrees interval to snap to.
  /// \param[in] _tolerance Tolerance which snapping occurs.
  /// \return True, unless the interval is 0 or larger than 360, or the
  /// tolerance is less than 0 or larger than 360.
  public: bool SetRotationSnapToDegrees(unsigned int _interval,
                                        double _tolerance);

  /// \brief Gets the current degree value that pose rotations will snap to when
  /// printed.
  /// \return The assigned degrees interval value to snap to. If it has not been
  /// assigned, a nullopt will be returned.
  public: std::optional<unsigned int> GetRotationSnapToDegrees() const;

  /// \brief Gets the tolerance for snapping degree values when printed.
  /// \return The assigned tolerance value which allows snapping to happen. If
  /// it has not been assigned, a nullopt will be returned.
  public: std::optional<double> GetRotationSnapTolerance() const;

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};

}
}

#endif
