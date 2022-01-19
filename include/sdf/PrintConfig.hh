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
  inline namespace SDF_VERSION_NAMESPACE {

  /// This class contains configuration options for printing elements.
  class SDFORMAT_VISIBLE PrintConfig
  {
    /// \brief Default constructor. All options are set to false by default.
    public: PrintConfig();

    /// \brief Sets the option for printing pose rotations in degrees if true,
    /// otherwise they will be printed as radians by default.
    /// \param[in] _value Whether to print pose rotations in degrees.
    public: void SetRotationInDegrees(bool _value);

    /// \brief Returns whether or not pose rotations should be printed in
    /// degrees.
    /// \return True if pose rotations are printed in degrees, false otherwise.
    public: bool RotationInDegrees() const;

    /// \brief Sets the option for printing pose rotation in degrees as well as
    /// snapping the rotation to the desired interval, with the provided
    /// tolerance.
    /// \param[in] _interval Degrees interval to snap to, this value must be
    /// larger than 0, and less than or equal to 360.
    /// \param[in] _tolerance Tolerance which snapping occurs, this value must
    /// be larger than 0, less than 360, and less than the provided interval.
    /// \return True, unless any of the provided values are not valid.
    public: bool SetRotationSnapToDegrees(unsigned int _interval,
                                          double _tolerance);

    /// \brief Returns the current degree value that pose rotations will snap to
    /// when printed.
    /// \return The assigned degrees interval value to snap to. If it has not
    /// been assigned, a nullopt will be returned.
    public: std::optional<unsigned int> RotationSnapToDegrees() const;

    /// \brief Returns the tolerance for snapping degree values when printed.
    /// \return The assigned tolerance value which allows snapping to happen. If
    /// it has not been assigned, a nullopt will be returned.
    public: std::optional<double> RotationSnapTolerance() const;

    /// \brief Set print config to preserve <include> tags.
    /// \param[in] _preserve True to preserve <include> tags.
    /// False to expand included model.
    public: void SetPreserveIncludes(bool _preserve);

    /// \brief Check if <include> tags are to be preserved or expanded.
    /// \return True if <include> tags are preserved.
    /// False if they are to be expanded.
    public: bool PreserveIncludes() const;

    /// \brief Return true if both PrintConfig objects contain the same values.
    /// \param[in] _config PrintConfig to compare.
    /// \return True if 'this' == _config.
    public: bool operator==(const PrintConfig &_config) const;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
