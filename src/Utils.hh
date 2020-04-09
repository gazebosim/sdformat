/*
 * Copyright 2017 Open Source Robotics Foundation
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
#ifndef SDFORMAT_UTILS_HH
#define SDFORMAT_UTILS_HH

#include <algorithm>
#include <string>
#include <vector>
#include "sdf/Error.hh"
#include "sdf/Element.hh"
#include "sdf/Types.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief Check if the passed string is a reserved name.
  /// This currently includes "world" and all strings that start
  /// and end with "__".
  /// \param[in] _name Name to check.
  /// \returns True if the name is a reserved name and thus invalid.
  bool isReservedName(const std::string &_name);

  /// \brief Read the "name" attribute from an element.
  /// \param[in] _sdf SDF element pointer which contains the name.
  /// \param[out] _name String to hold the name value.
  /// \return True when the "name" attribute exists.
  bool loadName(sdf::ElementPtr _sdf, std::string &_name);

  /// \brief Read a pose element from and SDF pointer, and return (via
  /// function parameters) the pose value and coordinate frame.
  /// \param[in] _sdf Pointer to an SDF element that is a pose element.
  /// \param[out] _pose Value of the pose element. The default value is
  /// ignition::math::Pose3d::Zero.
  /// \param[out] _frame Value of the frame attribute. The default value is
  /// an empty string.
  /// \return True if the pose element contained an ignition::math::Pose3d
  /// value.
  bool loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
                std::string &_frame);

  /// \brief If the value is negative, convert it to positive infinity.
  /// Otherwise, return the original value.
  /// \param[in] _value The value to convert, if necessary.
  /// \return Infinity if the input value is negative, otherwise the original
  /// value.
  double infiniteIfNegative(double _value);

  /// \brief Load all objects of a specific sdf element type. No error
  /// is returned if an element is not present. This function assumes that
  /// an element has a "name" attribute that must be unique.
  /// \param[in] _sdf The SDF element that contains zero or more elements.
  /// \param[in] _sdfName Name of the sdf element, such as "model".
  /// \param[out] _objs Elements that match _sdfName in _sdf are added to this
  /// vector, unless an error is encountered during load or a duplicate name
  /// exists.
  /// \return The vector of errors. An empty vector indicates no errors were
  /// experienced.
  template<typename Class>
  sdf::Errors loadUniqueRepeated(sdf::ElementPtr _sdf,
      const std::string &_sdfName, std::vector<Class> &_objs)
  {
    Errors errors;

    std::vector<std::string> names;

    // Check that an element exists.
    if (_sdf->HasElement(_sdfName))
    {
      // Read all the elements.
      sdf::ElementPtr elem = _sdf->GetElement(_sdfName);
      while (elem)
      {
        Class obj;

        // Load the model and capture the errors.
        Errors loadErrors = obj.Load(elem);

        // keep processing even if there are loadErrors
        {
          std::string name;

          // Read the name for uniqueness checks. Don't report errors here.
          // Errors are captured in obj.Load(elem) above.
          sdf::loadName(elem, name);

          // Check that the name does not exist.
          if (std::find(names.begin(), names.end(), name) != names.end())
          {
            errors.push_back({ErrorCode::DUPLICATE_NAME,
                _sdfName + " with name[" + name + "] already exists."});
          }
          else
          {
            // Add the object to the result if no errors have been encountered.
            _objs.push_back(std::move(obj));
            names.push_back(name);
          }

          // Add the load errors to the master error list.
          errors.insert(errors.end(), loadErrors.begin(), loadErrors.end());
        }

        elem = elem->GetNextElement(_sdfName);
      }
    }
    // Do not add an error if the model tag is missing. This is an internal
    // function that is called by class without checking if an element actually
    // exists. This is a bit of safe code reduction.

    return errors;
  }

  /// \brief Load all objects of a specific sdf element type. No error
  /// is returned if an element is not present.
  /// \param[in] _sdf The SDF element that contains zero or more elements.
  /// \param[in] _sdfName Name of the sdf element, such as "model".
  /// \param[out] _objs Elements that match _sdfName in _sdf are added to this
  /// vector, unless an error is encountered during load.
  /// \return The vector of errors. An empty vector indicates no errors were
  /// experienced.
  template<typename Class>
  sdf::Errors loadRepeated(sdf::ElementPtr _sdf,
      const std::string &_sdfName, std::vector<Class> &_objs)
  {
    Errors errors;

    // Check that an element exists.
    if (_sdf->HasElement(_sdfName))
    {
      // Read all the elements.
      sdf::ElementPtr elem = _sdf->GetElement(_sdfName);
      while (elem)
      {
        Class obj;

        // Load the model and capture the errors.
        Errors loadErrors = obj.Load(elem);

        {
          // Add the load errors to the master error list.
          errors.insert(errors.end(), loadErrors.begin(), loadErrors.end());

          // but keep object anyway
          _objs.push_back(std::move(obj));
        }

        elem = elem->GetNextElement(_sdfName);
      }
    }
    // Do not add an error if the model tag is missing. This is an internal
    // function that is called by class without checking if an element actually
    // exists. This is a bit of safe code reduction.

    return errors;
  }
  }
}
#endif
