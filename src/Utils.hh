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

#include <string>
#include <vector>
#include "sdf/Element.hh"
#include "sdf/Model.hh"
#include "sdf/Types.hh"

namespace sdf
{
  /// \brief Read the "name" attribute from an element.
  /// \param[in] _sdf SDF element pointer which contains the name.
  /// \param[out] _name String to hold the name value.
  /// \return True when the "name" attribute exists.
  bool loadName(sdf::ElementPtr _sdf, std::string &_name);

  /// \brief Load all the models in an sdf element. No error is returned if
  /// a model element is not present.
  /// \param[in] _sdf The SDF element that contains one or more models.
  /// \param[in,out] _models Models in _sdf are added to this vector, unless an
  /// error is encountered during model load or a duplicate model name
  /// exists.
  /// \return The vector of errors. An empty vector indicates no errors were
  /// experienced.
  Errors loadModels(const sdf::ElementPtr _sdf,
                    std::vector<Model> &_models);
}
#endif
