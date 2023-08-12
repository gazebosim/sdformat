/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
 */

#ifndef SDFORMAT_PYTHON_PARAM_HH_
#define SDFORMAT_PYTHON_PARAM_HH_

#include <pybind11/pybind11.h>

#include "sdf/Param.hh"

#include "sdf/config.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
// Define a pybind11 wrapper for an sdf::Param
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineParam(pybind11::object module);

template <typename T>
std::string computeSuffix()
{
  // TypeToString returns a value with a space, which would not be a valid 
  // python function name, so we override that here.
  if constexpr (std::is_same_v<T, unsigned int>)
    return "unsigned_int";
  return ParamPrivate::TypeToString<T>();
}

}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif  // SDFORMAT_PYTHON_SCENE_HH_
