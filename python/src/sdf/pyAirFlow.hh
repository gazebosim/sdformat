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

#ifndef SDFORMAT_PYTHON_AIRFLOW_HH_
#define SDFORMAT_PYTHON_AIRFLOW_HH_

#include <pybind11/pybind11.h>

#include "sdf/AirFlow.hh"

#include "sdf/config.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/// Define a pybind11 wrapper for an sdf::AirFlow
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineAirFlow(pybind11::object module);
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif  // SDFORMAT_PYTHON_AirFlow_HH_
