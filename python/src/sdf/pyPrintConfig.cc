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

#include "pyPrintConfig.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>

#include "sdf/PrintConfig.hh"
#include "pybind11_helpers.hh"

#include "sdf/config.hh"

using namespace pybind11::literals;
namespace py = pybind11;
namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{

void definePrintConfig(py::object module)
{
  py::class_<sdf::PrintConfig>(module, "PrintConfig").def(py::init<>());
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
