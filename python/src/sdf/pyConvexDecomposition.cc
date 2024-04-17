/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "pyMesh.hh"

#include <pybind11/pybind11.h>

#include "sdf/ParserConfig.hh"
#include "sdf/Mesh.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineConvexDecomposition(pybind11::object module)
{
  pybind11::class_<sdf::ConvexDecomposition>(module, "ConvexDecomposition")
    .def(pybind11::init<>())
    .def("max_convex_hulls", &sdf::ConvexDecomposition::MaxConvexHulls,
         "Get the maximum number of convex hulls that can be generated.")
    .def("set_max_convex_hulls", &sdf::ConvexDecomposition::SetMaxConvexHulls,
         "Set the maximum number of convex hulls that can be generated.")
    .def("__copy__", [](const sdf::ConvexDecomposition &self) {
      return sdf::ConvexDecomposition(self);
    })
    .def("__deepcopy__", [](const sdf::ConvexDecomposition &self, pybind11::dict) {
      return sdf::ConvexDecomposition(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
