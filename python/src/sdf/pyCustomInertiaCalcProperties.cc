/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "pyCustomInertiaCalcProperties.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/Element.hh"
#include "sdf/Mesh.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineCustomInertiaCalcProperties(pybind11::object module)
{
  pybind11::class_<sdf::CustomInertiaCalcProperties>(
      module, "CustomInertiaCalcProperties")
    .def(pybind11::init<>())
    .def(pybind11::init<double, sdf::Mesh, sdf::ElementPtr>())
    .def(pybind11::init<sdf::CustomInertiaCalcProperties>())
    .def("density", &sdf::CustomInertiaCalcProperties::Density,
         "Get the density of the mesh.")
    .def("set_density", &sdf::CustomInertiaCalcProperties::SetDensity,
         "Set the density of the mesh.")
    .def("mesh", &sdf::CustomInertiaCalcProperties::Mesh,
         "Get the mesh object being used, or None if it has not been set.")
    .def("set_mesh", &sdf::CustomInertiaCalcProperties::SetMesh,
         "Set the mesh object.")
    .def("auto_inertia_params",
         &sdf::CustomInertiaCalcProperties::AutoInertiaParams,
         "Get the <auto_inertia_params> element. User defined calculator "
         "params can be accessed through this element.")
    .def("set_auto_inertia_params",
         &sdf::CustomInertiaCalcProperties::SetAutoInertiaParams,
         "Set the <auto_inertia_params> element.")
    .def("__copy__", [](const sdf::CustomInertiaCalcProperties &self) {
      return sdf::CustomInertiaCalcProperties(self);
    })
    .def("__deepcopy__", [](const sdf::CustomInertiaCalcProperties &self,
                            pybind11::dict) {
      return sdf::CustomInertiaCalcProperties(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
