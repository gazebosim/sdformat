/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include "pySurface.hh"

#include <pybind11/pybind11.h>

#include "sdf/Surface.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineContact(pybind11::object module)
{
  pybind11::class_<sdf::Contact>(module, "Contact")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Contact>())
    .def("collide_bitmask", &sdf::Contact::CollideBitmask,
         "Get the collide bitmask parameter.")
    .def("set_collide_bitmask", &sdf::Contact::SetCollideBitmask,
         "Set the collide bitmask parameter.")
    .def("__copy__", [](const sdf::Contact &self) {
      return sdf::Contact(self);
    })
    .def("__deepcopy__", [](const sdf::Contact &self, pybind11::dict) {
      return sdf::Contact(self);
    }, "memo"_a);
}
/////////////////////////////////////////////////
void defineSurface(pybind11::object module)
{
  pybind11::class_<sdf::Surface>(module, "Surface")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Surface>())
    .def("contact", &sdf::Surface::Contact,
         pybind11::return_value_policy::reference,
         "Get the associated contact object")
    .def("set_contact", &sdf::Surface::SetContact,
         "Set the associated contact object.")
    .def("__copy__", [](const sdf::Surface &self) {
      return sdf::Surface(self);
    })
    .def("__deepcopy__", [](const sdf::Surface &self, pybind11::dict) {
      return sdf::Surface(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
