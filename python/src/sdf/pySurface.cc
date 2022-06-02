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
void defineFriction(pybind11::object module)
{
  pybind11::class_<sdf::Friction>(module, "Friction")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Friction>())
    .def("ode", &sdf::Friction::ODE,
         pybind11::return_value_policy::reference_internal,
         "Get the ODE object.")
    .def("set_ode", &sdf::Friction::SetODE,
         "Set the ODE object.")
    .def("__copy__", [](const sdf::Friction &self) {
      return sdf::Friction(self);
    })
    .def("__deepcopy__", [](const sdf::Friction &self, pybind11::dict) {
      return sdf::Friction(self);
    }, "memo"_a);
}
/////////////////////////////////////////////////
void defineODE(pybind11::object module)
{
  pybind11::class_<sdf::ODE>(module, "ODE")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::ODE>())
    .def("fdir1", &sdf::ODE::Fdir1,
         "Get the fdir1 parameter.")
    .def("set_fdir1", &sdf::ODE::SetFdir1,
         "Set the fdir1 parameter.")
    .def("mu", &sdf::ODE::Mu,
         "Get the mu parameter.")
    .def("set_mu", &sdf::ODE::SetMu,
         "Set the mu parameter.")
    .def("mu2", &sdf::ODE::Mu2,
         "Get the mu2 parameter.")
    .def("set_mu2", &sdf::ODE::SetMu2,
         "Set the mu2 parameter.")
    .def("slip1", &sdf::ODE::Slip1,
         "Get the slip1 parameter.")
    .def("set_slip1", &sdf::ODE::SetSlip1,
         "Set the slip1 parameter.")
    .def("slip2", &sdf::ODE::Slip2,
         "Get the slip2 parameter.")
    .def("set_slip2", &sdf::ODE::SetSlip2,
         "Set the slip2 parameter.")
    .def("__copy__", [](const sdf::ODE &self) {
      return sdf::ODE(self);
    })
    .def("__deepcopy__", [](const sdf::ODE &self, pybind11::dict) {
      return sdf::ODE(self);
    }, "memo"_a);
}
/////////////////////////////////////////////////
void defineSurface(pybind11::object module)
{
  pybind11::class_<sdf::Surface>(module, "Surface")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Surface>())
    .def("contact", &sdf::Surface::Contact,
         pybind11::return_value_policy::reference_internal,
         "Get the associated contact object")
    .def("set_contact", &sdf::Surface::SetContact,
         "Set the associated contact object.")
    .def("friction", &sdf::Surface::Friction,
         pybind11::return_value_policy::reference_internal,
         "Get the associated friction object")
    .def("set_friction", &sdf::Surface::SetFriction,
         "Set the associated friction object.")
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
