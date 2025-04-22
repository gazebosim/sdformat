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
    .def("bullet_friction", &sdf::Friction::BulletFriction,
         pybind11::return_value_policy::reference_internal,
         "Get the bullet friction object.")
    .def("set_bullet_friction", &sdf::Friction::SetBulletFriction,
         "Set the bullet friction object.")
    .def("torsional", &sdf::Friction::Torsional,
         pybind11::return_value_policy::reference_internal,
         "Get the torsional friction object.")
    .def("set_torsional", &sdf::Friction::SetTorsional,
         "Set the torsional friction object.")
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
/////////////////////////////////////////////////
void defineBulletFriction(pybind11::object module)
{
  pybind11::class_<sdf::BulletFriction>(module, "BulletFriction")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::BulletFriction>())
    .def("friction", &sdf::BulletFriction::Friction,
         "Get the friction parameter.")
    .def("set_friction", &sdf::BulletFriction::SetFriction,
         "Set the friction parameter.")
    .def("friction2", &sdf::BulletFriction::Friction2,
         "Get the friction parameter.")
    .def("set_friction2", &sdf::BulletFriction::SetFriction2,
         "Set the friction2 parameter.")
    .def("fdir1", &sdf::BulletFriction::Fdir1,
         "Get the fdir1 parameter.")
    .def("set_fdir1", &sdf::BulletFriction::SetFdir1,
         "Set the fdir1 parameter.")
    .def("rolling_friction", &sdf::BulletFriction::RollingFriction,
         "Get the rolling friction parameter.")
    .def("set_rolling_friction", &sdf::BulletFriction::SetRollingFriction,
         "Set the rolling friction parameter.")
    .def("__copy__", [](const sdf::BulletFriction &self) {
      return sdf::BulletFriction(self);
    })
    .def("__deepcopy__", [](const sdf::BulletFriction &self, pybind11::dict) {
      return sdf::BulletFriction(self);
    }, "memo"_a);
}
/////////////////////////////////////////////////
void defineTorsional(pybind11::object module)
{
  pybind11::class_<sdf::Torsional>(module, "Torsional")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Torsional>())
    .def("coefficient", &sdf::Torsional::Coefficient,
         "Get the coefficient parameter.")
    .def("set_coefficient", &sdf::Torsional::SetCoefficient,
         "Set the coefficient parameter.")
    .def("use_patch_radius", &sdf::Torsional::UsePatchRadius,
         "Get whether to use patch radius for torsional friction calculation.")
    .def("set_use_patch_radius", &sdf::Torsional::SetUsePatchRadius,
         "Set whether to use patch radius for torsional friction calculation.")
    .def("patch_radius", &sdf::Torsional::PatchRadius,
         "Get the radius of contact patch surface.")
    .def("set_patch_radius", &sdf::Torsional::SetPatchRadius,
         "Set the radius of contact patch surface.")
    .def("surface_radius", &sdf::Torsional::SurfaceRadius,
         "Get the surface radius on the contact point.")
    .def("set_surface_radius", &sdf::Torsional::SetSurfaceRadius,
         "Set the surface radius on the contact point.")
    .def("ode_slip", &sdf::Torsional::ODESlip,
         "Get the ODE force dependent slip for torsional friction.")
    .def("set_ode_slip", &sdf::Torsional::SetODESlip,
         "Set the ODE force dependent slip for torsional friction.")
    .def("__copy__", [](const sdf::Torsional &self) {
      return sdf::Torsional(self);
    })
    .def("__deepcopy__", [](const sdf::Torsional &self, pybind11::dict) {
      return sdf::Torsional(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
