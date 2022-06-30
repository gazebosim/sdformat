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

#include "pyPhysics.hh"

#include <pybind11/pybind11.h>

#include "sdf/Physics.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void definePhysics(pybind11::object module)
{
  pybind11::class_<sdf::Physics>(module, "Physics")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Physics>())
    .def("name", &sdf::Physics::Name,
         "Get the name of this set of physics parameters.")
    .def("set_name", &sdf::Physics::SetName,
         "Set the name of this set of physics parameters.")
    .def("is_default", &sdf::Physics::IsDefault,
         "Get whether this physics profile is marked as default. "
         "If true, this physics profile is set as the default physics profile "
         "for the World. If multiple default physics elements exist, the first "
         "physics profile marked as default is chosen. If no default physics "
         "element exists, the first physics element is chosen.")
    .def("set_default", &sdf::Physics::SetDefault,
         "Set whether this physics profile is the default.")
    .def("engine_type", &sdf::Physics::EngineType,
         "Get the physics profile dynamics engine type. "
         "Current options are ode, bullet, simbody and dart. Defaults to ode "
         "if left unspecified.")
    .def("set_engine_type", &sdf::Physics::SetEngineType,
         "Set the physics profile dynamics engine type.")
    .def("max_step_size", &sdf::Physics::MaxStepSize,
         "Get the max step size in seconds.")
    .def("set_max_step_size", &sdf::Physics::SetMaxStepSize,
         "Set the max step size in seconds.")
    .def("real_time_factor", &sdf::Physics::RealTimeFactor,
         "Get the target real time factor.")
    .def("set_real_time_factor", &sdf::Physics::SetRealTimeFactor,
         "Set the target realtime factor.")
    .def("max_contacts", &sdf::Physics::MaxContacts,
         "Get the maximum number of contacts allowed between two "
         "entities. This value can be overridden by a max_contacts element in "
         "a collision element.")
    .def("set_max_contacts", &sdf::Physics::SetMaxContacts,
         "Set the maximum number of contacts allowed between two "
         "entities. This value can be overridden by a max_contacts element in "
         "collision element.")
    .def("__copy__", [](const sdf::Physics &self) {
      return sdf::Physics(self);
    })
    .def("__deepcopy__", [](const sdf::Physics &self, pybind11::dict) {
      return sdf::Physics(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
