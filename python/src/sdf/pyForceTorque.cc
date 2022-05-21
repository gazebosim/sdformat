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

#include "pyForceTorque.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/ForceTorque.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineForceTorque(pybind11::object module)
{
  pybind11::class_<sdf::ForceTorque> forceTorqueModule(module, "ForceTorque");
  forceTorqueModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::ForceTorque>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("force_x_noise", &sdf::ForceTorque::ForceXNoise,
         "Get the force noise values in the measurement frame X-axis.")
    .def("set_force_x_noise", &sdf::ForceTorque::SetForceXNoise,
         "Set the force noise values in the measurement frame X-axis.")
    .def("force_y_noise", &sdf::ForceTorque::ForceYNoise,
         "Get the force noise values in the measurement frame Y-axis.")
    .def("set_force_y_noise", &sdf::ForceTorque::SetForceYNoise,
         "Set the force noise values in the measurement frame Y-axis.")
    .def("force_z_noise", &sdf::ForceTorque::ForceZNoise,
         "Get the force noise values in the measurement frame Z-axis.")
    .def("set_force_z_noise", &sdf::ForceTorque::SetForceZNoise,
         "Set the force noise values in the measurement frame Z-axis.")
    .def("torque_x_noise", &sdf::ForceTorque::TorqueXNoise,
         "Get the torque noise values in the measurement frame X-axis.")
    .def("set_torque_x_noise", &sdf::ForceTorque::SetTorqueXNoise,
         "Set the torque noise values in the measurement frame X-axis.")
    .def("torque_y_noise", &sdf::ForceTorque::TorqueYNoise,
         "Get the torque noise values in the measurement frame Y-axis.")
    .def("set_torque_y_noise", &sdf::ForceTorque::SetTorqueYNoise,
         "Set the torque noise values in the measurement frame Y-axis.")
    .def("torque_z_noise", &sdf::ForceTorque::TorqueZNoise,
         "Get the torque noise values in the measurement frame Z-axis.")
    .def("set_torque_z_noise", &sdf::ForceTorque::SetTorqueZNoise,
         "Set the torque noise values in the measurement frame Z-axis.")
    .def("frame", &sdf::ForceTorque::Frame,
         "Get the frame in which the wrench values are reported.")
    .def("set_frame", &sdf::ForceTorque::SetFrame,
         "Set the frame in which the wrench values are reported.")
    .def("measure_direction", &sdf::ForceTorque::MeasureDirection,
         "Get the measure direction of the wrench values.")
    .def("set_measure_direction", &sdf::ForceTorque::SetMeasureDirection,
         "Set the measure direction of the wrench values.")
    .def("__copy__", [](const sdf::ForceTorque &self) {
      return sdf::ForceTorque(self);
    })
    .def("__deepcopy__", [](const sdf::ForceTorque &self, pybind11::dict) {
      return sdf::ForceTorque(self);
    }, "memo"_a);

    pybind11::enum_<sdf::ForceTorqueFrame>(
      forceTorqueModule, "ForceTorqueFrame")
        .value("INVALID", sdf::ForceTorqueFrame::INVALID)
        .value("PARENT", sdf::ForceTorqueFrame::PARENT)
        .value("CHILD", sdf::ForceTorqueFrame::CHILD)
        .value("SENSOR", sdf::ForceTorqueFrame::SENSOR);

    pybind11::enum_<sdf::ForceTorqueMeasureDirection>(
      forceTorqueModule, "ForceTorqueMeasureDirection")
        .value("INVALID", sdf::ForceTorqueMeasureDirection::INVALID)
        .value("PARENT_TO_CHILD", sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD)
        .value("CHILD_TO_PARENT", sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
