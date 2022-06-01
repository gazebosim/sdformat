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

#include "pyIMU.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Imu.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineIMU(pybind11::object module)
{
  pybind11::class_<sdf::Imu>(module, "IMU")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Imu>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("linear_acceleration_x_noise", &sdf::Imu::LinearAccelerationXNoise,
         "Get the noise values related to the body-frame linear "
         "acceleration on the X-axis.")
    .def("set_linear_acceleration_x_noise", &sdf::Imu::SetLinearAccelerationXNoise,
         "Set the noise values related to the body-frame linear "
         "acceleration on the X-axis.")
    .def("linear_acceleration_y_noise", &sdf::Imu::LinearAccelerationYNoise,
         "Get the noise values related to the body-frame linear "
         "acceleration on the Y-axis.")
    .def("set_linear_acceleration_y_noise", &sdf::Imu::SetLinearAccelerationYNoise,
         "Set the noise values related to the body-frame linear "
         "acceleration on the Y-axis.")
    .def("linear_acceleration_z_noise", &sdf::Imu::LinearAccelerationZNoise,
         "Get the noise values related to the body-frame linear "
         "acceleration on the Z-axis.")
    .def("set_linear_acceleration_z_noise", &sdf::Imu::SetLinearAccelerationZNoise,
         "Set the noise values related to the body-frame linear "
         "acceleration on the Z-axis.")
    .def("angular_velocity_x_noise", &sdf::Imu::AngularVelocityXNoise,
         "Get the noise values related to the body-frame angular "
         "acceleration on the X-axis.")
    .def("set_angular_velocity_x_noise", &sdf::Imu::SetAngularVelocityXNoise,
         "Set the noise values related to the body-frame angular "
         "acceleration on the X-axis.")
    .def("angular_velocity_y_noise", &sdf::Imu::AngularVelocityYNoise,
         "Get the noise values related to the body-frame angular "
         "acceleration on the Y-axis.")
    .def("set_angular_velocity_y_noise", &sdf::Imu::SetAngularVelocityYNoise,
         "Set the noise values related to the body-frame angular "
         "acceleration on the Y-axis.")
    .def("angular_velocity_z_noise", &sdf::Imu::AngularVelocityZNoise,
         "Get the noise values related to the body-frame angular "
         "acceleration on the Z-axis.")
    .def("set_angular_velocity_z_noise", &sdf::Imu::SetAngularVelocityZNoise,
         "Set the noise values related to the body-frame angular "
         "acceleration on the Z-axis.")
    .def("gravity_dir_x", &sdf::Imu::GravityDirX,
         "Used when localization is set to GRAV_UP or GRAV_DOWN, a "
         "projection of this vector into a plane that is orthogonal to the "
         "gravity vector defines the direction of the IMU reference frame's "
         "X-axis.  grav_dir_x is  defined in the coordinate frame as defined "
         "by the parent_frame element.")
    .def("set_gravity_dir_x", &sdf::Imu::SetGravityDirX,
         "Used when localization is set to GRAV_UP or GRAV_DOWN, a "
         "projection of this vector into a plane that is orthogonal to the "
         "gravity vector defines the direction of the IMU reference frame's "
         "X-axis. grav_dir_x is  defined in the coordinate frame as defined "
         "by the parent_frame element.")
    .def("gravity_dir_x_parent_frame", &sdf::Imu::GravityDirXParentFrame,
         "Get the name of parent frame which the GravityDirX vector is "
         "defined relative to. It can be any valid fully scoped link name or "
         "the special reserved \"world\" frame. If left empty, use the "
         "sensor's ownlocal frame.")
    .def("set_gravity_dir_x_parent_frame", &sdf::Imu::SetGravityDirXParentFrame,
         "Set the name of parent frame which the GravityDirX vector is "
         "defined relative to. It can be any valid fully scoped link name or "
         "the special reserved \"world\" frame. If left empty, use the "
         "sensor's own local frame.")
    .def("localization", &sdf::Imu::Localization,
         "This string represents special hardcoded use cases that are "
         "commonly seen with typical robot IMU's")
    .def("set_localization", &sdf::Imu::SetLocalization,
         "See Localization(const std::string &).")
    .def("custom_rpy", &sdf::Imu::CustomRpy,
         "This field and CustomRpyParentFrame are used when "
         "Localization is set to CUSTOM. Orientation "
         "(fixed axis roll, pitch yaw) transform from ParentFrame to this "
         "IMU's reference frame.")
    .def("set_custom_rpy", &sdf::Imu::SetCustomRpy,
         "See CustomRpy() const.")
    .def("custom_rpy_parent_frame", &sdf::Imu::CustomRpyParentFrame,
         "Get the name of parent frame which the custom_rpy transform is "
         "defined relative to. It can be any valid fully scoped link name or "
         "the special reserved \"world\" frame. If left empty, use the "
         "sensor's own local frame.")
    .def("set_custom_rpy_parent_frame", &sdf::Imu::SetCustomRpyParentFrame,
         "Set the name of parent frame which the custom_rpy transform is "
         "defined relative to. It can be any valid fully scoped link name or "
         "the special reserved \"world\" frame. If left empty, use the "
         "sensor's own local frame.")
    .def("orientation_enabled", &sdf::Imu::OrientationEnabled,
         "Get whether orientation data generation is enabled.")
    .def("set_orientation_enabled", &sdf::Imu::SetOrientationEnabled,
         "Set whether to enable orientation data generation.")
    .def("__copy__", [](const sdf::Imu &self) {
      return sdf::Imu(self);
    })
    .def("__deepcopy__", [](const sdf::Imu &self, pybind11::dict) {
      return sdf::Imu(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
