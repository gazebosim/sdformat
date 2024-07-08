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

#include "pySensor.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Sensor.hh"

#include "sdf/Magnetometer.hh"
#include "sdf/Altimeter.hh"
#include "sdf/AirPressure.hh"
#include "sdf/Camera.hh"
#include "sdf/NavSat.hh"
#include "sdf/ForceTorque.hh"
#include "sdf/Imu.hh"
#include "sdf/Lidar.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineSensor(pybind11::object module)
{
  pybind11::class_<sdf::Sensor>(module, "Sensor")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Sensor>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("name", &sdf::Sensor::Name,
         "Get the name of the sensor.")
    .def("set_name", &sdf::Sensor::SetName,
         "Set the name of the sensor.")
    .def("frame_id", &sdf::Sensor::FrameId,
         "Get the frame id of the sensor.")
    .def("set_frame_id", &sdf::Sensor::SetFrameId,
         "Set the frame id of the sensor.")
    .def("topic", &sdf::Sensor::Topic,
         "Get the topic on which sensor data should be published.")
    .def("set_topic", &sdf::Sensor::SetTopic,
         "Set the topic on which sensor data should be published.")
    .def("enable_metrics", &sdf::Sensor::EnableMetrics,
         "Get flag state for enabling performance metrics publication.")
    .def("set_enable_metrics", &sdf::Sensor::SetEnableMetrics,
         "Set flag to enable publishing performance metrics")
    .def("raw_pose", &sdf::Sensor::RawPose,
         "Get the pose of the sensor. This is the pose of the sensor "
         "as specified in SDF (<sensor> <pose> ... </pose></sensor>), and is "
         "typically used to express the position and rotation of a sensor in a "
         "global coordinate frame.")
    .def("set_raw_pose", &sdf::Sensor::SetRawPose,
         "Set the pose of the sensor.")
    .def("pose_relative_to", &sdf::Sensor::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link/joint coordinate frame.")
    .def("set_pose_relative_to", &sdf::Sensor::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link/joint coordinate frame.")
    .def("semantic_pose", &sdf::Sensor::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("type", &sdf::Sensor::Type,
         "Get the sensor type.")
    .def("set_type",
         pybind11::overload_cast<const SensorType>(&sdf::Sensor::SetType),
         "Set the sensor type.")
    .def("set_type",
         pybind11::overload_cast<const std::string &>(&sdf::Sensor::SetType),
         "Set the sensor type from a string.")
    .def("type_str", &sdf::Sensor::TypeStr,
         "Get the sensor type as a string.")
    .def("update_rate", &sdf::Sensor::UpdateRate,
         "Get the update rate in Hz.")
    .def("set_update_rate", &sdf::Sensor::SetUpdateRate,
         "Set the update rate.")
    .def("magnetometer_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::MagnetometerSensor),
         pybind11::return_value_policy::reference_internal,
         "Get the magnetometer sensor, or None if this sensor type "
         "is not a Magnetometer.")
    .def("set_magnetometer_sensor", &sdf::Sensor::SetMagnetometerSensor,
         "Set the magnetometer sensor.")
    .def("altimeter_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::AltimeterSensor),
         pybind11::return_value_policy::reference_internal,
         "Get the altimeter sensor, or None if this sensor type "
         "is not an Altimeter.")
    .def("set_altimeter_sensor", &sdf::Sensor::SetAltimeterSensor,
         "Set the altimeter sensor.")
    .def("air_pressure_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::AirPressureSensor),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable air pressure sensor, or None if this sensor type "
         "is not an AirPressure sensor.")
    .def("set_air_pressure_sensor", &sdf::Sensor::SetAirPressureSensor,
         "Set the air pressure sensor.")
    .def("camera_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::CameraSensor),
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to a camera sensor, or None if the sensor "
         "does not contain a camera sensor.")
    .def("set_camera_sensor", &sdf::Sensor::SetCameraSensor,
         "Set the altimeter sensor.")
    .def("nav_sat_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::NavSatSensor),
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to a NAVSAT sensor, or None if the sensor "
         "does not contain an NAVSAT sensor.")
    .def("set_nav_sat_sensor", &sdf::Sensor::SetNavSatSensor,
         "Set the NAVSAT sensor.")
    .def("force_torque_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::ForceTorqueSensor),
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to a force torque sensor, or None if the sensor "
         "does not contain a force torque sensor.")
    .def("set_force_torque_sensor", &sdf::Sensor::SetForceTorqueSensor,
         "Set the force torque sensor.")
    .def("imu_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::ImuSensor),
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to an IMU sensor, or None if the sensor "
         "does not contain an IMU sensor.")
    .def("set_imu_sensor", &sdf::Sensor::SetImuSensor,
         "Set the IMU sensor.")
    .def("lidar_sensor",
         pybind11::overload_cast<>(
           &sdf::Sensor::LidarSensor),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable lidar sensor, or None if this sensor type is "
         "not a Lidar.")
    .def("set_lidar_sensor", &sdf::Sensor::SetLidarSensor,
         "Set the lidar sensor.")
    .def("plugins",
         pybind11::overload_cast<>(&sdf::Sensor::Plugins, pybind11::const_),
         "Get the plugins attached to this object.")
    .def("clear_plugins", &sdf::Sensor::ClearPlugins,
         "Remove all plugins")
    .def("add_plugin", &sdf::Sensor::AddPlugin,
         "Add a plugin to this object.")
    .def("__copy__", [](const sdf::Sensor &self) {
      return sdf::Sensor(self);
    })
    .def("__deepcopy__", [](const sdf::Sensor &self, pybind11::dict) {
      return sdf::Sensor(self);
    }, "memo"_a);

    pybind11::enum_<sdf::SensorType>(module, "Sensortype")
      .value("NONE", sdf::SensorType::NONE)
      .value("ALTIMETER", sdf::SensorType::ALTIMETER)
      .value("CAMERA", sdf::SensorType::CAMERA)
      .value("CONTACT", sdf::SensorType::CONTACT)
      .value("DEPTH_CAMERA", sdf::SensorType::DEPTH_CAMERA)
      .value("FORCE_TORQUE", sdf::SensorType::FORCE_TORQUE)
      .value("GPU_LIDAR", sdf::SensorType::GPU_LIDAR)
      .value("IMU", sdf::SensorType::IMU)
      .value("LOGICAL_CAMERA", sdf::SensorType::LOGICAL_CAMERA)
      .value("MAGNETOMETER", sdf::SensorType::MAGNETOMETER)
      .value("MULTICAMERA", sdf::SensorType::MULTICAMERA)
      .value("LIDAR", sdf::SensorType::LIDAR)
      .value("RFID", sdf::SensorType::RFID)
      .value("RFIDTAG", sdf::SensorType::RFIDTAG)
      .value("SONAR", sdf::SensorType::SONAR)
      .value("WIRELESS_RECEIVER", sdf::SensorType::WIRELESS_RECEIVER)
      .value("WIRELESS_TRANSMITTER", sdf::SensorType::WIRELESS_TRANSMITTER)
      .value("AIR_PRESSURE", sdf::SensorType::AIR_PRESSURE)
      .value("RGBD_CAMERA", sdf::SensorType::RGBD_CAMERA)
      .value("THERMAL_CAMERA", sdf::SensorType::THERMAL_CAMERA)
      .value("NAVSAT", sdf::SensorType::NAVSAT)
      .value("SEGMENTATION_CAMERA", sdf::SensorType::SEGMENTATION_CAMERA)
      .value("BOUNDINGBOX_CAMERA", sdf::SensorType::BOUNDINGBOX_CAMERA)
      .value("CUSTOM", sdf::SensorType::CUSTOM)
      .value("WIDE_ANGLE_CAMERA", sdf::SensorType::WIDE_ANGLE_CAMERA);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
