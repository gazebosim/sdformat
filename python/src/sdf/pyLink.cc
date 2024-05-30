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
#include "pyLink.hh"

#include <pybind11/pybind11.h>

#include <string>

#include "sdf/Link.hh"
#include "sdf/Visual.hh"
#include "sdf/Collision.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Projector.hh"
#include "sdf/Sensor.hh"
#include "sdf/Light.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineLink(pybind11::object module)
{
  pybind11::class_<sdf::Link>(module, "Link")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Link>())
    .def("name", &sdf::Link::Name,
         "Get the name of the link.")
    .def("set_name", &sdf::Link::SetName,
         "Set the name of the link.")
    .def("visual_count",
         &sdf::Link::VisualCount,
         "Get the number of visuals.")
    .def("visual_by_index",
         pybind11::overload_cast<uint64_t>(&sdf::Link::VisualByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable visual based on an index.")
    .def("visual_name_exists",
         &sdf::Link::VisualNameExists,
         "Get whether a visual name exists.")
    .def("visual_by_name",
         pybind11::overload_cast<const std::string &>(&sdf::Link::VisualByName),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable visual based on a name.")
    .def("collision_count",
         &sdf::Link::CollisionCount,
         "Get the number of collisions.")
    .def("collision_by_index",
         pybind11::overload_cast<uint64_t>(&sdf::Link::CollisionByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable collision based on an index.")
    .def("collision_name_exists",
         &sdf::Link::CollisionNameExists,
         "Get whether a collision name exists.")
    .def("collision_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Link::CollisionByName),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable collision based on a name.")
    .def("light_count", &sdf::Link::LightCount,
         "Get the number of lights.")
    .def("light_by_index",
         pybind11::overload_cast<const uint64_t>(
           &sdf::Link::LightByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable light based on an index.")
    .def("light_name_exists", &sdf::Link::LightNameExists,
         "Get whether a light name exists.")
    .def("light_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Link::LightByName),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable light based on a name.")
    .def("projector_count", &sdf::Link::ProjectorCount,
         "Get the number of projectors.")
    .def("projector_by_index",
         pybind11::overload_cast<const uint64_t>(
           &sdf::Link::ProjectorByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable projector based on an index.")
    .def("projector_name_exists", &sdf::Link::ProjectorNameExists,
         "Get whether a projector name exists.")
    .def("projector_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Link::ProjectorByName),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable projector based on a name.")
    .def("sensor_count", &sdf::Link::SensorCount,
         "Get the number of sensors.")
    .def("sensor_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Link::SensorByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a sensor based on an index.")
    .def("sensor_name_exists", &sdf::Link::SensorNameExists,
         "Get whether a sensor name exists.")
    .def("sensor_by_name",
         pybind11::overload_cast<const std::string &>(
           &sdf::Link::SensorByName),
         pybind11::return_value_policy::reference_internal,
         "Get a sensor based on a name.")
    // TODO(ahcorde): Enable particle emitter
    // .def(
    //     "ParticleEmitterCount", &sdf::Link::ParticleEmitterCount,
    //     "Get the number of particle emitters.")
    // .def(
    //     "ParticleEmitterByIndex", &sdf::Link::ParticleEmitterByIndex,
    //     "Get a particle emitter  based on an index.")
    // .def(
    //     "ParticleEmitterNameExists", &sdf::Link::ParticleEmitterNameExists,
    //     "Get whether a particle emitter name exists.")
    // .def(
    //     "ParticleEmitterByName", &sdf::Link::ParticleEmitterByName,
    //     "Get a particle emitter based on a name.")
    .def("inertial",
         &sdf::Link::Inertial,
         "Get the inertial value for this link")
    .def("set_inertial",
         &sdf::Link::SetInertial,
         "Set the inertial value for this link.")
    .def("raw_pose", &sdf::Link::RawPose,
         "Get the pose of the link object. This is the pose of the "
         "link as specified in SDF")
    .def("set_raw_pose",
         &sdf::Link::SetRawPose,
         "Set the pose of the link.")
    .def("pose_relative_to",
         &sdf::Link::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent model.")
    .def("set_pose_relative_to",
         &sdf::Link::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent model.")
    .def("semantic_pose",
         &sdf::Link::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("enable_wind",
         &sdf::Link::EnableWind,
         "Check if this link should be subject to wind. "
         "If true, this link should be affected by wind.")
    .def("set_enable_wind",
         &sdf::Link::SetEnableWind,
         "Set whether this link should be subject to wind.")
    .def("enable_gravity",
         &sdf::Link::EnableGravity,
         "Check if this link should be subject to gravity. "
         "If true, this link should be affected by gravity.")
    .def("set_enable_gravity",
         &sdf::Link::SetEnableGravity,
         "Set whether this link should be subject to gravity.")
    .def("add_collision",
         &sdf::Link::AddCollision,
         "Add a collision to the link.")
    .def("add_visual",
         &sdf::Link::AddVisual,
         "Add a visual to the link.")
    .def("add_light",
         &sdf::Link::AddLight,
         "Add a light to the link.")
    .def("add_sensor",
         &sdf::Link::AddSensor,
         "Add a sensor to the link.")
    // .def("AddParticleEmitter",
    //      &sdf::Link::AddParticleEmitter,
    //      "Add a particle emitter to the link.")
    .def("add_projector",
         &sdf::Link::AddProjector,
         "Add a projector to the link.")
    .def("clear_collisions",
         &sdf::Link::ClearCollisions,
         "Remove all collisions")
    .def("clear_visuals",
         &sdf::Link::ClearVisuals,
         "Remove all visuals")
    .def("clear_lights",
         &sdf::Link::ClearLights,
         "Remove all lights")
    .def("clear_sensors",
         &sdf::Link::ClearSensors,
         "Remove all sensors")
    // .def("clear_particle_emitters",
    //      &sdf::Link::ClearParticleEmitters,
    //      "Remove all particle emitters")
    .def("clear_projector",
         &sdf::Link::ClearProjectors,
         "Remove all projectors")
    .def("__copy__", [](const sdf::Link &self) {
      return sdf::Link(self);
    })
    .def("__deepcopy__", [](const sdf::Link &self, pybind11::dict) {
      return sdf::Link(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
