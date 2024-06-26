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

#include "pyWorld.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Actor.hh"
#include "sdf/Atmosphere.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Physics.hh"
#include "sdf/World.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineWorld(pybind11::object module)
{
  // TODO(ahcorde)
  //  - Added Scene methods
  //  - Added Physics methods

  pybind11::class_<sdf::World> geometryModule(module, "World");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::World>())
    .def("resolve_auto_inertials", &sdf::World::ResolveAutoInertials,
         "Calculate and set the inertials for all the models in the world object")
    .def("validate_graphs", &sdf::World::ValidateGraphs,
         "Check that the FrameAttachedToGraph and PoseRelativeToGraph "
         "are valid.")
    .def("name", &sdf::World::Name,
         "Get the name of model.")
    .def("set_name", &sdf::World::SetName,
         "Set the name of model.")
    .def("audio_device", &sdf::World::AudioDevice,
         "Get the audio device name. The audio device can be used to "
         "playback audio files. A value of \"default\" or an empty string "
         "indicates that the system's default audio device should be used.")
    .def("set_audio_device", &sdf::World::SetAudioDevice,
         "Set the audio device name. See std::string AudioDevice() const "
         "for more information.")
    .def("wind_linear_velocity", &sdf::World::WindLinearVelocity,
         "Get the wind linear velocity in the global/world coordinate "
         "frame. Units are meters per second.")
    .def("set_wind_linear_velocity", &sdf::World::SetWindLinearVelocity,
         "Set the wind linear velocity in the global/world coordinate "
         "frame. Units are meters per second.")
    .def("gravity", &sdf::World::Gravity,
         "Get the acceleration due to gravity. Units are meters per "
         "second squared. The default value is "
         "Earth's standard gravity at sea level, which equals "
         "[0, 0, -9.80665].")
    .def("set_gravity", &sdf::World::SetGravity,
         "Set the acceleration due to gravity. Units are meters per "
         "second squared.")
    .def("magnetic_field", &sdf::World::MagneticField,
         "Get the magnetic vector in Tesla, expressed in "
         "a coordinate frame defined by the SphericalCoordinates property. "
         "A spherical coordinate can be specified in SDF using the "
         "<spherical_coordinates> element.")
    .def("set_magnetic_field", &sdf::World::SetMagneticField,
         "Set the magnetic vector in Tesla, expressed in "
         "a coordinate frame defined by the SphericalCoordinate. "
         "A spherical coordinate can be specified in SDF using the "
         "<spherical_coordinates> element.")
    .def("spherical_coordinates", &sdf::World::SphericalCoordinates,
         pybind11::return_value_policy::reference_internal,
         "Get a mutable spherical coordinates for the world origin.")
    .def("set_spherical_coordinates", &sdf::World::SetSphericalCoordinates,
         "Set the spherical coordinates for the world origin.")
    .def("model_count", &sdf::World::ModelCount,
         "Get the number of explicit models that are immediate (not nested) "
         "children of this World object.")
    .def("model_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::World::ModelByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a mutable immediate (not nested) child joint model on an index.")
     .def("model_by_name",
          pybind11::overload_cast<const std::string &>(
            &sdf::World::ModelByName),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable immediate (not nested) mutable child model based on an "
          "index.")
     .def("model_name_exists", &sdf::World::ModelNameExists,
          "Get whether a model name exists.")
     .def("name_exists_in_frame_attached_to_graph",
          &sdf::World::NameExistsInFrameAttachedToGraph,
          "Check if a given name exists in the FrameAttachedTo graph at the "
          "scope of the world.")
     .def("add_model", &sdf::World::AddModel,
          "Add a model to the world.")
     // .def("add_actor", &sdf::World::AddActor,
     //      "Add a actor to the world.")
     .def("add_light", &sdf::World::AddLight,
          "Add a light to the world.")
     .def("add_physics", &sdf::World::AddPhysics,
          "Add a physics object to the world.")
     .def("add_frame", &sdf::World::AddFrame,
          "Add a frame object to the world.")
     .def("add_joint", &sdf::World::AddJoint,
          "Add a joint to the world.")
     .def("clear_models", &sdf::World::ClearModels,
          "Remove all models.")
     // .def("clear_actors", &sdf::World::ClearActors,
     //      "Remove all actors.")
     .def("clear_lights", &sdf::World::ClearLights,
          "Remove all lights.")
     .def("clear_physics", &sdf::World::ClearPhysics,
          "Remove all physics objects.")
     .def("clear_frames", &sdf::World::ClearFrames,
          "Remove all frames.")
     .def("clear_joints", &sdf::World::ClearJoints,
          "Remove all joints.")
     .def("actor_count", &sdf::World::ActorCount,
          "Get the number of actors.")
     .def("frame_count", &sdf::World::FrameCount,
          "Get the number of explicit frames that are immediate (not nested) "
          "children of this World object.")
     .def("frame_by_index",
          pybind11::overload_cast<uint64_t>(
            &sdf::World::FrameByIndex),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable immediate (not nested) explicit frame based on an index.")
     .def("frame_by_name",
          pybind11::overload_cast<const std::string &>(
            &sdf::World::FrameByName),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable immediate (not nested) mutable child frame based on an "
          "index.")
     .def("frame_name_exists", &sdf::World::FrameNameExists,
          "Get whether a frame name exists.")
     .def("joint_count", &sdf::World::JointCount,
          "Get the number of joints.")
     .def("joint_by_index",
          pybind11::overload_cast<uint64_t>(
            &sdf::World::JointByIndex),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable joint based on an index.")
     .def("joint_by_name",
          pybind11::overload_cast<const std::string &>(
            &sdf::World::JointByName),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable joint based on name.")
     .def("joint_name_exists", &sdf::World::JointNameExists,
          "Get whether a joint name exists.")
     .def("light_count", &sdf::World::LightCount,
          "Get the number of lights.")
     .def("light_by_index",
          pybind11::overload_cast<uint64_t>(
            &sdf::World::LightByIndex),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable light based on an index.")
     .def("light_name_exists", &sdf::World::LightNameExists,
          "Get whether a light name exists.")
     .def("atmosphere", &sdf::World::Atmosphere,
           pybind11::return_value_policy::reference_internal,
          "Get a pointer to the atmosphere model associated with this "
          "world. A None indicates that an atmosphere model has not been "
          "set.")
     .def("set_atmosphere", &sdf::World::SetAtmosphere,
          "Set the atmosphere model associated with this world.")
     .def("gui", &sdf::World::Gui,
          pybind11::return_value_policy::reference_internal,
          "Get a pointer to the Gui associated with this "
          "world. None indicates that a Gui element has not been specified.")
     .def("set_gui", &sdf::World::SetGui,
          "Set the Gui parameters associated with this world.")
     .def("scene", &sdf::World::Scene,
          pybind11::return_value_policy::reference_internal,
          "Get a pointer to the Scene associated with this "
          "world. A nullptr indicates that a Scene element has not been "
          "specified.")
     .def("set_scene", &sdf::World::SetScene,
          "Set the Scene parameters associated with this world.")
     .def("physics_count", &sdf::World::PhysicsCount,
          "Get the number of physics profiles.")
     .def("physics_by_index",
          pybind11::overload_cast<uint64_t>(
            &sdf::World::PhysicsByIndex),
          pybind11::return_value_policy::reference_internal,
          "Get a mutable physics profile based on an index.")
     .def("physics_default",
          &sdf::World::PhysicsDefault,
          pybind11::return_value_policy::reference_internal,
          "Get the default physics profile.")
     .def("physics_name_exists", &sdf::World::PhysicsNameExists,
          "Get whether a physics profile name exists.")
     .def("plugins",
          pybind11::overload_cast<>(&sdf::World::Plugins, pybind11::const_),
          "Get the plugins attached to this object.")
     .def("clear_plugins", &sdf::World::ClearPlugins,
          "Remove all plugins")
     .def("add_plugin", &sdf::World::AddPlugin,
          "Add a plugin to this object.")
    .def("__copy__", [](const sdf::World &self) {
      return sdf::World(self);
    })
    .def("__deepcopy__", [](const sdf::World &self, pybind11::dict) {
      return sdf::World(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
