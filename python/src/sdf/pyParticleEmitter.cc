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

#include "pyParticleEmitter.hh"

#include <pybind11/pybind11.h>

#include "sdf/ParserConfig.hh"

#include "sdf/ParticleEmitter.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineParticleEmitter(pybind11::object module)
{
  pybind11::class_<sdf::ParticleEmitter> particleEmitterModule(module, "ParticleEmitter");
  particleEmitterModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::ParticleEmitter>())
    .def("name", &sdf::ParticleEmitter::Name,
         "Get the name of the particle emitter.")
    .def("set_name", &sdf::ParticleEmitter::SetName,
         "Set the name of the particle emitter.")
    .def("type", &sdf::ParticleEmitter::Type,
         "Get the type of particle emitter.")
    .def("set_type",
         pybind11::overload_cast<const sdf::ParticleEmitterType>(
           &sdf::ParticleEmitter::SetType),
         "Set the type of particle emitter.")
    .def("set_type",
         pybind11::overload_cast<const std::string &>(
           &sdf::ParticleEmitter::SetType),
         "Set the type of particle emitter.")
    .def("type_str", &sdf::ParticleEmitter::TypeStr,
         "Get the particle emitter type as a string.")
    .def("emitting", &sdf::ParticleEmitter::Emitting,
         "Get whether the particle emitter should run (emit "
         "particles).")
    .def("set_emitting", &sdf::ParticleEmitter::SetEmitting,
         "Set whether the particle emitter is running, emitting "
         "particles.")
    .def("duration", &sdf::ParticleEmitter::Duration,
         "Get the number of seconds the emitter is active."
         "A value less than or equal to zero indicates infinite duration.")
    .def("set_duration", &sdf::ParticleEmitter::SetDuration,
         "Set the number of seconds the emitter is active")
    .def("lifetime", &sdf::ParticleEmitter::Lifetime,
         "Get the number of seconds each particle will 'live' for "
         "before being destroyed.")
    .def("set_lifetime", &sdf::ParticleEmitter::SetLifetime,
         "Set the number of seconds each particle will 'live' for.")
    .def("rate", &sdf::ParticleEmitter::Rate,
         "Get the number of particles per second that should be emitted.")
    .def("set_rate", &sdf::ParticleEmitter::SetRate,
         "Set the number of particles per second that should be emitted.")
    .def("scale_rate", &sdf::ParticleEmitter::ScaleRate,
         "Get the amount by which to scale the particles in both x "
         "and y direction per second.")
    .def("set_scale_rate", &sdf::ParticleEmitter::SetScaleRate,
         "Set the amount by which to scale the particles in both x "
         "and y direction per second.")
    .def("min_velocity", &sdf::ParticleEmitter::MinVelocity,
         "Get the minimum velocity for each particle.")
    .def("set_min_velocity", &sdf::ParticleEmitter::SetMinVelocity,
         "Set the minimum velocity for each particle.")
    .def("max_velocity", &sdf::ParticleEmitter::MaxVelocity,
         "Get the maximum velocity for each particle.")
    .def("set_max_velocity", &sdf::ParticleEmitter::SetMaxVelocity,
         "Set the maximum velocity for each particle.")
    .def("size", &sdf::ParticleEmitter::Size,
         "Get the size of the emitter where the particles are sampled.")
    .def("set_size", &sdf::ParticleEmitter::SetSize,
         "Set the size of the emitter where the particles are sampled.")
    .def("particle_size", &sdf::ParticleEmitter::ParticleSize,
         "Get the size of a particle in meters.")
    .def("set_particle_size", &sdf::ParticleEmitter::SetParticleSize,
         "Set the size of a particle in meters.")
    .def("color_start", &sdf::ParticleEmitter::ColorStart,
         "Gets the starting color for all particles emitted.")
    .def("set_color_start", &sdf::ParticleEmitter::SetColorStart,
         "Set the starting color for all particles emitted.")
    .def("color_end", &sdf::ParticleEmitter::ColorEnd,
         "Get the end color for all particles emitted.")
    .def("set_color_end", &sdf::ParticleEmitter::SetColorEnd,
         "Set the end color for all particles emitted.")
    .def("color_range_image", &sdf::ParticleEmitter::ColorRangeImage,
         "Get the path to the color image used as an affector.")
    .def("set_color_range_image", &sdf::ParticleEmitter::SetColorRangeImage,
         "Set the path to the color image used as an affector.")
    .def("topic", &sdf::ParticleEmitter::Topic,
         "Get the topic used to update the particle emitter properties.")
    .def("set_topic", &sdf::ParticleEmitter::SetTopic,
         "Set the topic used to update the particle emitter properties.")
    .def("scatter_ratio", &sdf::ParticleEmitter::ScatterRatio,
         "Get the particle scatter ratio. This is used to determine the "
         "ratio of particles that will be detected by sensors.")
    .def("set_scatter_ratio", &sdf::ParticleEmitter::SetScatterRatio,
         "Set the particle scatter ratio. This is used to determine the "
         "ratio of particles that will be detected by sensors.")
    .def("raw_pose", &sdf::ParticleEmitter::RawPose,
         "Get the pose of the camer. This is the pose of the ParticleEmitter "
         "as specified in SDF (<particle_emitter> <pose> ... "
         "</pose></particle_emitter>).")
    .def("set_raw_pose", &sdf::ParticleEmitter::SetRawPose,
         "Set the pose of the ParticleEmitter.")
    .def("pose_relative_to", &sdf::ParticleEmitter::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("set_pose_relative_to", &sdf::ParticleEmitter::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("semantic_pose", &sdf::ParticleEmitter::SemanticPose,
         "Get SemanticPose object of this object to aid in resolving "
         "poses.")
    .def("material", &sdf::ParticleEmitter::Material,
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to the emitter's material properties. This can "
         "be a None if material properties have not been set.")
    .def("set_material", &sdf::ParticleEmitter::SetMaterial,
         "Set the emitter's material.")
    .def("file_path", &sdf::ParticleEmitter::FilePath,
         "The path to the file where this element was loaded from.")
    .def("set_file_path", &sdf::ParticleEmitter::SetFilePath,
         "Set the path to the file where this element was loaded from.")
    .def("__copy__", [](const sdf::ParticleEmitter &self) {
      return sdf::ParticleEmitter(self);
    })
    .def("__deepcopy__", [](const sdf::ParticleEmitter &self, pybind11::dict) {
      return sdf::ParticleEmitter(self);
    }, "memo"_a);

    pybind11::enum_<sdf::ParticleEmitterType>(particleEmitterModule, "ParticleEmitterType")
      .value("POINT", sdf::ParticleEmitterType::POINT)
      .value("BOX", sdf::ParticleEmitterType::BOX)
      .value("CYLINDER", sdf::ParticleEmitterType::CYLINDER)
      .value("ELLIPSOID", sdf::ParticleEmitterType::ELLIPSOID);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
