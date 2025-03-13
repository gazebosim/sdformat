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

#include "pyLight.hh"

#include <pybind11/pybind11.h>

#include "sdf/ParserConfig.hh"

#include "sdf/Box.hh"
#include "sdf/Capsule.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Ellipsoid.hh"
#include "sdf/Light.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Sphere.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineLight(pybind11::object module)
{
  pybind11::class_<sdf::Light> lightModule(module, "Light");
  lightModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Light>())
    .def("type", &sdf::Light::Type,
         "Get the type of light.")
    .def("set_type", &sdf::Light::SetType,
         "Set the type of light.")
    .def("name", &sdf::Light::Name,
         "Get the name of the light.")
    .def("set_name", &sdf::Light::SetName,
         "Set the name of the light.")
    .def("raw_pose", &sdf::Light::RawPose,
         "Get the pose of the camera. This is the pose of the Light "
         "as specified in SDF (<light> <pose> ... </pose></light>).")
    .def("set_raw_pose", &sdf::Light::SetRawPose,
         "Set the pose of the Light.")
    .def("pose_relative_to", &sdf::Light::PoseRelativeTo,
         "Get the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
    .def("set_pose_relative_to", &sdf::Light::SetPoseRelativeTo,
         "Set the name of the coordinate frame relative to which this "
         "object's pose is expressed. An empty value indicates that the frame "
         "is relative to the parent link.")
     .def("semantic_pose", &sdf::Light::SemanticPose,
          "Get SemanticPose object of this object to aid in resolving "
          "poses.")
    .def("cast_shadows", &sdf::Light::CastShadows,
         "Get whether the light casts shadows.")
    .def("set_cast_shadows", &sdf::Light::SetCastShadows,
         "Set whether the light casts shadows.")
    .def("light_on", &sdf::Light::LightOn,
         "Get if the light is on")
    .def("set_light_on", &sdf::Light::SetLightOn,
         "Set if the light is ON/OFF")
    .def("visualize", &sdf::Light::Visualize,
         "Whether light visualization in the GUI is enabled.")
    .def("set_visualize", &sdf::Light::SetVisualize,
         "Set whether light visualization in the GUI is enabled.")
    .def("intensity", &sdf::Light::Intensity,
         "Get the light intensity")
    .def("set_intensity", &sdf::Light::SetIntensity,
         "Set the light intensity")
    .def("diffuse", &sdf::Light::Diffuse,
         "Get the diffuse color. The diffuse color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("set_diffuse", &sdf::Light::SetDiffuse,
         "Set the diffuse color. The diffuse color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("specular", &sdf::Light::Specular,
         "Get the specular color. The specular color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("set_specular", &sdf::Light::SetSpecular,
         "Set the specular color. The specular color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("attenuation_range", &sdf::Light::AttenuationRange,
         "Get the range of the light source in meters.")
    .def("set_attenuation_range", &sdf::Light::SetAttenuationRange,
         "Set the range of the light source in meters.")
    .def("linear_attenuation_factor", &sdf::Light::LinearAttenuationFactor,
         "Get the linear attenuation factor. This value is clamped to "
         "a value between 0 and 1, where 1 means attenuate evenly over the "
         "distance.")
    .def("set_linear_attenuation_factor",
         &sdf::Light::SetLinearAttenuationFactor,
         "Set the linear attenuation factor. This value is clamped to "
         "a value between 0 and 1, where 1 means attenuate evenly over the "
         "distance.")
    .def("constant_attenuation_factor", &sdf::Light::ConstantAttenuationFactor,
         "Get the constant attenuation factor. This value is clamped to "
         "a value between 0 and 1,  where 1.0 means never attenuate and 0.0 is "
         "complete attenuation.")
    .def("set_constant_attenuation_factor",
         &sdf::Light::SetConstantAttenuationFactor,
         "Get the constant attenuation factor. This value is clamped to "
         "a value between 0 and 1,  where 1.0 means never attenuate and 0.0 is "
         "complete attenuation.")
    .def("quadratic_attenuation_factor", &sdf::Light::QuadraticAttenuationFactor,
         "Get the quadratic attenuation factor which adds a curvature to "
         "the attenuation.")
    .def("set_quadratic_attenuation_factor",
         &sdf::Light::SetQuadraticAttenuationFactor,
         "Set the quadratic attenuation factor which adds a curvature to "
         "the attenuation.")
    .def("direction", &sdf::Light::Direction,
         "Get the direction of the light source. This only has meaning "
         "for spot and directional light types. The default value is "
         "[0, 0, -1].")
    .def("set_direction", &sdf::Light::SetDirection,
         "Set the direction of the light source. This only has meaning "
         "for spot and directional light types.")
    .def("spot_inner_angle", &sdf::Light::SpotInnerAngle,
         "Get the angle covered by the bright inner cone.")
    .def("set_spot_inner_angle", &sdf::Light::SetSpotInnerAngle,
         "Set the angle covered by the bright inner cone.")
    .def("spot_outer_angle", &sdf::Light::SpotOuterAngle,
         "Get the angle covered by the outer cone.")
    .def("set_spot_outer_angle", &sdf::Light::SetSpotOuterAngle,
         "Set the angle covered by the bright inner cone.")
    .def("spot_falloff", &sdf::Light::SpotFalloff,
         "Get the rate of falloff between the inner and outer cones. "
         "A value of 1.0 is a linear falloff, less than 1.0 is a slower "
         "falloff, and a higher value indicates a faster falloff.")
    .def("set_spot_falloff", &sdf::Light::SetSpotFalloff,
         "Set the rate of falloff between the inner and outer cones. "
         "A value of 1.0 is a linear falloff, less than 1.0 is a slower "
         "falloff, and a higher value indicates a faster falloff.")
    .def("__copy__", [](const sdf::Light &self) {
      return sdf::Light(self);
    })
    .def("__deepcopy__", [](const sdf::Light &self, pybind11::dict) {
      return sdf::Light(self);
    }, "memo"_a);

    pybind11::enum_<sdf::LightType>(module, "LightType")
      .value("INVALID", sdf::LightType::INVALID)
      .value("POINT", sdf::LightType::POINT)
      .value("SPOT", sdf::LightType::SPOT)
      .value("DIRECTIONAL", sdf::LightType::DIRECTIONAL);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
