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

#include "pyPbr.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Pbr.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void definePbr(pybind11::object module)
{
  pybind11::class_<sdf::Pbr> pbrModule(module, "Pbr");
  pbrModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Pbr>())
    .def("set_workflow", &sdf::Pbr::SetWorkflow,
         "Set a PBR workflow by type")
    .def("workflow", &sdf::Pbr::Workflow,
         pybind11::return_value_policy::reference_internal,
         "Get a PBR workflow by type")
    .def("__copy__", [](const sdf::Pbr &self) {
      return sdf::Pbr(self);
    })
    .def("__deepcopy__", [](const sdf::Pbr &self, pybind11::dict) {
      return sdf::Pbr(self);
    }, "memo"_a);

  pybind11::enum_<sdf::NormalMapSpace>(module, "NormalMapSpace")
    .value("TANGENT", sdf::NormalMapSpace::TANGENT)
    .value("OBJECT", sdf::NormalMapSpace::OBJECT);
}

/////////////////////////////////////////////////
void definePbrWorkflow(pybind11::object module)
{
  pybind11::class_<sdf::PbrWorkflow> workflowModule(module, "PbrWorkflow");
  workflowModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::PbrWorkflow>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("albedo_map", &sdf::PbrWorkflow::AlbedoMap,
         "Get the albedo map filename. This will be an empty string if "
         "an albedo map has not been set.")
    .def("set_albedo_map", &sdf::PbrWorkflow::SetAlbedoMap,
         "Set the albedo map filename.")
    .def("normal_map", &sdf::PbrWorkflow::NormalMap,
         "Get the normal map filename. This will be an empty string if "
         "a normal map has not been set.")
    .def("set_normal_map", &sdf::PbrWorkflow::SetNormalMap,
         pybind11::arg("_map"),
         pybind11::arg("_space") = sdf::NormalMapSpace::TANGENT,
         "Set the normal map filename.")
    .def("normal_map_type", &sdf::PbrWorkflow::NormalMapType,
         "Get the normal map type, either tangent or object space")
    .def("environment_map", &sdf::PbrWorkflow::EnvironmentMap,
         "Get the environment map filename. This will be an empty string "
         "if an environment map has not been set.")
    .def("set_environment_map", &sdf::PbrWorkflow::SetEnvironmentMap,
         "Set the environment map filename.")
    .def("ambient_occlusion_map", &sdf::PbrWorkflow::AmbientOcclusionMap,
         "Get the ambient occlusion map filename. This will be an empty "
         "string if an ambient occlusion map has not been set.")
    .def("set_ambient_occlusion_map", &sdf::PbrWorkflow::SetAmbientOcclusionMap,
         "Set the ambient occlusion map filename.")
    .def("roughness_map", &sdf::PbrWorkflow::RoughnessMap,
         "Get the roughness map filename for metal workflow. This will be "
         "an empty string if a roughness map has not been set.")
    .def("set_roughness_map", &sdf::PbrWorkflow::SetRoughnessMap,
         "Set the roughness map filename for metal workflow.")
    .def("metalness_map", &sdf::PbrWorkflow::MetalnessMap,
         "Set the metalness map filename for metal workflow. This will be "
         "an empty string if a metalness map has not been set.")
    .def("set_metalness_map", &sdf::PbrWorkflow::SetMetalnessMap,
         "Set the metalness map filename for metal workflow.")
    .def("emissive_map", &sdf::PbrWorkflow::EmissiveMap,
         "Get the emissive map filename. This will be an empty string "
         "if an emissive map has not been set.")
    .def("set_emissive_map", &sdf::PbrWorkflow::SetEmissiveMap,
         "Set the emissive map filename.")
    .def("light_map", &sdf::PbrWorkflow::LightMap,
         "Get the light map filename. This will be an empty string "
         "if an light map has not been set.")
    .def("set_light_map", &sdf::PbrWorkflow::SetLightMap,
         pybind11::arg("_map"), pybind11::arg("_uvSet") = 0u,
         "Set the light map filename.")
    .def("light_map_tex_coord_set", &sdf::PbrWorkflow::LightMapTexCoordSet,
         "Get the light map texture coordinate set.")
    .def("metalness", &sdf::PbrWorkflow::Metalness,
         "Get the metalness value of the material for metal workflow")
    .def("set_metalness", &sdf::PbrWorkflow::SetMetalness,
         "Set the metalness value of the material for metal workflow.")
    .def("roughness", &sdf::PbrWorkflow::Roughness,
         "Get the roughness value of the material for metal workflow")
    .def("set_roughness", &sdf::PbrWorkflow::SetRoughness,
         "Set the roughness value of the material for metal workflow.")
    .def("glossiness_map", &sdf::PbrWorkflow::GlossinessMap,
         "Get the glossiness map filename for specular workflow. This will "
         "be an empty string if a glossiness map has not been set.")
    .def("set_glossiness_map", &sdf::PbrWorkflow::SetGlossinessMap,
         "Set the glossiness map filename for specular workflow.")
    .def("glossiness", &sdf::PbrWorkflow::Glossiness,
         "Get the glossiness value of the material for specular workflow")
    .def("set_glossiness", &sdf::PbrWorkflow::SetGlossiness,
         "Set the glossiness value of the material for specular workflow.")
    .def("specular_map", &sdf::PbrWorkflow::SpecularMap,
         "Get the specular map filename for specular workflow. This will "
         "be an empty string if a specular map has not been set.")
    .def("set_specular_map", &sdf::PbrWorkflow::SetSpecularMap,
         "Set the specular map filename for specular workflow.")
    .def("type", &sdf::PbrWorkflow::Type,
         "Get the workflow type.")
    .def("set_type", &sdf::PbrWorkflow::SetType,
         "Set the PBR workflow to use")
    .def("__copy__", [](const sdf::PbrWorkflow &self) {
      return sdf::PbrWorkflow(self);
    })
    .def("__deepcopy__", [](const sdf::PbrWorkflow &self, pybind11::dict) {
      return sdf::PbrWorkflow(self);
    }, "memo"_a);

    pybind11::enum_<sdf::PbrWorkflowType>(module, "PbrWorkflowType")
      .value("NONE", sdf::PbrWorkflowType::NONE)
      .value("METAL", sdf::PbrWorkflowType::METAL)
      .value("SPECULAR", sdf::PbrWorkflowType::SPECULAR);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
