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

#include "pyMaterial.hh"

#include <pybind11/pybind11.h>

#include "sdf/Material.hh"
#include "sdf/Pbr.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineMaterial(pybind11::object module)
{
  pybind11::class_<sdf::Material> materialModule(module, "Material");
  materialModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Material>())
    .def("ambient", &sdf::Material::Ambient,
         "Get the ambient color. The ambient color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("set_ambient", &sdf::Material::SetAmbient,
         "Set the ambient color. The ambient color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("diffuse", &sdf::Material::Diffuse,
         "Get the diffuse color. The diffuse color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("set_diffuse", &sdf::Material::SetDiffuse,
         "Set the diffuse color. The diffuse color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("specular", &sdf::Material::Specular,
         "Get the specular color. The specular color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("set_specular", &sdf::Material::SetSpecular,
         "Set the specular color. The specular color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("shininess", &sdf::Material::Shininess,
         "Get the specular exponent.")
    .def("set_shininess", &sdf::Material::SetShininess,
         "Set the specular exponent.")
    .def("emissive", &sdf::Material::Emissive,
         "Get the emissive color. The emissive color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("set_emissive", &sdf::Material::SetEmissive,
         "Set the emissive color. The emissive color is "
         "specified by a set of three numbers representing red/green/blue, "
         "each in the range of [0,1].")
    .def("render_order", &sdf::Material::RenderOrder,
         "Get render order for coplanar polygons. The higher value will "
         "be rendered on top of the other coplanar polygons. The default value "
         "is zero.")
    .def("set_render_order", &sdf::Material::SetRenderOrder,
         "Set render order.")
    .def("lighting", &sdf::Material::Lighting,
         "Get whether dynamic lighting is enabled. The default "
         "value is true.")
    .def("set_lighting", &sdf::Material::SetLighting,
        "Set whether dynamic lighting is enabled.")
    .def("double_sided", &sdf::Material::DoubleSided,
         "Get whether double sided material is enabled. The default "
         "value is false.")
    .def("set_double_sided", &sdf::Material::SetDoubleSided,
         "Set whether double sided material is enabled.")
    .def("script_uri", &sdf::Material::ScriptUri,
         "Get the URI of the material script, if one has been set.")
    .def("set_script_uri", &sdf::Material::SetScriptUri,
         "Set the URI of the material script.")
    .def("script_name", &sdf::Material::ScriptName,
         "Get the name of the material script, or empty if one has not "
         "been specified. The name should match an "
         "script element in the script located at the ScriptUri().")
    .def("set_script_name", &sdf::Material::SetScriptName,
         "Set the name of the material script. The name should match an "
         "script element in the script located at the ScriptUri().")
    .def("shader", &sdf::Material::Shader,
         "Get the type of shader.")
    .def("set_shader", &sdf::Material::SetShader,
         "Set the type of shader.")
    .def("normal_map", &sdf::Material::NormalMap,
         "Get the normal map filename. This will be an empty string if "
         "a normal map has not been set.")
    .def("set_normal_map", &sdf::Material::SetNormalMap,
         "Set the normal map filename.")
    .def("set_pbr_material", &sdf::Material::SetPbrMaterial,
         pybind11::return_value_policy::reference_internal,
         "Set the Physically Based Rendering (PBR) material")
    .def("pbr_material", &sdf::Material::PbrMaterial,
         pybind11::return_value_policy::reference_internal,
         "Get the Physically Based Rendering (PBR) material")
    .def("file_path", &sdf::Material::FilePath,
         "The path to the file where this element was loaded from.")
    .def("set_file_path", &sdf::Material::SetFilePath,
         "Set the path to the file where this element was loaded from.")
    .def("__copy__", [](const sdf::Material &self) {
      return sdf::Material(self);
    })
    .def("__deepcopy__", [](const sdf::Material &self, pybind11::dict) {
      return sdf::Material(self);
    }, "memo"_a);

    pybind11::enum_<sdf::ShaderType>(module, "ShaderType")
      .value("PIXEL", sdf::ShaderType::PIXEL)
      .value("VERTEX", sdf::ShaderType::VERTEX)
      .value("NORMAL_MAP_OBJECTSPACE", sdf::ShaderType::NORMAL_MAP_OBJECTSPACE)
      .value("NORMAL_MAP_TANGENTSPACE",
             sdf::ShaderType::NORMAL_MAP_TANGENTSPACE);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
