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

#include "pyHeightmap.hh"

#include <pybind11/pybind11.h>

#include "sdf/Heightmap.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineHeightmapTexture(pybind11::object module)
{
  pybind11::class_<sdf::HeightmapTexture>(module, "HeightmapTexture")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::HeightmapTexture>())
    .def("size", &sdf::HeightmapTexture::Size,
         "Get the heightmap texture's size.")
    .def("set_size", &sdf::HeightmapTexture::SetSize,
         "Set the size of the texture in meters.")
    .def("diffuse", &sdf::HeightmapTexture::Diffuse,
         "Get the heightmap texture's diffuse map.")
    .def("set_diffuse", &sdf::HeightmapTexture::SetDiffuse,
         "Set the filename of the diffuse map.")
    .def("normal", &sdf::HeightmapTexture::Normal,
         "Get the heightmap texture's normal map.")
    .def("set_normal", &sdf::HeightmapTexture::SetNormal,
         "Set the filename of the normal map.")
    .def("__copy__", [](const sdf::HeightmapTexture &self) {
      return sdf::HeightmapTexture(self);
    })
    .def("__deepcopy__", [](const sdf::HeightmapTexture &self, pybind11::dict) {
      return sdf::HeightmapTexture(self);
    }, "memo"_a);
}

/////////////////////////////////////////////////
void defineHeightmapBlend(pybind11::object module)
{
  pybind11::class_<sdf::HeightmapBlend>(module, "HeightmapBlend")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::HeightmapBlend>())
    .def("min_height", &sdf::HeightmapBlend::MinHeight,
         "Get the heightmap blend's minimum height.")
    .def("set_min_height", &sdf::HeightmapBlend::SetMinHeight,
         "Set the minimum height of the blend in meters.")
    .def("fade_distance", &sdf::HeightmapBlend::FadeDistance,
         "Get the heightmap blend's fade distance.")
    .def("set_fade_distance", &sdf::HeightmapBlend::SetFadeDistance,
         "Set the distance over which the blend occurs.")
    .def("__copy__", [](const sdf::HeightmapBlend &self) {
      return sdf::HeightmapBlend(self);
    })
    .def("__deepcopy__", [](const sdf::HeightmapBlend &self, pybind11::dict) {
      return sdf::HeightmapBlend(self);
    }, "memo"_a);
}

/////////////////////////////////////////////////
void defineHeightmap(pybind11::object module)
{
  pybind11::class_<sdf::Heightmap>(module, "Heightmap")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Heightmap>())
    .def("uri", &sdf::Heightmap::Uri,
         "Get the heightmap's URI.")
    .def("set_uri", &sdf::Heightmap::SetUri,
         "Set the URI to a grayscale image.")
    .def("file_path", &sdf::Heightmap::FilePath,
         "The path to the file where this element was loaded from.")
    .def("set_file_path", &sdf::Heightmap::SetFilePath,
         "Set the path to the file where this element was loaded from.")
    .def("size", &sdf::Heightmap::Size,
         "Get the heightmap's scaling factor.")
    .def("set_size", &sdf::Heightmap::SetSize,
         "Set the heightmap's scaling factor. Defaults to 1x1x1.")
    .def("position", &sdf::Heightmap::Position,
         "Get the heightmap's position offset.")
    .def("set_position", &sdf::Heightmap::SetPosition,
         "Set the heightmap's position offset.")
    .def("use_terrain_paging", &sdf::Heightmap::UseTerrainPaging,
         "Get whether the heightmap uses terrain paging.")
    .def("set_use_terrain_paging", &sdf::Heightmap::SetUseTerrainPaging,
         "Set whether the heightmap uses terrain paging. Defaults to false.")
    .def("sampling", &sdf::Heightmap::Sampling,
         "Get the heightmap's sampling per datum.")
    .def("set_sampling", &sdf::Heightmap::SetSampling,
         "Set the heightmap's sampling. Defaults to 1.")
    .def("texture_count", &sdf::Heightmap::TextureCount,
         "Get the number of heightmap textures.")
    .def("texture_by_index", &sdf::Heightmap::TextureByIndex,
         pybind11::return_value_policy::reference,
         "Get a heightmap texture based on an index.")
    .def("add_texture", &sdf::Heightmap::AddTexture,
         "Add a heightmap texture.")
    .def("blend_count", &sdf::Heightmap::BlendCount,
         "Get the number of heightmap blends.")
    .def("blend_by_index", &sdf::Heightmap::BlendByIndex,
         pybind11::return_value_policy::reference,
         "Get a heightmap blend based on an index.")
    .def("add_blend", &sdf::Heightmap::AddBlend,
         "Add a heightmap blend.")
    .def("__copy__", [](const sdf::Heightmap &self) {
      return sdf::Heightmap(self);
    })
    .def("__deepcopy__", [](const sdf::Heightmap &self, pybind11::dict) {
      return sdf::Heightmap(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
