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

#include "pyScene.hh"

#include <pybind11/pybind11.h>

#include "sdf/Scene.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineScene(pybind11::object module)
{
  pybind11::class_<sdf::Scene>(module, "Scene")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Scene>())
    .def("ambient", &sdf::Scene::Ambient,
         "Get the ambient color of the scene")
    .def("set_ambient", &sdf::Scene::SetAmbient,
         "Set the ambient color of the scene")
    .def("background", &sdf::Scene::Background,
         "Get the background color of the scene")
    .def("set_background", &sdf::Scene::SetBackground,
         "Set the background color of the scene")
    .def("grid", &sdf::Scene::Grid,
         "Get whether grid is enabled")
    .def("set_grid", &sdf::Scene::SetGrid,
         "Set whether the grid should be enabled")
    .def("origin_visual", &sdf::Scene::OriginVisual,
         "Get whether origin visual is enabled")
    .def("set_origin_visual", &sdf::Scene::SetOriginVisual,
         "Set whether the origin visual should be enabled")
    .def("shadows", &sdf::Scene::Shadows,
         "Get whether shadows are enabled")
    .def("set_shadows", &sdf::Scene::SetShadows,
         "Set whether shadows should be enabled")
    .def("set_sky", &sdf::Scene::SetSky,
         "Set sky")
    .def("sky", &sdf::Scene::Sky,
         pybind11::return_value_policy::reference_internal,
         "Get sky")
    .def("__copy__", [](const sdf::Scene &self) {
      return sdf::Scene(self);
    })
    .def("__deepcopy__", [](const sdf::Scene &self, pybind11::dict) {
      return sdf::Scene(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
