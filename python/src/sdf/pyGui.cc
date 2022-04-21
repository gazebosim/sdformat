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

#include "pyGui.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Gui.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineGui(pybind11::object module)
{
  pybind11::class_<sdf::Gui>(module, "Gui")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Gui>())
    .def(pybind11::self == pybind11::self)
    .def("fullscreen", &sdf::Gui::Fullscreen,
         "Get whether the Gui should be fullscreen.")
    .def("set_fullscreen", &sdf::Gui::SetFullscreen,
         "Set whether the Gui should be full screen.")
    .def("plugin_count", &sdf::Gui::PluginCount,
         "Get the filename of the shared library.")
    .def("plugin_by_index", &sdf::Gui::PluginByIndex,
         pybind11::return_value_policy::reference,
         "Get a plugin based on an index.")
    .def("clear_plugins", &sdf::Gui::ClearPlugins,
         "Remove all plugins")
    .def("add_plugin", &sdf::Gui::AddPlugin,
         "Add a plugin to this object.")
    .def("plugins",
         pybind11::overload_cast<>(&sdf::Gui::Plugins),
         "Get a mutable vector of plugins attached to this object")
    .def("__copy__", [](const sdf::Gui &self) {
      return sdf::Gui(self);
    })
    .def("__deepcopy__", [](const sdf::Gui &self, pybind11::dict) {
      return sdf::Gui(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
