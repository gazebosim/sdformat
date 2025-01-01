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

#include "pyRoot.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "pybind11_helpers.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineRoot(pybind11::object module)
{
  pybind11::class_<sdf::Root>(module, "Root")
    .def(pybind11::init<>())
    .def("resolve_auto_inertials", &sdf::Root::ResolveAutoInertials,
         "Calculate and set the inertial properties")
    .def("load",
         [](Root &self, const std::string &_filename)
         {
           ThrowIfErrors(self.Load(_filename));
         },
         "Parse the given SDF file, and generate objects based on types "
         "specified in the SDF file.")
    .def("load",
         [](Root &self, const std::string &_filename,
            const ParserConfig &_config)
         {
           ThrowIfErrors(self.Load(_filename, _config));
         },
         "Parse the given SDF file, and generate objects based on types "
         "specified in the SDF file.")
   .def("load_sdf_string",
         [](Root &self, const std::string &_sdf)
         {
           ThrowIfErrors(self.LoadSdfString(_sdf));
         },
         "Parse the given SDF string, and generate objects based on types "
         "specified in the SDF file.")
    .def("load_sdf_string",
         [](Root &self, const std::string &_sdf, const ParserConfig &_config)
         {
           ThrowIfErrors(self.LoadSdfString(_sdf, _config));
         },
         "Parse the given SDF string, and generate objects based on types "
         "specified in the SDF file.")
    .def("version", &sdf::Root::Version,
         "Get the SDF version specified in the parsed file or SDF "
         "pointer.")
    .def("set_version", &sdf::Root::SetVersion,
         "Set the SDF version string.")
    .def("world_count", &sdf::Root::WorldCount,
         "Get the number of worlds.")
    .def("world_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Root::WorldByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a world based on an index.")
     .def("world_name_exists", &sdf::Root::WorldNameExists,
          "Get whether a world name exists.")
    .def("model", pybind11::overload_cast<sdf::Model *>(&sdf::Root::Model),
         pybind11::return_value_policy::reference_internal,
         "Get a model object if it exists.")
    .def("set_model", &sdf::Root::SetModel,
         pybind11::return_value_policy::reference_internal,
         "Set the model object. This will override any existing model, "
         "actor, and light object.")
    .def("light", &sdf::Root::Light,
         pybind11::return_value_policy::reference_internal,
         "Get a pointer to the light object if it exists.")
    .def("set_light", &sdf::Root::SetLight,
         "Set the light object. This will override any existing model, "
         "actor, and light object.")
    .def("clear_actor_light_model", &sdf::Root::ClearActorLightModel,
         "Remove the actor, light, or model if one of them exists."
         "The SDF Root object can only hold one, or none, from the set"
         "[Actor, Light, Model].")
    .def("add_world", [](Root &self, const World &_world)
         {
           ThrowIfErrors(self.AddWorld(_world));
         },
         "Add a world to the root.")
    .def("clear_worlds", &sdf::Root::ClearWorlds,
         "Remove all worlds.")
    .def("update_graphs", [](Root &self)
         {
           ThrowIfErrors(self.UpdateGraphs());
         },
         "Recreate the frame and pose graphs for the worlds and model "
         "that are children of this Root object. You can call this function "
         "to build new graphs when the DOM was created programmatically, or "
         "if you want to regenerate the graphs after editing the DOM.")
    .def("to_string", [](const sdf::Root &self)
         {
           return self.ToElement()->ToString("");
         })
    .def("__copy__", [](const sdf::Root &self) {
      return self.Clone();
    })
    .def("__deepcopy__", [](const sdf::Root &self, pybind11::dict) {
      return self.Clone();
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
