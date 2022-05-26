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

#include "pyPlugin.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Plugin.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void definePlugin(pybind11::object module)
{
  auto toString = [](const sdf::Plugin &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };
  pybind11::class_<sdf::Plugin>(module, "Plugin")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Plugin>())
    .def(pybind11::init<
      const std::string &, const std::string &, const std::string &>(),
      pybind11::arg("_filename"), pybind11::arg("_name"),
      pybind11::arg("_xmlContent") = "")
    .def("name", &sdf::Plugin::Name,
         "Get the name of the plugin.")
    .def("set_name", &sdf::Plugin::SetName,
         "Set the name of the plugin.")
    .def("filename", &sdf::Plugin::Filename,
         "Get the filename of the shared library.")
    .def("clear_contents", &sdf::Plugin::ClearContents,
         "Remove the contents of the plugin, this is everything that")
    .def("insert_content",
         pybind11::overload_cast<const std::string>
          (&sdf::Plugin::InsertContent),
         "Insert an element into the plugin content.")
    .def("set_filename", &sdf::Plugin::SetFilename,
         "Set the filename of the shared library.")
    .def("__str__", toString)
    .def("__repr__", toString)
    .def("__copy__", [](const sdf::Plugin &self) {
      return sdf::Plugin(self);
    })
    .def("__deepcopy__", [](const sdf::Plugin &self, pybind11::dict) {
      return sdf::Plugin(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
