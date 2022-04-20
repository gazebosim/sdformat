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

#include "pySky.hh"

#include <pybind11/pybind11.h>

#include "sdf/Sky.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineSky(pybind11::object module)
{
  pybind11::class_<sdf::Sky>(module, "Sky")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Sky>())
    .def("time", &sdf::Sky::Time,
         "Get time of day [0..24]")
    .def("set_time", &sdf::Sky::SetTime,
         "Set time of day")
    .def("sunrise", &sdf::Sky::Sunrise,
         "Get sunrise time")
    .def("set_sunrise", &sdf::Sky::SetSunrise,
         "Set Sunrise time")
    .def("sunset", &sdf::Sky::Sunset,
         "Get sunset time")
    .def("set_sunset", &sdf::Sky::SetSunset,
         "Set Sunset time")
    .def("cloud_speed", &sdf::Sky::CloudSpeed,
         "Get cloud speed")
    .def("set_cloud_speed", &sdf::Sky::SetCloudSpeed,
         "Set cloud speed")
    .def("cloud_direction", &sdf::Sky::CloudDirection,
         "Get cloud direction angle (angle around up axis)")
    .def("set_cloud_direction", &sdf::Sky::SetCloudDirection,
         "Set cloud direction angle (angle around up axis)")
    .def("cloud_humidity", &sdf::Sky::CloudHumidity,
         "Get cloud humidity.")
    .def("set_cloud_humidity", &sdf::Sky::SetCloudHumidity,
         "Set cloud humidity")
    .def("cloud_mean_size", &sdf::Sky::CloudMeanSize,
         "Get cloud mean size")
    .def("set_cloud_mean_size", &sdf::Sky::SetCloudMeanSize,
         "Set cloud mean size")
    .def("CloudAmbient", &sdf::Sky::CloudAmbient,
         "Get cloud ambient color")
    .def("set_CloudAmbient", &sdf::Sky::SetCloudAmbient,
         "Set cloud ambient color")
    .def("__copy__", [](const sdf::Sky &self) {
      return sdf::Sky(self);
    })
    .def("__deepcopy__", [](const sdf::Sky &self, pybind11::dict) {
      return sdf::Sky(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
