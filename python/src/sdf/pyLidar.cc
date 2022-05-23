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

#include "pyLidar.hh"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "sdf/Lidar.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineLidar(pybind11::object module)
{
  pybind11::class_<sdf::Lidar> geometryModule(module, "Lidar");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Lidar>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def("horizontal_scan_samples", &sdf::Lidar::HorizontalScanSamples,
         "Get the number of lidar rays horizontally to generate per laser "
         "sweep.")
    .def("set_horizontal_scan_samples", &sdf::Lidar::SetHorizontalScanSamples,
         "Get the number of lidar rays horizontally to generate per laser "
         "sweep.")
    .def("horizontal_scan_resolution", &sdf::Lidar::HorizontalScanResolution,
         "Get the resolution for horizontal scan.")
    .def("set_horizontal_scan_resolution", &sdf::Lidar::SetHorizontalScanResolution,
         "Set the resolution for horizontal scan.")
    .def("horizontal_scan_min_angle", &sdf::Lidar::HorizontalScanMinAngle,
         "Get the minimum angle for horizontal scan.")
    .def("set_horizontal_scan_min_angle", &sdf::Lidar::SetHorizontalScanMinAngle,
         "Set the minimum angle for horizontal scan.")
    .def("horizontal_scan_max_angle", &sdf::Lidar::HorizontalScanMaxAngle,
         "Get the maximum angle for horizontal scan.")
    .def("set_horizontal_scan_max_angle", &sdf::Lidar::SetHorizontalScanMaxAngle,
         "Set the maximum angle for horizontal scan.")
    .def("vertical_scan_samples", &sdf::Lidar::VerticalScanSamples,
         "Get the number of lidar rays vertically to generate per laser "
         "sweep.")
    .def("set_vertical_scan_samples", &sdf::Lidar::SetVerticalScanSamples,
         "Set the number of lidar rays vertically to generate per laser "
         "sweep.")
    .def("vertical_scan_resolution", &sdf::Lidar::VerticalScanResolution,
         "Get the resolution for vertical scan.")
    .def("set_vertical_scan_resolution", &sdf::Lidar::SetVerticalScanResolution,
         "Set the resolution for vertical scan.")
    .def("vertical_scan_min_angle", &sdf::Lidar::VerticalScanMinAngle,
         "Get the minimum angle for vertical scan.")
    .def("set_vertical_scan_min_angle", &sdf::Lidar::SetVerticalScanMinAngle,
         "Set the minimum angle for vertical scan.")
    .def("vertical_scan_max_angle", &sdf::Lidar::VerticalScanMaxAngle,
         "Get the maximum angle for vertical scan.")
    .def("set_vertical_scan_max_angle",
         &sdf::Lidar::SetVerticalScanMaxAngle,
         "Set the maximum angle for vertical scan.")
    .def("range_min",
         &sdf::Lidar::RangeMin,
         "Get minimum distance for each lidar ray.")
    .def("set_range_min", &sdf::Lidar::SetRangeMin,
         "Set minimum distance for each lidar ray.")
    .def("range_max",
         &sdf::Lidar::RangeMax,
         "Get the maximum angle for vertical scan.")
    .def("set_range_max",
         &sdf::Lidar::SetRangeMax,
         "Set the maximum angle for vertical scan.")
    .def("range_resolution",
         &sdf::Lidar::RangeResolution,
         "Get linear resolution of each lidar ray.")
    .def("set_range_resolution",
         &sdf::Lidar::SetRangeResolution,
         "Set linear resolution of each lidar ray.")
    .def("lidar_noise",
         &sdf::Lidar::LidarNoise,
         "Get the noise values for the lidar sensor.")
    .def("set_lidar_noise", &sdf::Lidar::SetLidarNoise,
         "Set the noise values for the lidar sensor.")
    .def("visibility_mask",
         &sdf::Lidar::VisibilityMask,
         "Get the visibility mask of a lidar")
    .def("set_visibility_mask", &sdf::Lidar::SetVisibilityMask,
         "Set the visibility mask of a lidar.")
    .def("__copy__", [](const sdf::Lidar &self) {
      return sdf::Lidar(self);
    })
    .def("__deepcopy__", [](const sdf::Lidar &self, pybind11::dict) {
      return sdf::Lidar(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
