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

#include "pyPolyline.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sdf/Polyline.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void definePolyline(pybind11::object module)
{
  pybind11::class_<sdf::Polyline>(module, "Polyline")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Polyline>())
    .def("height", &sdf::Polyline::Height,
         "Get the polyline's height in meters.")
    .def("set_height", &sdf::Polyline::SetHeight,
         "Set the polyline's height in meters.")
    .def("point_count", &sdf::Polyline::PointCount,
         "Get the number of points.")
    .def("point_by_index",
         pybind11::overload_cast<uint64_t>(
           &sdf::Polyline::PointByIndex),
         pybind11::return_value_policy::reference_internal,
         "Get a point by its index.")
    .def("add_point", &sdf::Polyline::AddPoint,
         "Add a point to the polyline.")
    .def("clear_points", &sdf::Polyline::ClearPoints,
         "Remove all points from the polyline.")
    .def("points", &sdf::Polyline::Points,
         "Get the polyline's points. Each point has 2D coordinates in "
         "meters")
    .def("__copy__", [](const sdf::Polyline &self) {
      return sdf::Polyline(self);
    })
    .def("__deepcopy__", [](const sdf::Polyline &self, pybind11::dict) {
      return sdf::Polyline(self);
    }, "memo"_a);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
