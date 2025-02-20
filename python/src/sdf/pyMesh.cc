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

#include "pyMesh.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "sdf/ParserConfig.hh"
#include "sdf/Mesh.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineMesh(pybind11::object module)
{
  pybind11::class_<sdf::Mesh>(module, "Mesh")
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Mesh>())
    .def("optimization", &sdf::Mesh::Optimization,
         "Get the mesh's optimization method.")
    .def("optimization_str", &sdf::Mesh::OptimizationStr,
         "Get the mesh's optimization method")
    .def("set_optimization",
         pybind11::overload_cast<sdf::MeshOptimization>(
           &sdf::Mesh::SetOptimization),
         "Set the mesh optimization method.")
    .def("set_optimization",
         pybind11::overload_cast<const std::string &>(
           &sdf::Mesh::SetOptimization),
         "Set the mesh optimization method.")
    .def("convex_decomposition", &sdf::Mesh::ConvexDecomposition,
         pybind11::return_value_policy::reference_internal,
         "Get the associated ConvexDecomposition object")
    .def("set_convex_decomposition", &sdf::Mesh::SetConvexDecomposition,
         "Set the associated ConvexDecomposition object.")
    .def("uri", &sdf::Mesh::Uri,
         "Get the mesh's URI.")
    .def("set_uri", &sdf::Mesh::SetUri,
         "Set the mesh's URI.")
    .def("file_path", &sdf::Mesh::FilePath,
         "The path to the file where this element was loaded from.")
    .def("set_file_path", &sdf::Mesh::SetFilePath,
         "Set the path to the file where this element was loaded from.")
    .def("scale", &sdf::Mesh::Scale,
         "Get the mesh's scale factor.")
    .def("set_scale", &sdf::Mesh::SetScale,
         "Set the mesh's scale factor.")
    .def("submesh", &sdf::Mesh::Submesh,
         "A submesh, contained with the mesh at the specified URI, may "
         "optionally be specified. If specified, this submesh should be used "
         "instead of the entire mesh.")
    .def("set_submesh", &sdf::Mesh::SetSubmesh,
         "Set the mesh's submesh. See Submesh() for more information.")
    .def("center_submesh", &sdf::Mesh::CenterSubmesh,
         "Get whether the submesh should be centered at 0,0,0. This will "
         "effectively remove any transformations on the submesh before the "
         "poses from parent links and models are applied. The return value is "
         "only applicable if a SubMesh has been specified.")
    .def("set_center_submesh", &sdf::Mesh::SetCenterSubmesh,
         "Set whether the submesh should be centered. See CenterSubmesh() "
         "for more information.")
    .def("axis_aligned_box", &sdf::Mesh::AxisAlignedBox,
         "Get the axis-aligned box that contains this Mesh.")
    .def("__copy__", [](const sdf::Mesh &self) {
      return sdf::Mesh(self);
    })
    .def("__deepcopy__", [](const sdf::Mesh &self, pybind11::dict) {
      return sdf::Mesh(self);
    }, "memo"_a);

    pybind11::enum_<sdf::MeshOptimization>(module, "MeshOptimization")
      .value("NONE", sdf::MeshOptimization::NONE)
      .value("CONVEX_HULL", sdf::MeshOptimization::CONVEX_HULL)
      .value("CONVEX_DECOMPOSITION", sdf::MeshOptimization::CONVEX_DECOMPOSITION);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
