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

#include "pyGeometry.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "sdf/ParserConfig.hh"

#include "sdf/Box.hh"
#include "sdf/Capsule.hh"
#include "sdf/Cone.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Ellipsoid.hh"
#include "sdf/Geometry.hh"
#include "sdf/Heightmap.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Sphere.hh"

using namespace pybind11::literals;

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{
/////////////////////////////////////////////////
void defineGeometry(pybind11::object module)
{
  // TODO(ahcorde) Add HeightmapShape and SetHeightmapShape
  pybind11::class_<sdf::Geometry> geometryModule(module, "Geometry");
  geometryModule
    .def(pybind11::init<>())
    .def(pybind11::init<sdf::Geometry>())
    .def("calculate_inertial", &sdf::Geometry::CalculateInertial,
         "Calculate and return the Mass Matrix values for the Geometry")
    .def("type", &sdf::Geometry::Type,
         "Get the type of geometry.")
    .def("set_type", &sdf::Geometry::SetType,
         "Set the type of geometry.")
    .def("box_shape", &sdf::Geometry::BoxShape,
         pybind11::return_value_policy::reference,
         "Get the box geometry, or None if the contained geometry is "
         "not a box.")
    .def("set_box_shape", &sdf::Geometry::SetBoxShape,
         "Set the box shape.")
    .def("capsule_shape", &sdf::Geometry::CapsuleShape,
         pybind11::return_value_policy::reference,
         "Get the capsule geometry, or None if the contained "
         "geometry is not a capsule.")
    .def("set_capsule_shape", &sdf::Geometry::SetCapsuleShape,
         "Set the capsule shape.")
    .def("cone_shape", &sdf::Geometry::ConeShape,
         pybind11::return_value_policy::reference,
         "Get the cone geometry, or None if the contained "
         "geometry is not a cone.")
    .def("set_cone_shape", &sdf::Geometry::SetConeShape,
         "Set the cone shape.")
    .def("cylinder_shape", &sdf::Geometry::CylinderShape,
         pybind11::return_value_policy::reference,
         "Get the cylinder geometry, or None if the contained "
         "geometry is not a cylinder.")
    .def("set_cylinder_shape", &sdf::Geometry::SetCylinderShape,
         "Set the cylinder shape.")
    .def("ellipsoid_shape", &sdf::Geometry::EllipsoidShape,
         pybind11::return_value_policy::reference,
         "Get the ellipsoid geometry, or None if the contained "
         "geometry is not a ellipsoid.")
    .def("set_ellipsoid_shape", &sdf::Geometry::SetEllipsoidShape,
         "Set the elliposid shape.")
    .def("sphere_shape", &sdf::Geometry::SphereShape,
         pybind11::return_value_policy::reference,
         "Get the sphere geometry, or None if the contained "
         "geometry is not a sphere.")
    .def("set_sphere_shape", &sdf::Geometry::SetSphereShape,
         "Set the sphere shape.")
    .def("plane_shape", &sdf::Geometry::PlaneShape,
         pybind11::return_value_policy::reference,
         "Get the plane geometry, or None if the contained "
         "geometry is not a plane.")
    .def("set_plane_shape", &sdf::Geometry::SetPlaneShape,
         "Set the plane shape.")
    .def("mesh_shape", &sdf::Geometry::MeshShape,
         pybind11::return_value_policy::reference,
         "Get the mesh geometry, or None if the contained "
         "geometry is not a mesh.")
    .def("set_mesh_shape", &sdf::Geometry::SetMeshShape,
         "Set the mesh shape.")
    .def("set_heightmap_shape", &sdf::Geometry::SetHeightmapShape,
         "Set the heightmap shape.")
    .def("heightmap_shape", &sdf::Geometry::HeightmapShape,
          pybind11::return_value_policy::reference,
          "Get the heightmap geometry.")
    .def("axis_aligned_box", &sdf::Geometry::AxisAlignedBox,
         "Get the axis-aligned box that contains the Geometry.")
    .def("__copy__", [](const sdf::Geometry &self) {
      return sdf::Geometry(self);
    })
    .def("__deepcopy__", [](const sdf::Geometry &self, pybind11::dict) {
      return sdf::Geometry(self);
    }, "memo"_a);

    pybind11::enum_<sdf::GeometryType>(module, "GeometryType")
      .value("EMPTY", sdf::GeometryType::EMPTY)
      .value("BOX", sdf::GeometryType::BOX)
      .value("CONE", sdf::GeometryType::CONE)
      .value("CYLINDER", sdf::GeometryType::CYLINDER)
      .value("PLANE", sdf::GeometryType::PLANE)
      .value("SPHERE", sdf::GeometryType::SPHERE)
      .value("MESH", sdf::GeometryType::MESH)
      .value("HEIGHTMAP", sdf::GeometryType::HEIGHTMAP)
      .value("CAPSULE", sdf::GeometryType::CAPSULE)
      .value("ELLIPSOID", sdf::GeometryType::ELLIPSOID);
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
