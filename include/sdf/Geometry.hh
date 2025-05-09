/*
 * Copyright 2018 Open Source Robotics Foundation
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
 *
 */
#ifndef SDF_GEOMETRY_HH_
#define SDF_GEOMETRY_HH_

#include <vector>
#include <optional>

#include <gz/utils/ImplPtr.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/config.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Types.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class Box;
  class Capsule;
  class Cone;
  class Cylinder;
  class Ellipsoid;
  class Heightmap;
  class Mesh;
  class ParserConfig;
  class Plane;
  class Polyline;
  class Sphere;

  /// \enum GeometryType
  /// \brief The set of geometry types.
  enum class GeometryType
  {
    /// \brief Empty geometry. This means no shape has been defined.
    EMPTY = 0,

    /// \brief A box geometry.
    BOX = 1,

    /// \brief A cylinder geometry.
    CYLINDER = 2,

    /// \brief A plane geometry.
    PLANE = 3,

    /// \brief A sphere geometry.
    SPHERE = 4,

    /// \brief A mesh geometry.
    MESH = 5,

    /// \brief A heightmap geometry.
    HEIGHTMAP = 6,

    /// \brief A capsule geometry.
    CAPSULE = 7,

    /// \brief An ellipsoid geometry
    ELLIPSOID = 8,

    /// \brief A polyline geometry.
    POLYLINE = 9,

    /// \brief A polyline geometry.
    CONE = 10,
  };

  /// \brief Geometry provides access to a shape, such as a Box. Use the
  /// Type function to determine the type of shape contained within a
  /// Geometry. Access to shape data, such as a box's size, is achieved
  /// through the shape accessors, such as const Box *BoxShape() const.
  class SDFORMAT_VISIBLE Geometry
  {
    /// \brief Default constructor
    public: Geometry();

    /// \brief Load the geometry based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the geometry based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \param[in] _config Parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(sdf::ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Get the type of geometry.
    /// \return The geometry type.
    public: GeometryType Type() const;

    /// \brief Set the type of geometry.
    /// \param[in] _type The geometry type.
    public: void SetType(const GeometryType _type);

    /// \brief Get the box geometry, or nullptr if the contained geometry is
    /// not a box.
    /// \return Pointer to the visual's box geometry, or nullptr if the
    /// geometry is not a box.
    /// \sa GeometryType Type() const
    public: const Box *BoxShape() const;

    /// \brief Set the box shape.
    /// \param[in] _box The box shape.
    public: void SetBoxShape(const Box &_box);

    /// \brief Get the capsule geometry, or nullptr if the contained
    /// geometry is not a capsule.
    /// \return Pointer to the capsule geometry, or nullptr if the
    /// geometry is not a capsule.
    /// \sa GeometryType Type() const
    public: const Capsule *CapsuleShape() const;

    /// \brief Set the capsule shape.
    /// \param[in] _capsule The capsule shape.
    public: void SetCapsuleShape(const Capsule &_capsule);

    /// \brief Get the cone geometry, or nullptr if the contained
    /// geometry is not a cone.
    /// \return Pointer to the visual's cone geometry, or nullptr if the
    /// geometry is not a cone.
    /// \sa GeometryType Type() const
    public: const Cone *ConeShape() const;

    /// \brief Set the cone shape.
    /// \param[in] _cone The cone shape.
    public: void SetConeShape(const Cone &_cone);

    /// \brief Get the cylinder geometry, or nullptr if the contained
    /// geometry is not a cylinder.
    /// \return Pointer to the visual's cylinder geometry, or nullptr if the
    /// geometry is not a cylinder.
    /// \sa GeometryType Type() const
    public: const Cylinder *CylinderShape() const;

    /// \brief Set the cylinder shape.
    /// \param[in] _cylinder The cylinder shape.
    public: void SetCylinderShape(const Cylinder &_cylinder);

    /// \brief Get the ellipsoid geometry, or nullptr if the contained
    /// geometry is not an ellipsoid.
    /// \return Pointer to the ellipsoid geometry, or nullptr if the geometry is
    /// not an ellipsoid.
    /// \sa GeometryType Type() const
    public: const Ellipsoid *EllipsoidShape() const;

    /// \brief Set the ellipsoid shape.
    /// \param[in] _ellipsoid The ellipsoid shape.
    public: void SetEllipsoidShape(const Ellipsoid &_ellipsoid);

    /// \brief Get the sphere geometry, or nullptr if the contained geometry is
    /// not a sphere.
    /// \return Pointer to the visual's sphere geometry, or nullptr if the
    /// geometry is not a sphere.
    /// \sa GeometryType Type() const
    public: const Sphere *SphereShape() const;

    /// \brief Set the sphere shape.
    /// \param[in] _sphere The sphere shape.
    public: void SetSphereShape(const Sphere &_sphere);

    /// \brief Get the polyline geometry. Vector is empty if the contained
    /// geometry is not a polyline.
    /// \return The visual's polyline geometries.
    /// \sa GeometryType Type() const
    public: const std::vector<Polyline> &PolylineShape() const;

    /// \brief Set the polyline shape.
    /// \param[in] _polyline The polyline shape.
    public: void SetPolylineShape(const std::vector<Polyline> &_polyline);

    /// \brief Get the plane geometry, or nullptr if the contained geometry is
    /// not a plane.
    /// \return Pointer to the visual's plane geometry, or nullptr if the
    /// geometry is not a plane.
    /// \sa GeometryType Type() const
    public: const Plane *PlaneShape() const;

    /// \brief Set the plane shape.
    /// \param[in] _plane The plane shape.
    public: void SetPlaneShape(const Plane &_plane);

    /// \brief Get the mesh geometry, or nullptr if the contained geometry is
    /// not a mesh.
    /// \return Pointer to the visual's mesh geometry, or nullptr if the
    /// geometry is not a mesh.
    /// \sa GeometryType Type() const
    public: const Mesh *MeshShape() const;

    /// \brief Set the mesh shape.
    /// \param[in] _mesh The mesh shape.
    public: void SetMeshShape(const Mesh &_mesh);

    /// \brief Get the heightmap geometry, or nullptr if the contained geometry
    /// is not a heightmap.
    /// \return Pointer to the heightmap geometry, or nullptr if the geometry is
    /// not a heightmap.
    /// \sa GeometryType Type() const
    public: const Heightmap *HeightmapShape() const;

    /// \brief Set the heightmap shape.
    /// \param[in] _heightmap The heightmap shape.
    public: void SetHeightmapShape(const Heightmap &_heightmap);

    /// \brief Calculate and return the Inertial values for the Geometry
    /// \param[out] _errors A vector of Errors object. Each object
    /// would contain an error code and an error message.
    /// \param[in] _config Parser Config
    /// \param[in] _density The density of the geometry element.
    /// \param[in] _autoInertiaParams ElementPtr to <auto_inertia_params>
    /// \return A std::optional with gz::math::Inertiald object or std::nullopt
    public: std::optional<gz::math::Inertiald> CalculateInertial(
      sdf::Errors &_errors, const ParserConfig &_config,
      double _density, sdf::ElementPtr _autoInertiaParams);

    /// \brief Calculate and return the AxisAlignedBox for the Geometry
    /// \param _meshAabbCalculator The function to calculate the AABB of a mesh
    /// \return std::optional with gz::math::AxisAlignedBox object or
    /// std::nullopt if the geometry type does not support AABB calculation
    public: std::optional<gz::math::AxisAlignedBox> AxisAlignedBox(
      const Mesh::AxisAlignedBoxCalculator &_meshAabbCalculator) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// geometry.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated geometry values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// geometry.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated geometry values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
