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
#ifndef SDF_MESH_HH_
#define SDF_MESH_HH_

#include <string>
#include <ignition/math/Vector3.hh>
#include <sdf/Element.hh>
#include <sdf/Error.hh>

namespace sdf
{
  // Forward declare private data class.
  class MeshPrivate;

  /// \brief Mesh represents a mesh shape, and is usually accessed through a
  /// Geometry.
  class SDFORMAT_VISIBLE Mesh
  {
    /// \brief Constructor
    public: Mesh();

    /// \brief Destructor
    public: virtual ~Mesh();

    /// \brief Load the mesh geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the mesh's URI.
    /// \return The URI of the mesh data.
    public: std::string Uri() const;

    /// \brief Set the mesh's URI.
    /// \param[in] _uri The URI of the mesh.
    public: void SetUri(const std::string &_uri);

    /// \brief Get the mesh's scale factor.
    /// \return The mesh's scale factor.
    public: ignition::math::Vector3d Scale() const;

    /// \brief Set the mesh's scale factor.
    /// \return The mesh's scale factor.
    public: void SetScale(const ignition::math::Vector3d &_scale);

    /// \brief A submesh, contained with the mesh at the specified URI, may
    /// optionally be specified. If specified, this submesh should be used
    /// instead of the entire mesh.
    /// \return The name of the submesh within the mesh at the specified URI.
    public: std::string Submesh() const;

    /// \brief Set the mesh's submesh. See Submesh() for more information.
    /// \param[in] _submesh Name of the submesh. The name should match a submesh
    /// within the mesh at the specified URI.
    public: void SetSubmesh(const std::string &_submesh);

    /// \brief Get whether the submesh should be centered at 0,0,0. This will
    /// effectively remove any transformations on the submesh before the poses
    /// from parent links and models are applied. The return value is only
    /// applicable if a SubMesh has been specified.
    /// \return True if the submesh should be centered.
    public: bool CenterSubmesh() const;

    /// \brief Set whether the submesh should be centered. See CenterSubmesh()
    /// for more information.
    /// \param[in] _center True to center the submesh.
    public: void SetCenterSubmesh(const bool _center);

    /// \brief Get a pointer to the SDF element that was used during load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: MeshPrivate *dataPtr;
  };
}
#endif
