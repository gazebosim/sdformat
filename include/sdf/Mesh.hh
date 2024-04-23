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
#include <optional>

#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/CustomInertiaCalcProperties.hh>
#include <sdf/Element.hh>
#include <sdf/Error.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class ParserConfig;

  /// \brief Mesh optimization method
  enum class MeshOptimization
  {
    /// \brief No mesh optimization
    NONE,
    /// \brief Convex hull
    CONVEX_HULL,
    /// \brief Convex decomposition
    CONVEX_DECOMPOSITION
  };

  /// \brief Convex decomposition
  class SDFORMAT_VISIBLE ConvexDecomposition
  {
    /// \brief Default constructor
    public: ConvexDecomposition();

    /// \brief Load the contact based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Get the maximum number of convex hulls that can be generated.
    public: [[nodiscard]] unsigned int MaxConvexHulls() const;

    /// \brief Set the maximum number of convex hulls that can be generated.
    public: void SetMaxConvexHulls(unsigned int _maxConvexHulls);

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Mesh represents a mesh shape, and is usually accessed through a
  /// Geometry.
  class SDFORMAT_VISIBLE Mesh
  {
    /// \brief Constructor
    public: Mesh();

    /// \brief Load the mesh geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the mesh geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \param[in] _config Parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(sdf::ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Get the mesh's optimization method
    /// \return The mesh optimization method.
    /// MeshOptimization::NONE if no mesh simplificaton is done.
    public: [[nodiscard]] MeshOptimization Optimization() const;

    /// \brief Get the mesh's optimization method
    /// \return The mesh optimization method.
    /// Empty string if no mesh simplificaton is done.
    public: [[nodiscard]] std::string OptimizationStr() const;

    /// \brief Set the mesh optimization method.
    /// \param[in] _optimization The mesh optimization method.
    public: void SetOptimization(MeshOptimization _optimization);

    /// \brief Set the mesh optimization method.
    /// \param[in] _optimization The mesh optimization method.
    /// \return True if the _optimizationStr parameter matched a known
    /// mesh optimization method. False if the mesh optimization method
    /// could not be set.
    public: bool SetOptimization(const std::string &_optimizationStr);

    /// \brief Get the associated ConvexDecomposition object
    /// \returns Pointer to the associated ConvexDecomposition object,
    /// nullptr if the Mesh doesn't contain a ConvexDecomposition element.
    public: [[nodiscard]] const sdf::ConvexDecomposition *ConvexDecomposition() const;

    /// \brief Set the associated ConvexDecomposition object.
    /// \param[in] _convexDecomposition The ConvexDecomposition object.
    public: void SetConvexDecomposition(
        const sdf::ConvexDecomposition &_convexDecomposition);

    /// \brief Get the mesh's URI.
    /// \return The URI of the mesh data.
    public: [[nodiscard]] std::string Uri() const;

    /// \brief Set the mesh's URI.
    /// \param[in] _uri The URI of the mesh.
    public: void SetUri(const std::string &_uri);

    /// \brief The path to the file where this element was loaded from.
    /// \return Full path to the file on disk.
    public: [[nodiscard]] const std::string &FilePath() const;

    /// \brief Set the path to the file where this element was loaded from.
    /// \paramp[in] _filePath Full path to the file on disk.
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Get the mesh's scale factor.
    /// \return The mesh's scale factor.
    public: [[nodiscard]] gz::math::Vector3d Scale() const;

    /// \brief Set the mesh's scale factor.
    /// \return The mesh's scale factor.
    public: void SetScale(const gz::math::Vector3d &_scale);

    /// \brief A submesh, contained with the mesh at the specified URI, may
    /// optionally be specified. If specified, this submesh should be used
    /// instead of the entire mesh.
    /// \return The name of the submesh within the mesh at the specified URI.
    public: [[nodiscard]] std::string Submesh() const;

    /// \brief Set the mesh's submesh. See Submesh() for more information.
    /// \param[in] _submesh Name of the submesh. The name should match a submesh
    /// within the mesh at the specified URI.
    public: void SetSubmesh(const std::string &_submesh);

    /// \brief Get whether the submesh should be centered at 0,0,0. This will
    /// effectively remove any transformations on the submesh before the poses
    /// from parent links and models are applied. The return value is only
    /// applicable if a SubMesh has been specified.
    /// \return True if the submesh should be centered.
    public: [[nodiscard]] bool CenterSubmesh() const;

    /// \brief Set whether the submesh should be centered. See CenterSubmesh()
    /// for more information.
    /// \param[in] _center True to center the submesh.
    public: void SetCenterSubmesh(const bool _center);

    /// \brief Calculate and return the Inertial values for the Mesh
    /// \param[out] _errors A vector of Errors object. Each object
    /// would contain an error code and an error message.
    /// \param[in] _density Density of the mesh in kg/m^3
    /// \param[in] _autoInertiaParams ElementPtr to
    /// <auto_inertia_params> element
    /// \param[in] _config Parser Configuration
    /// \return A std::optional with gz::math::Inertiald object or std::nullopt
    public: std::optional<gz::math::Inertiald>
            CalculateInertial(sdf::Errors &_errors,
                              double _density,
                              const sdf::ElementPtr _autoInertiaParams,
                              const ParserConfig &_config);

    /// \brief Get a pointer to the SDF element that was used during load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// mesh.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated mesh values.
    public: [[nodiscard]] sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// mesh.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated mesh values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
