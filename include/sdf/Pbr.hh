/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_PBR_HH_
#define SDF_PBR_HH_

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declarations.
  class PbrPrivate;
  class PbrWorkflowPrivate;

  /// \brief Type of PBR workflow.
  enum class PbrWorkflowType : int
  {
    /// \brief No workflow
    NONE = 0,

    /// \brief Metal/Roughness workflow
    METAL = 1,

    /// \brief Specular/Glossiness workflow
    SPECULAR = 2,
  };

  /// \brief Space the normal map is defined in
  enum class NormalMapSpace: int
  {
    /// \brief Tangent space
    TANGENT = 0,

    /// \brief Object space
    OBJECT = 1,
  };

  /// \brief This class contains Physically-Based-Rendering (PBR) workflow
  /// properties.
  class SDFORMAT_VISIBLE PbrWorkflow
  {
    /// \brief Default constructor
    public: PbrWorkflow();

    /// \brief Copy constructor
    /// \param[in] _workflow Workflow to copy.
    public: PbrWorkflow(const PbrWorkflow &_workflow);

    /// \brief Move constructor
    /// \param[in] _workflow to move.
    public: PbrWorkflow(PbrWorkflow &&_workflow) noexcept;

    /// \brief Destructor
    public: ~PbrWorkflow();

    /// \brief Load the pbr workflow based on an element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Assignment operator.
    /// \param[in] _workflow The workflow to set values from.
    /// \return *this
    public: PbrWorkflow &operator=(const PbrWorkflow &_workflow);

    /// \brief Move assignment operator.
    /// \param[in] _workflow The workflow to move from.
    /// \return *this
    public: PbrWorkflow &operator=(PbrWorkflow &&_workflow);

    /// \brief Return true if both PbrWorkflow objects contain the same values.
    /// \param[_in] _workflow PbrWorkflow value to compare.
    /// \returen True if 'this' == _workflow.
    public: bool operator==(const PbrWorkflow &_workflow) const;

    /// \brief Return true this PbrWorkflow object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _workflow PbrWorkflow value to compare.
    /// \returen True if 'this' != _workflow.
    public: bool operator!=(const PbrWorkflow &_workflow) const;

    /// \brief Get the albedo map filename. This will be an empty string if
    /// an albedo map has not been set.
    /// \return Filename of the albedo map, or empty string if an albedo map
    /// has not been specified.
    public: std::string AlbedoMap() const;

    /// \brief Set the albedo map filename.
    /// \param[in] _map Filename of the albedo map.
    public: void SetAlbedoMap(const std::string &_map);

    /// \brief Get the normal map filename. This will be an empty string if
    /// a normal map has not been set.
    /// \return Filename of the normal map, or empty string if a normal map
    /// has not been specified.
    public: std::string NormalMap() const;

    /// \brief Set the normal map filename.
    /// \param[in] _map Filename of the normal map.
    /// \param[in] _space Space that the normal map is defined in.
    /// Defaults to tangent space.
    public: void SetNormalMap(const std::string &_map,
        NormalMapSpace _space = NormalMapSpace::TANGENT);

    /// \brief Get the normal map type, either tangent or object space
    /// \return Space that the normal map is defined in
    public: NormalMapSpace NormalMapType() const;

    /// \brief Get the environment map filename. This will be an empty string
    /// if an environment map has not been set.
    /// \return Filename of the environment map, or empty string if an
    /// environment map has not been specified.
    public: std::string EnvironmentMap() const;

    /// \brief Set the environment map filename.
    /// \param[in] _map Filename of the environment map.
    public: void SetEnvironmentMap(const std::string &_map);

    /// \brief Get the ambient occlusion map filename. This will be an empty
    /// string if an ambient occlusion map has not been set.
    /// \return Filename of the ambient occlusion map, or empty string if an
    /// ambient occlusion map has not been specified.
    public: std::string AmbientOcclusionMap() const;

    /// \brief Set the ambient occlusion map filename.
    /// \param[in] _map Filename of the ambient occlusion map.
    public: void SetAmbientOcclusionMap(const std::string &_map);

    /// \brief Get the roughness map filename for metal workflow. This will be
    /// an empty string if a roughness map has not been set.
    /// \return Filename of the roughness map, or empty string if a roughness
    /// map has not been specified.
    public: std::string RoughnessMap() const;

    /// \brief Set the roughness map filename for metal workflow.
    /// \param[in] _map Filename of the roughness map.
    public: void SetRoughnessMap(const std::string &_map);

    /// \brief Get the metalness map filename for metal workflow. This will be
    /// an empty string if a metalness map has not been set.
    /// \return Filename of the metalness map, or empty string if a metalness
    /// map has not been specified.
    public: std::string MetalnessMap() const;

    /// \brief Set the metalness map filename for metal workflow.
    /// \param[in] _map Filename of the metalness map.
    public: void SetMetalnessMap(const std::string &_map);

    /// \brief Get the emissive map filename. This will be an empty string
    /// if an emissive map has not been set.
    /// \return Filename of the emissive map, or empty string if an emissive
    /// map has not been specified.
    public: std::string EmissiveMap() const;

    /// \brief Set the emissive map filename.
    /// \param[in] _map Filename of the emissive map.
    public: void SetEmissiveMap(const std::string &_map);

    /// \brief Get the light map filename. This will be an empty string
    /// if an light map has not been set.
    /// \return Filename of the light map, or empty string if a light
    /// map has not been specified.
    public: std::string LightMap() const;

    /// \brief Set the light map filename.
    /// \param[in] _map Filename of the light map.
    /// \param[in] _uvSet Index of the light map texture coordinate set
    public: void SetLightMap(const std::string &_map, unsigned int _uvSet = 0u);

    /// \brief Get the light map texture coordinate set.
    /// \return Index of the texture coordinate set
    public: unsigned int LightMapTexCoordSet() const;

    /// \brief Get the metalness value of the material for metal workflow
    /// \return metalness value of the material
    public: double Metalness() const;

    /// \brief Set the metalness value of the material for metal workflow.
    /// \param[in] _value Metalness value.
    public: void SetMetalness(const double _value);

    /// \brief Get the roughness value of the material for metal workflow
    /// \return roughness value of the material
    public: double Roughness() const;

    /// \brief Set the roughness value of the material for metal workflow.
    /// \param[in] _value Roughness value.
    public: void SetRoughness(const double _value);

    /// \brief Get the glossiness map filename for specular workflow. This will
    /// be an empty string if a glossiness map has not been set.
    /// \return Filename of the glossiness map, or empty string if a glossiness
    /// map has not been specified.
    public: std::string GlossinessMap() const;

    /// \brief Set the glossiness map filename for specular workflow.
    /// \param[in] _map Filename of the glossiness map.
    public: void SetGlossinessMap(const std::string &_map);

    /// \brief Get the glossiness value of the material for specular workflow
    /// \return Glossiness value of the material
    public: double Glossiness() const;

    /// \brief Set the glossiness value of the material for specular workflow.
    /// \param[in] _value Glossiness value.
    public: void SetGlossiness(const double _value);

    /// \brief Get the specular map filename for specular workflow. This will
    /// be an empty string if a specular map has not been set.
    /// \return Filename of the specular map, or empty string if a specular
    /// map has not been specified.
    public: std::string SpecularMap() const;

    /// \brief Set the specular map filename for specular workflow.
    /// \param[in] _map Filename of the specular map.
    public: void SetSpecularMap(const std::string &_map);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the workflow type.
    /// \return PBR workflow type.
    public: PbrWorkflowType Type() const;

    /// \brief Set the PBR workflow to use
    /// \param[in] _type Workflow type to use
    public: void SetType(PbrWorkflowType _type);

    /// \brief Private data pointer.
    private: PbrWorkflowPrivate *dataPtr = nullptr;
  };

  /// \brief This class provides access to Physically-Based-Rendering (PBR)
  ///  material workflows.
  class SDFORMAT_VISIBLE Pbr
  {
    /// \brief Default constructor
    public: Pbr();

    /// \brief Copy constructor
    /// \param[in] _pbr Pbr to copy.
    public: Pbr(const Pbr &_pbr);

    /// \brief Move constructor
    /// \param[in] _pbr Pbr to move.
    public: Pbr(Pbr &&_pbr) noexcept;

    /// \brief Destructor
    public: ~Pbr();

    /// \brief Assignment operator.
    /// \param[in] _pbr The pbr to set values from.
    /// \return *this
    public: Pbr &operator=(const Pbr &_pbr);

    /// \brief Move assignment operator.
    /// \param[in] _pbr The pbr to move values from.
    /// \return *this
    public: Pbr &operator=(Pbr &&_pbr);

    /// \brief Load the pbr based on an element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Set a PBR workflow by type
    /// \param[in] _type Type of PBR workflow
    /// \param[in] _workflow Workflow of the specified type.
    /// \sa PbrWorkflowType
    public: void SetWorkflow(PbrWorkflowType _type,
        const PbrWorkflow &_workflow);

    /// \brief Get a PBR workflow by type
    /// \param[in] _type Type of PBR workflow
    /// \return Workflow of the specified type.
    /// \sa PbrWorkflowType
    public: PbrWorkflow *Workflow(PbrWorkflowType _type) const;

    /// \brief Private data pointer.
    private: PbrPrivate *dataPtr = nullptr;
  };
  }
}
#endif
