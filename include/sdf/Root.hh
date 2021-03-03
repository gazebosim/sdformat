/*
 * Copyright 2017 Open Source Robotics Foundation
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
#ifndef SDF_ROOT_HH_
#define SDF_ROOT_HH_

#include <string>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/SDFImpl.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class Actor;
  class Light;
  class Model;
  class World;

  /// \brief Root class that acts as an entry point to the SDF document
  /// model.
  ///
  /// Multiple worlds can exist in a single SDF file. A user of multiple
  /// worlds could run parallel instances of simulation, or offer selection
  /// of a world at runtime.
  ///
  /// # Usage
  ///
  /// In this example, a root object is loaded from a file specified in
  /// the first command line argument to a program.
  ///
  /// \snippet examples/dom.cc rootUsage
  ///
  class SDFORMAT_VISIBLE Root
  {
    /// \brief Default constructor
    public: Root();

    /// \brief Parse the given SDF file, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _filename Name of the SDF file to parse.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(const std::string &_filename);

    /// \brief Parse the given SDF file, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _filename Name of the SDF file to parse.
    /// \param[in] _config Custom parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(
                const std::string &_filename, const ParserConfig &_config);

    /// \brief Parse the given SDF string, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _sdf SDF string to parse.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors LoadSdfString(const std::string &_sdf);

    /// \brief Parse the given SDF string, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _sdf SDF string to parse.
    /// \param[in] _config Custom parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors LoadSdfString(
                const std::string &_sdf, const ParserConfig &_config);

    /// \brief Parse the given SDF pointer, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _sdf SDF pointer to parse.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(const SDFPtr _sdf);

    /// \brief Parse the given SDF pointer, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _sdf SDF pointer to parse.
    /// \param[in] _config Custom parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(const SDFPtr _sdf, const ParserConfig &_config);

    /// \brief Get the SDF version specified in the parsed file or SDF
    /// pointer.
    /// \return SDF version string.
    /// \sa void SetVersion(const std::string &_version)
    public: std::string Version() const;

    /// \brief Set the SDF version string.
    /// \param[in] _version The new SDF version.
    /// \sa std::string Version()
    public: void SetVersion(const std::string &_version);

    /// \brief Get the number of worlds.
    /// \return Number of worlds contained in this Root object.
    public: uint64_t WorldCount() const;

    /// \brief Get a world based on an index.
    /// \param[in] _index Index of the world. The index should be in the
    /// range [0..WorldCount()).
    /// \return Pointer to the world. Nullptr if the index does not exist.
    /// \sa uint64_t WorldCount() const
    public: const World *WorldByIndex(const uint64_t _index) const;

    /// \brief Get whether a world name exists.
    /// \param[in] _name Name of the world to check.
    /// \return True if there exists a world with the given name.
    public: bool WorldNameExists(const std::string &_name) const;

    /// \brief Get the number of models that are immediate (not nested) children
    /// of this Root object.
    /// \return Number of models contained in this Root object.
    public: uint64_t ModelCount() const SDF_DEPRECATED(11.0);

    /// \brief Get a model based on an index.
    /// \param[in] _index Index of the model. The index should be in the
    /// range [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: const sdf::Model *ModelByIndex(const uint64_t _index) const
        SDF_DEPRECATED(11.0);

    /// \brief Get whether a model name exists.
    /// \param[in] _name Name of the model to check.
    /// \return True if there exists a model with the given name.
    public: bool ModelNameExists(const std::string &_name) const
        SDF_DEPRECATED(11.0);

    /// \brief Get a pointer to the model object if it exists.
    ///
    /// If there is more than one model, this will return the first element.
    /// This method is preferred to ModelByIndex, as its behavior is
    /// consistent with the planned future API. Having more than one Model, or
    /// more than one of Model/Actor/Light is now considered deprecated and
    /// should not be relied upon going forward.
    ///
    /// \return A pointer to the model, nullptr if it doesn't exist
    public: const sdf::Model *Model() const;

    /// \brief Get the number of lights.
    /// \return Number of lights contained in this Root object.
    public: uint64_t LightCount() const SDF_DEPRECATED(11.0);

    /// \brief Get a light based on an index.
    /// \param[in] _index Index of the light. The index should be in the
    /// range [0..LightCount()).
    /// \return Pointer to the light. Nullptr if the index does not exist.
    /// \sa uint64_t LightCount() const
    public: const sdf::Light *LightByIndex(const uint64_t _index) const
        SDF_DEPRECATED(11.0);

    /// \brief Get whether a light name exists.
    /// \param[in] _name Name of the light to check.
    /// \return True if there exists a light with the given name.
    public: bool LightNameExists(const std::string &_name) const
        SDF_DEPRECATED(11.0);

    /// \brief Get a pointer to the light object if it exists.
    ///
    /// If there is more than one light, this will return the first element. If
    /// there is already a model element, this will return null.
    /// This method is preferred to LightByIndex, as its behavior is
    /// consistent with the planned future API. Having more than one Light, or
    /// more than one of Model/Actor/Light is now considered deprecated and
    /// should not be relied upon going forward.
    ///
    /// \return A pointer to the light, nullptr if it doesn't exist
    public: const sdf::Light *Light() const;

    /// \brief Get the number of actors.
    /// \return Number of actors contained in this Root object.
    public: uint64_t ActorCount() const SDF_DEPRECATED(11.0);

    /// \brief Get an actor based on an index.
    /// \param[in] _index Index of the actor. The actor should be in the
    /// range [0..ActorCount()).
    /// \return Pointer to the actor. Nullptr if the index does not exist.
    /// \sa uint64_t ActorCount() const
    public: const sdf::Actor *ActorByIndex(const uint64_t _index) const
        SDF_DEPRECATED(11.0);

    /// \brief Get whether an actor name exists.
    /// \param[in] _name Name of the actor to check.
    /// \return True if there exists an actor with the given name.
    public: bool ActorNameExists(const std::string &_name) const
        SDF_DEPRECATED(11.0);

    /// \brief Get a pointer to the actor object if it exists.
    ///
    /// If there is more than one actor, this will return the first element. If
    /// there is already a model or light element, this will return null.
    /// This method is preferred to ActorByIndex, as its behavior is
    /// consistent with the planned future API. Having more than one Actor, or
    /// more than one of Model/Actor/Light is now considered deprecated and
    /// should not be relied upon going forward.
    ///
    /// \return A pointer to the actor, nullptr if it doesn't exist
    public: const sdf::Actor *Actor() const;

    /// \brief Get a pointer to the SDF element that was generated during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
#endif
