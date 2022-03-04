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

    /// \brief Get a mutable world based on an index.
    /// \param[in] _index Index of the world. The index should be in the
    /// range [0..WorldCount()).
    /// \return Pointer to the world. Nullptr if the index does not exist.
    /// \sa uint64_t WorldCount() const
    public: World *WorldByIndex(const uint64_t _index);

    /// \brief Get whether a world name exists.
    /// \param[in] _name Name of the world to check.
    /// \return True if there exists a world with the given name.
    public: bool WorldNameExists(const std::string &_name) const;

    /// \brief Get a pointer to the model object if it exists.
    ///
    /// \return A pointer to the model, nullptr if it doesn't exist
    public: const sdf::Model *Model() const;

    /// \brief Set the model object. This will override any existing model,
    /// actor, and light object.
    /// \param[in] _model The model to use.
    public: void SetModel(const sdf::Model &_model);

    /// \brief Get a pointer to the light object if it exists.
    ///
    /// \return A pointer to the light, nullptr if it doesn't exist
    public: const sdf::Light *Light() const;

    /// \brief Set the light object. This will override any existing model,
    /// actor, and light object.
    /// \param[in] _light The light to use.
    public: void SetLight(const sdf::Light &_light);

    /// \brief Get a pointer to the actor object if it exists.
    ///
    /// \return A pointer to the actor, nullptr if it doesn't exist
    public: const sdf::Actor *Actor() const;

    /// \brief Set the actor object. This will override any existing model,
    /// actor, and light object.
    /// \param[in] _actor The actor to use.
    public: void SetActor(const sdf::Actor &_actor);

    /// \brief Get a pointer to the SDF element that was generated during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Add a world to the root.
    /// \param[in] _word World to add.
    /// \return True if successful, false if a world with the name already
    /// exists.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors AddWorld(const World &_world);

    /// \brief Remove all worlds.
    public: void ClearWorlds();

    /// \brief Deep copy this Root object and return the new Root object.
    /// \return A clone of this Root object.
    /// Deprecate this function in SDF version 13, and use
    /// IGN_UTILS_IMPL_PTR instead.
    public: sdf::Root Clone() const;

    /// \brief Recreate the frame and pose graphs for the worlds and model
    /// that are children of this Root object. You can call this function
    /// to build new graphs when the DOM was created programmatically, or
    /// if you want to regenerate the graphs after editing the DOM.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors UpdateGraphs();

    /// \brief Create and return an SDF element filled with data from this
    /// root.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[in] _useIncludeTag This will pass the _useIncludeTag to
    /// sdf::Model::ToElement.
    /// \return SDF element pointer with updated root values.
    public: sdf::ElementPtr ToElement(bool _useIncludeTag = true) const;

    /// \brief Private data pointer
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
  }
}
#endif
