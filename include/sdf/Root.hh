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

#include "sdf/SDFImpl.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class Model;
  class RootPrivate;
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

    /// \brief Destructor
    public: ~Root();

    /// \brief Parse the given SDF file, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _filename Name of the SDF file to parse.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(const std::string &_filename);

    /// \brief Parse the given SDF string, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _sdf SDF string to parse.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors LoadSdfString(const std::string &_sdf);

    /// \brief Parse the given SDF pointer, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _sdf SDF pointer to parse.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(const SDFPtr _sdf);

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

    /// \brief Get the number of models.
    /// \return Number of models contained in this Root object.
    public: uint64_t ModelCount() const;

    /// \brief Get a model based on an index.
    /// \param[in] _index Index of the model. The index should be in the
    /// range [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: const Model *ModelByIndex(const uint64_t _index) const;

    /// \brief Get whether a model name exists.
    /// \param[in] _name Name of the model to check.
    /// \return True if there exists a model with the given name.
    public: bool ModelNameExists(const std::string &_name) const;

    /// \brief Get a pointer to the SDF element that was generated during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer
    private: RootPrivate *dataPtr = nullptr;
  };
}
#endif
