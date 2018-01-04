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
#ifndef SDF_MODEL_HH_
#define SDF_MODEL_HH_

namespace sdf
{
  // Forward declare private data class.
  class ModelPrivate;

  class SDFORMAT_VISIBLE Model
  {
    /// \brief Default constructor
    public: Model();

    /// \brief Move constructor
    /// \param[in] _model Model to move.
    public: Model(Model &&_model);

    /// \brief Destructor
    public: ~Model();

    /// \brief Load the model based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Print debug information to standard out.
    /// \param[in] _prefix String to prefix all output.
    public: void DebugPrint(const std::string &_prefix = "") const;

    /// \brief Get the name of the model.
    /// \return Name of the model.
    public: std::string Name() const;

    /// \brief Set the name of the model.
    /// \param[in] _name Name of the model.
    public: void SetName(const std::string &_name) const;

    /// \brief Private data pointer.
    private: ModelPrivate *dataPtr = nullptr;
  };
}
#endif
