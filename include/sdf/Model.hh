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

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class Joint;
  class Link;
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

    /// \brief Get the name of the model.
    /// The name of the model should be unique within the scope of a World.
    /// \return Name of the model.
    public: std::string Name() const;

    /// \brief Set the name of the model.
    /// The name of the model should be unique within the scope of a World.
    /// \param[in] _name Name of the model.
    public: void SetName(const std::string &_name) const;

    /// \brief Get the number of links.
    /// \return Number of links contained in this Model object.
    public: uint64_t LinkCount() const;

    /// \brief Get a link based on an index.
    /// \param[in] _index Index of the link. The index should be in the
    /// range [0..LinkCount()).
    /// \return Pointer to the link. Nullptr if the index does not exist.
    /// \sa uint64_t LinkCount() const
    public: const Link *LinkByIndex(const uint64_t _index) const;

    /// \brief Get whether a link name exists.
    /// \param[in] _name Name of the link to check.
    /// \return True if there exists a link with the given name.
    public: bool LinkNameExists(const std::string &_name) const;

    /// \brief Get the number of joints.
    /// \return Number of joints contained in this Model object.
    public: uint64_t JointCount() const;

    /// \brief Get a joint based on an index.
    /// \param[in] _index Index of the joint. The index should be in the
    /// range [0..JointCount()).
    /// \return Pointer to the joint. Nullptr if the index does not exist.
    /// \sa uint64_t JointCount() const
    public: const Joint *JointByIndex(const uint64_t _index) const;

    /// \brief Get whether a joint name exists.
    /// \param[in] _name Name of the joint to check.
    /// \return True if there exists a joint with the given name.
    public: bool JointNameExists(const std::string &_name) const;

    /// \brief Private data pointer.
    private: ModelPrivate *dataPtr = nullptr;
  };
}
#endif
