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
#ifndef SDF_COLLISION_HH_
#define SDF_COLLISION_HH_

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class CollisionPrivate;

  class SDFORMAT_VISIBLE Collision
  {
    /// \brief Default constructor
    public: Collision();

    /// \brief Move constructor
    /// \param[in] _collision Collision to move.
    public: Collision(Collision &&_collision);

    /// \brief Destructor
    public: ~Collision();

    /// \brief Load the collision based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf,
                std::shared_ptr<FrameGraph> _frameGraph);

    /// \brief Get the name of the collision.
    /// The name of the collision must be unique within the scope of a Link.
    /// \return Name of the collision.
    public: std::string Name() const;

    /// \brief Set the name of the collision.
    /// The name of the collision must be unique within the scope of a Link.
    /// \param[in] _name Name of the collision.
    public: void SetName(const std::string &_name) const;

    /// \brief Private data pointer.
    private: CollisionPrivate *dataPtr = nullptr;
  };
}
#endif
