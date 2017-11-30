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
#ifndef SDF_DOM_LINK_HH_
#define SDF_DOM_LINK_HH_

#include <string>

#include "sdf/dom/Entity.hh"
#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class LinkPrivate;

  /// \brief A physical link with inertia, collision, and visual properties.
  /// A link must be a child of a model, and any number of links may exist in
  /// a model.
  class SDFORMAT_VISIBLE Link : public Entity
  {
    /// \brief Constructor
    public: Link();

    /// \brief Destructor
    public: ~Link();

    /// \brief Load the link based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf An SDF Element pointer to a <link> element.
    /// \return True when no errors were encountered.
    public: bool Load(sdf::ElementPtr _sdf) override;

    /// \brief Print debug information to standard out.
    /// \param[in] _prefix String to prefix all output.
    public: void Print(const std::string &_prefix = "") const;

    /// \brief Private data pointer
    private: LinkPrivate *dataPtr;
  };
}
#endif
