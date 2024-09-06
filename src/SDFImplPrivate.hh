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
#ifndef _SDFIMPLPRIVATE_HH_
#define _SDFIMPLPRIVATE_HH_

#include <string>

#include "sdf/Types.hh"

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief Private data for base SDF class
  class SDFPrivate
  {
    public: SDFPrivate() : root(new Element)
    {
    }

    public: SDFPrivate(const SDFPrivate& other)
    {
      this->path = other.path;
      this->originalVersion = other.originalVersion;
      this->root = other.root->Clone();
    }

    /// \brief Store the root element.
    /// \sa ElementPtr Root()
    /// \sa void Root(const ElementPtr _root)
    public: ElementPtr root;

    /// \brief Path to SDF document on disk.
    public: std::string path;

    /// \brief Spec version that this was originally parsed from.
    public: std::string originalVersion;
  };
  /// \}
}
}
#endif
