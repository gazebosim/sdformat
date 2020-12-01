/*
 * Copyright 2020 Open Source Robotics Foundation
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
#ifndef SDF_PARAM_PASSING_HH_
#define SDF_PARAM_PASSING_HH_

#include <tinyxml2.h>
#include <string>

#include "sdf/SDFImpl.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {

    /// \brief Updates the included model (_includeSDF) with the specified
    /// modifications listed under //include/experimental:params
    /// \param[in] _childXmlParams Pointer to //include/experimental:params
    /// children
    /// \param[in,out] _includeSDF The loaded (from include) SDF pointer to
    // parse and update data into
    /// \param[out] _errors Captures errors found during parsing
    SDFORMAT_VISIBLE
    void updateParams(const tinyxml2::XMLElement *_childXmlParams,
                      SDFPtr _includeSDF, Errors &_errors);

    /// \brief Retrieves the specified element by the element identifier
    /// and element name
    /// \param[in] _sdf The loaded (from include) SDF pointer
    /// \param[in] _elemId The element identifier
    /// \param[in] _elemName The element name
    /// \return ElementPtr to the specified element, nullptr if element could
    /// not found 
    SDFORMAT_VISIBLE
    ElementPtr getElementById(const SDFPtr _sdf, const char *_elemId,
                              const char *_elemName);
  }
}
#endif
