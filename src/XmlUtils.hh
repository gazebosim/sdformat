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
#ifndef SDFORMAT_XMLUTILS_HH
#define SDFORMAT_XMLUTILS_HH

#include <string>
#include <tinyxml2.h>

#include "sdf/Error.hh"
#include "sdf/Element.hh"
#include "sdf/Types.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {

  /// \brief Perform a deep copy of an XML Node
  ///
  /// This copies an XMLNode _src and all of its decendants
  /// into a specified XMLDocument.
  ///
  /// \param[in] _doc Document in which to place the copied node
  /// \param[in] _src The node to deep copy
  /// \returns The newly copied node upon success OR
  ///          nullptr if an error occurs.
  tinyxml2::XMLNode *DeepClone(tinyxml2::XMLDocument *_doc,
                               const tinyxml2::XMLNode *_src);

  /// \brief Converts the XML Element to a string
  /// \param[in] _elem Element to be converted
  /// \return The string representation
  std::string ElementToString(const tinyxml2::XMLElement *_elem);
  }
}
#endif
