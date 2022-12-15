/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _SDF_CONVERTER_HH_
#define _SDF_CONVERTER_HH_

#include <tinyxml2.h>

#include <string>
#include <tuple>

#include <sdf/sdf_config.h>
#include <sdf/Types.hh>
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief Convert from one version of SDF to another
  class Converter
  {
    /// \brief Convert SDF to the specified version.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _doc SDF xml doc
    /// \param[in] _toVersion Version number in string format.
    /// \param[in] _config Parser configuration.
    /// \param[in] _quiet False to be more verbose.
    public: static bool Convert(sdf::Errors &_errors,
                                tinyxml2::XMLDocument *_doc,
                                const std::string &_toVersion,
                                const ParserConfig &_config,
                                bool _quiet = false);

    /// \cond
    /// This is an internal function.
    /// \brief Generic convert function that converts the SDF based on the
    /// given Convert file.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _doc SDF xml doc
    /// \param[in] _convertDoc Convert xml doc
    /// \param[in] _config Parser configuration.
    public: static void Convert(sdf::Errors &_errors,
                                tinyxml2::XMLDocument *_doc,
                                tinyxml2::XMLDocument *_convertDoc,
                                const ParserConfig &_config);
    /// \endcond

    /// \brief Implementation of Convert functionality.
    /// \param[in] _elem SDF xml element tree to convert.
    /// \param[in] _convert Convert xml element tree.
    /// \param[in] _config Parser configuration.
    /// \param[out] _errors Vector of errors.
    private: static void ConvertImpl(tinyxml2::XMLElement *_elem,
                                     tinyxml2::XMLElement *_convert,
                                     const ParserConfig &_config,
                                     sdf::Errors &_errors);

    /// \brief Recursive helper function for ConvertImpl that converts
    /// elements named by the descendant_name attribute.
    /// \param[in] _e SDF xml element tree to convert.
    /// \param[in] _c Convert xml element tree.
    /// \param[in] _config Parser configuration.
    /// \param[out] _errors Vector of errors.
    private: static void ConvertDescendantsImpl(tinyxml2::XMLElement *_e,
                                                tinyxml2::XMLElement *_c,
                                                const ParserConfig &_config,
                                                sdf::Errors &_errors);

    /// \brief Rename an element or attribute.
    /// \param[in] _elem The element to be renamed, or the element which
    /// has the attribute to be renamed.
    /// \param[in] _renameElem A 'convert' element that describes the rename.
    /// \param[out] _errors Vector of errors.
    /// operation.
    private: static void Rename(tinyxml2::XMLElement *_elem,
                                tinyxml2::XMLElement *_renameElem,
                                sdf::Errors &_errors);

    /// \brief Map values from one element or attribute to another.
    /// \param[in] _elem Ancestor element of the element or attribute to
    /// be mapped.
    /// \param[in] _mapElem A 'convert' element that describes the map
    /// \param[out] _errors Vector of errors.
    /// operation.
    private: static void Map(tinyxml2::XMLElement *_elem,
                             tinyxml2::XMLElement *_mapElem,
                             sdf::Errors &_errors);

    /// \brief Move an element or attribute within a common ancestor element.
    /// \param[in] _elem Ancestor element of the element or attribute to
    /// be moved.
    /// \param[in] _moveElem A 'convert' element that describes the move
    /// operation.
    /// \param[in] _copy True to copy the element
    /// \param[out] _errors Vector of errors.
    private: static void Move(tinyxml2::XMLElement *_elem,
                              tinyxml2::XMLElement *_moveElem,
                              const bool _copy,
                              sdf::Errors &_errors);

    /// \brief Add an element or attribute to an element.
    /// \param[in] _elem The element to receive the value.
    /// \param[in] _addElem A 'convert' element that describes the add
    /// \param[out] _errors Vector of Errors.
    /// operation.
    private: static void Add(tinyxml2::XMLElement *_elem,
                             tinyxml2::XMLElement *_addElem,
                             sdf::Errors &_errors);

    /// \brief Remove an attribute or elements.
    /// \param[out] _errors Vector of Errors.
    /// \param[in] _elem The element from which data may be removed.
    /// \param[in] _removeElem The metadata about what to remove.
    /// \param[in] _removeOnlyEmpty If true, only remove an attribute
    /// containing an empty string or elements that contain neither value nor
    /// child elements nor attributes.
    private: static void Remove(sdf::Errors &_errors,
                                tinyxml2::XMLElement *_elem,
                                tinyxml2::XMLElement *_removeElem,
                                bool _removeOnlyEmpty = false);

    /// \brief Unflatten an element (conversion from SDFormat <= 1.7 to 1.8)
    /// \param[in] _elem The element to unflatten
    /// \param[out] _errors Vector of errors
    private: static void Unflatten(tinyxml2::XMLElement *_elem,
                                   sdf::Errors &_errors);

    /// \brief Finds all elements related to the unflattened model
    /// \param[in] _elem The element to unflatten
    /// \param[in] _newModel The new unflattened model element
    /// \param[in] _childNameIdx The beginning index of child element names
    /// \param[out] _errors Vector of errors
    /// (e.g., in newModelName::childName then _childNameIdx = 14)
    /// \return True if unflattened new model elements
    private: static bool FindNewModelElements(tinyxml2::XMLElement *_elem,
                                              tinyxml2::XMLElement *_newModel,
                                              const size_t &_childNameIdx,
                                              sdf::Errors &_errors);

    private: static const char *GetValue(const char *_valueElem,
                                         const char *_valueAttr,
                                         tinyxml2::XMLElement *_elem);

    private: static void CheckDeprecation(tinyxml2::XMLElement *_elem,
                                          tinyxml2::XMLElement *_convert,
                                          const ParserConfig &_config,
                                          sdf::Errors &_errors);
  };
  }
}
#endif
