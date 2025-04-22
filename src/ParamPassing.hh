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

#include "sdf/ParserConfig.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {

  namespace ParamPassing {

    /// \brief Updates the included model (_includeSDF) with the specified
    /// modifications listed under //include/experimental:params
    /// \param[in] _config Custom parser configuration
    /// \param[in] _source Source of the XML document, empty if it came from a
    /// string.
    /// \param[in] _childXmlParams Pointer to //include/experimental:params
    /// children
    /// \param[in,out] _includeSDF The loaded (from include) SDF pointer to
    // parse and update data into
    /// \param[out] _errors Captures errors found during parsing
    void updateParams(const ParserConfig &_config,
                      const std::string &_source,
                      tinyxml2::XMLElement *_childXmlParams,
                      ElementPtr _includeSDF,
                      Errors &_errors);

    /// \brief Retrieves the specified element by the element identifier
    /// and element name
    /// \param[in] _sdf The loaded (from include) SDF pointer
    /// \param[in] _elemName The element name, such as "model", "link",
    /// "collision", "visual".
    /// \param[in] _elemId The element identifier
    /// \param[in] _isParentElement Is true if _elemId is the parent and does
    /// not use _elemName to verify the correct element is found. Only used for
    /// the add action
    /// \return ElementPtr to the specified element, nullptr if element could
    /// not found
    ElementPtr getElementById(const ElementPtr _sdf,
                              const std::string &_elemName,
                              const std::string &_elemId,
                              const bool _isParentElement = false);

    /// \brief Finds the last index of a matching substring in _elemId from _ref
    /// \param[in] _elemId The element identifier
    /// \param[in] _startIdx The starting index for the substring in _elemId
    /// \param[in] _ref The reference to look for in _elemId
    /// \return int64_t the last index of the matching prefix in _elemId
    int64_t findPrefixLastIndex(const std::string &_elemId,
                                const int64_t &_startIdx,
                                const std::string &_ref);

    /// \brief Checks if the string is a valid action
    /// \param[in] _action The action
    /// \return True if the action is one of the following: add, modify, remove,
    /// or replace
    bool isValidAction(const std::string &_action);

    /// \brief Retrieves the sdf element that matches the xml element by element
    /// name and attribute 'name' if it exists (if the attribute 'name' does
    /// not exist in the xml then the element is retrieved using only the
    /// element name)
    /// \param[in] _elem The sdf (parent) element used to find the matching
    /// child element described in xml
    /// \param[in] _xml The xml element to find
    /// \param[in] _config Custom parser configuration.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _isModifyAction Is true if the action is modify, the
    /// attribute 'name' may not be in the sdf element (i.e., may be a
    /// modified/added attribute such as //camera)
    /// \return ElementPtr to the child element matching the xml, nullptr if
    /// element could not be found
    ElementPtr getElementByName(const ElementPtr _elem,
                                const tinyxml2::XMLElement *_xml,
                                const sdf::ParserConfig _config,
                                sdf::Errors &_errors,
                                const bool _isModifyAction = false);

    /// \brief Initialize an sdf element description from the xml element
    /// \param[in] _xml Pointer to xml element
    /// \param[in] _config Custom parser configuration
    /// \param[out] _errors Captures errors found during parsing
    /// \return ElementPtr to the initialized element description,
    /// nullptr if undefined/unknown sdf element
    ElementPtr initElementDescription(const tinyxml2::XMLElement *_xml,
                                      const ParserConfig &_config,
                                      Errors &_errors);

    /// \brief Handles individual actions of children in _childrenXml
    /// \param[in] _config Custom parser configuration
    /// \param[in] _source Source of the XML document, empty if it came from a
    /// string.
    /// \param[in] _childrenXml Pointer to xml element
    /// \param[out] _elem The sdf element to apply actions
    /// (i.e., add, modify, remove, replace)
    /// \param[out] _errors Captures errors found during parsing
    void handleIndividualChildActions(const ParserConfig &_config,
                                      const std::string &_source,
                                      tinyxml2::XMLElement *_childrenXml,
                                      ElementPtr _elem,
                                      Errors &_errors);

    /// \brief Adds a new element to an element from the included model
    /// \param[in] _config Custom parser configuration
    /// \param[in] _source Source of the XML document, empty if it came from a
    /// string.
    /// \param[in] _childXml Pointer to the new element to add, which is a child
    /// of //include/experimental:params
    /// \param[out] _elem The element from the included model to add the new
    /// element to
    /// \param[out] _errors Captures errors found during parsing
    void add(const ParserConfig &_config, const std::string &_source,
             tinyxml2::XMLElement *_childXml, ElementPtr _elem,
             Errors &_errors);

    /// \brief Modifies the attributes of an element from the included model
    /// \param[in] _xml Pointer to the xml element which contains the attributes
    /// to be modified
    /// \param[out] _elem The element from the included model to modify the
    /// attributes to
    /// \param[out] _errors Captures errors found during parsing
    void modifyAttributes(tinyxml2::XMLElement *_xml,
                         ElementPtr _elem, Errors &_errors);

    /// \brief Modifies the children elements of the included model
    /// \param[in] _xml Pointer to the xml element which contains the elements
    /// to be modified
    /// \param[in] _config Custom parser configuration.
    /// \param[out] _elem The element from the included model to modify
    /// \param[out] _errors Vector of errors.
    void modifyChildren(tinyxml2::XMLElement *_xml,
                        const sdf::ParserConfig &_config, ElementPtr _elem,
                        Errors &_errors);

    /// \brief Modifies element values and/or attributes of an element from the
    /// included model
    /// \param[in] _xml Pointer to the xml element which contains the elements
    /// to be modified
    /// \param[in] _config Custom parser configuration.
    /// \param[out] _elem The element from the included model to modify
    /// \param[out] _errors Captures errors found during parsing
    void modify(tinyxml2::XMLElement *_xml, const sdf::ParserConfig &_config,
                ElementPtr _elem, Errors &_errors);

    /// \brief Removes an element specified in xml
    /// \param[in] _xml Pointer to the xml element(s) to be removed from _elem
    /// \param[in] _config Custom parser configuration.
    /// \param[out] _elem The element from the included model to remove
    /// elements from
    /// \param[out] _errors Captures errors found during parsing
    void remove(const tinyxml2::XMLElement *_xml,
                const sdf::ParserConfig &_config, ElementPtr _elem,
                Errors &_errors);

    /// \brief Replace an element with another element
    /// \param[in] _newElem The replacement element
    /// \param[out] _origElem The element to be replaced
    void replace(const ElementPtr _newElem, ElementPtr _origElem);

    /// \brief Wrapper for parser.cc's readXml function, which populates
    /// an SDF Element from the XML input. The function first strips the
    /// action and element_id attributes before calling readXml
    /// \param[in] _config Custom parser configuration
    /// \param[in] _source Source of the XML document, empty if it came from a
    /// string.
    /// \param[in] _xml Pointer to the TinyXML element
    /// \param[in,out] _sdf SDF pointer to parse data into
    /// \param[out] _errors Captures errors found during parsing
    /// \return True on success, false on error
    bool xmlToSdf(const ParserConfig &_config, const std::string &_source,
                  tinyxml2::XMLElement *_xml, ElementPtr _sdf, Errors &_errors);
  }
  }
}
#endif
