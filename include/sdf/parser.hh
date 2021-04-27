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
#ifndef SDF_PARSER_HH_
#define SDF_PARSER_HH_

#include <string>

#include "sdf/SDFImpl.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
///
/// The parsing functions read XML elements contained in either a file or
/// string and translates the XML elements into SDF data structures. This
/// translation finds errors in the provided XML, fills in default values,
/// and performs any necessary version related conversions.
///
/// XML elements that are not part of the SDF specification are copied in
/// place. This preserves the given XML structure and data.
namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class Root;

  /// \brief Init based on the installed sdf_format.xml file
  SDFORMAT_VISIBLE
  bool init(SDFPtr _sdf);

  /// \brief Initialize the SDF interface using a file
  SDFORMAT_VISIBLE
  bool initFile(const std::string &_filename, SDFPtr _sdf);

  /// \brief Initialize an SDFElement interface using a file
  SDFORMAT_VISIBLE
  bool initFile(const std::string &_filename, ElementPtr _sdf);

  /// \brief Initialize the SDF interface using a string
  SDFORMAT_VISIBLE
  bool initString(const std::string &_xmlString, SDFPtr _sdf);

  /// \brief Populate the SDF values from a file
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. All files are converted to the latest
  /// SDF version
  /// \param[in] _filename Name of the SDF file
  /// \return Populated SDF pointer.
  SDFORMAT_VISIBLE
  sdf::SDFPtr readFile(const std::string &_filename);

  /// \brief Populate the SDF values from a file
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. All files are converted to the latest
  /// SDF version
  /// \param[in] _filename Name of the SDF file
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return Populated SDF pointer.
  SDFORMAT_VISIBLE
  sdf::SDFPtr readFile(const std::string &_filename, Errors &_errors);

  /// \brief Populate the SDF values from a file
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. All files are converted to the latest
  /// SDF version
  /// \param[in] _filename Name of the SDF file
  /// \param[in] _sdf Pointer to an SDF object.
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readFile(const std::string &_filename, SDFPtr _sdf, Errors &_errors);

  /// \brief Populate the SDF values from a file without converting to the
  /// latest SDF version
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. This function does not convert the
  /// loaded SDF to the latest version. Use this function with care, as it may
  /// prevent loading of DOM objects from this SDF object.
  /// \param[in] _filename Name of the SDF file
  /// \param[in] _sdf Pointer to an SDF object.
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readFileWithoutConversion(
      const std::string &_filename, SDFPtr _sdf, Errors &_errors);

  /// \brief Populate the SDF values from a file
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. All files are converted to the latest
  /// SDF version
  /// \param[in] _filename Name of the SDF file
  /// \param[in] _sdf Pointer to an SDF object.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readFile(const std::string &_filename, SDFPtr _sdf);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All string are converted to the
  /// latest SDF version
  /// \param[in] _xmlString XML string to be parsed.
  /// \param[in] _sdf Pointer to an SDF object.
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, SDFPtr _sdf, Errors &_errors);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All string are converted to the
  /// latest SDF version
  /// \param[in] _xmlString XML string to be parsed.
  /// \param[in] _sdf Pointer to an SDF object.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, SDFPtr _sdf);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All strings are converted to the
  /// latest SDF version
  /// \param[in] _xmlString XML string to be parsed.
  /// \param[in] _sdf Pointer to an SDF object.
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, ElementPtr _sdf,
      Errors &_errors);

  /// \brief Populate the SDF values from a string without converting to the
  /// latest SDF version
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// file it is converted to SDF first. This function does not convert the
  /// loaded SDF to the latest version. Use this function with care, as it may
  /// prevent loading of DOM objects from this SDF object.
  /// \param[in] _xmlString XML string to be parsed.
  /// \param[in] _sdf Pointer to an SDF object.
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readStringWithoutConversion(
      const std::string &_xmlString, SDFPtr _sdf, Errors &_errors);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All strings are converted to the
  /// latest SDF version
  /// \param[in] _xmlString XML string to be parsed.
  /// \param[in] _sdf Pointer to an sdf Element object.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, ElementPtr _sdf);

  /// \brief Get the file path to the model file
  /// \param[in] _modelDirPath directory system path of the model
  /// \return string with the full filesystem path to the best version (greater
  ///         SDF protocol supported by this sdformat version) of the .sdf
  ///         model files hosted by _modelDirPath.
  SDFORMAT_VISIBLE
  std::string getModelFilePath(const std::string &_modelDirPath);

  /// \brief Copy the contents of the first model element from one ElementPtr
  /// to another ElementPtr, prepending the copied model name with `::` to
  /// link and joint names, and apply the model pose to the copied link poses.
  /// If //xyz/@expressed_in == "__model__" for the axes of any copied joints,
  /// then apply the model pose rotation to those joint axes.
  /// \param[in] _sdf ElementPtr for model into which the elements will be
  /// copied.
  /// \param[in] _includeSDF The first model element from this ElementPtr will
  /// be copied to _sdf with the mentioned name and pose transformations.
  SDFORMAT_VISIBLE
  void addNestedModel(ElementPtr _sdf, ElementPtr _includeSDF);

  /// \brief Copy the contents of the first model element from one ElementPtr
  /// to another ElementPtr, prepending the copied model name with `::` to
  /// link and joint names, and apply the model pose to the copied link poses.
  /// If //xyz/@expressed_in == "__model__" for the axes of any copied joints,
  /// then apply the model pose rotation to those joint axes.
  /// \param[in] _sdf ElementPtr for model into which the elements will be
  /// copied.
  /// \param[in] _includeSDF The first model element from this ElementPtr will
  /// be copied to _sdf with the mentioned name and pose transformations.
  /// \param[out] _errors Any errors will be appended to this variable.
  SDFORMAT_VISIBLE
  void addNestedModel(ElementPtr _sdf, ElementPtr _includeSDF, Errors &_errors);

  /// \brief Convert an SDF file to a specific SDF version.
  /// \param[in] _filename Name of the SDF file to convert.
  /// \param[in] _version Version to convert _filename to.
  /// \param[out] _sdf Pointer to the converted SDF document.
  /// \return True on success.
  SDFORMAT_VISIBLE
  bool convertFile(const std::string &_filename, const std::string &_version,
                   SDFPtr _sdf);

  /// \brief Convert an SDF string to a specific SDF version.
  /// \param[in] _sdfString The SDF string to convert.
  /// \param[in] _version Version to convert _filename to.
  /// \param[out] _sdf Pointer to the converted SDF document.
  /// \return True on success.
  SDFORMAT_VISIBLE
  bool convertString(const std::string &_sdfString,
                     const std::string &_version, SDFPtr _sdf);

  /// \brief Check that for each model, the canonical_link attribute value
  /// matches the name of a link in the model if the attribute is set and
  /// not empty.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first error is found.
  /// \param[in] _root sdf Root object to check recursively.
  /// \return True if all models have valid canonical_link attributes.
  SDFORMAT_VISIBLE
  bool checkCanonicalLinkNames(const sdf::Root *_root);

  /// \brief For the world and each model, check that the attached_to graphs
  /// build without errors and have no cycles.
  /// Confirm that following directed edges from each vertex in the graph
  /// leads to a model, link, or world frame.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first error is found.
  /// \param[in] _root sdf Root object to check recursively.
  /// \return True if all attached_to graphs are valid.
  SDFORMAT_VISIBLE
  bool checkFrameAttachedToGraph(const sdf::Root *_root);

  /// \brief Check that for each frame, the attached_to attribute value
  /// does not match its own frame name but does match the name of a
  /// link, joint, or other frame in the model if the attribute is set and
  /// not empty.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first error is found.
  /// \param[in] _root sdf Root object to check recursively.
  /// \return True if all frames have valid attached_to attributes.
  SDFORMAT_VISIBLE
  bool checkFrameAttachedToNames(const sdf::Root *_root);

  /// \brief Check that all joints in contained models specify parent
  /// and child link names that match the names of sibling links.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first error is found.
  /// \param[in] _root sdf Root object to check recursively.
  /// \return True if all models have joints with valid parent and child
  /// link names.
  SDFORMAT_VISIBLE
  bool checkJointParentChildLinkNames(const sdf::Root *_root);

  /// \brief For the world and each model, check that the attached_to graphs
  /// build without errors and have no cycles.
  /// Confirm that following directed edges from each vertex in the graph
  /// leads to a model, link, or world frame.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first error is found.
  /// \param[in] _root sdf Root object to check recursively.
  /// \return True if all attached_to graphs are valid.
  SDFORMAT_VISIBLE
  bool checkPoseRelativeToGraph(const sdf::Root *_root);

  /// \brief Check that all sibling elements of the same type have unique names.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first duplicate name is found.
  /// \param[in] _elem sdf Element to check recursively.
  /// \return True if all contained elements have do not share a name with
  /// sibling elements of the same type.
  SDFORMAT_VISIBLE
  bool recursiveSameTypeUniqueNames(sdf::ElementPtr _elem);

  /// \brief Check that all sibling elements of the any type have unique names.
  /// This checks recursively and should check the files exhaustively
  /// rather than terminating early when the first duplicate name is found.
  /// \param[in] _elem sdf Element to check recursively.
  /// \return True if all contained elements have do not share a name with
  /// sibling elements of any type.
  SDFORMAT_VISIBLE
  bool recursiveSiblingUniqueNames(sdf::ElementPtr _elem);

  /// \brief Check whether the element should be validated. If this returns
  /// false, validators such as the unique name and reserve name checkers should
  /// skip this element and its descendants.
  /// \param[in] _elem sdf Element to check.
  /// \return True if the element should be validated
  SDFORMAT_VISIBLE
  bool shouldValidateElement(sdf::ElementPtr _elem);
  }
}
#endif
