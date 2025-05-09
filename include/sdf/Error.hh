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
#ifndef SDF_ERROR_HH_
#define SDF_ERROR_HH_

#include <optional>
#include <ostream>
#include <string>
#include <gz/utils/ImplPtr.hh>
#include <sdf/config.hh>
#include "sdf/Console.hh"
#include "sdf/system_util.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::string
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \enum ErrorCode
  /// \brief Set of error codes. Usually one or more errors are returned in
  /// an Errors vector. The collection of Errors should be take as a whole,
  /// where an error toward the beginning of the vector can inform errors
  /// toward the end of the vector.
  /// \sa Errors
  enum class ErrorCode
  {
    // \brief No error
    NONE = 0,

    /// \brief Indicates that reading an SDF file failed.
    FILE_READ,

    /// \brief A duplicate name was found for an element where unique names
    /// are required.
    DUPLICATE_NAME,

    /// \brief A reserved name was used in an entity name attribute.
    RESERVED_NAME,

    /// \brief Indicates that a required SDF attribute is missing.
    ATTRIBUTE_MISSING,

    /// \brief This error indicates that an SDF attribute is invalid.
    ATTRIBUTE_INVALID,

    /// \brief This error indicates that an SDF attribute is deprecated.
    ATTRIBUTE_DEPRECATED,

    /// \brief Indicates an attribute was included that is not part of the sdf
    /// spec
    ATTRIBUTE_INCORRECT_TYPE,

    /// \brief Indicates that a required SDF element is missing.
    ELEMENT_MISSING,

    /// \brief This error indicates that an SDF element is invalid.
    ELEMENT_INVALID,

    /// \brief This error indicates that an SDF element is deprecated.
    ELEMENT_DEPRECATED,

    /// \brief Indicates that an  incorrect SDF element type was
    /// encountered. This error is used when an element of certain type is
    /// expected, and an element of a different type was received.
    ELEMENT_INCORRECT_TYPE,

    /// \brief Generic error type for Elements
    ELEMENT_ERROR,

    /// \brief A URI is invalid.
    URI_INVALID,

    /// \brief A error occurred while trying to resolve a URI.
    URI_LOOKUP,

    /// \brief A filesystem directory does not exist.
    DIRECTORY_NONEXISTANT,

    /// \brief A model with an invalid canonical link.
    MODEL_CANONICAL_LINK_INVALID,

    /// \brief A model without a link.
    MODEL_WITHOUT_LINK,

    /// \brief Indicates that a DOM object tried to read a nested model.
    NESTED_MODELS_UNSUPPORTED,

    /// \brief A link has invalid inertia.
    LINK_INERTIA_INVALID,

    /// \brief A joint has an invalid child link.
    JOINT_CHILD_LINK_INVALID,

    /// \brief A joint has an invalid parent link.
    JOINT_PARENT_LINK_INVALID,

    /// \brief A joint has the same link specified as parent and child.
    JOINT_PARENT_SAME_AS_CHILD,

    /// \brief The frame attached-to value does not match the name of an
    /// existing frame in the current scope.
    FRAME_ATTACHED_TO_INVALID,

    /// \brief The frame attached-to graph contains a cycle.
    FRAME_ATTACHED_TO_CYCLE,

    /// \brief The frame attached-to graph has an internal error.
    FRAME_ATTACHED_TO_GRAPH_ERROR,

    /// \brief The pose relative-to value does not match the name of an
    /// existing frame in the current scope.
    POSE_RELATIVE_TO_INVALID,

    /// \brief The pose relative-to graph contains a cycle.
    POSE_RELATIVE_TO_CYCLE,

    /// \brief The pose relative-to graph has an internal error.
    POSE_RELATIVE_TO_GRAPH_ERROR,

    /// \brief The rotation snap config provided is not valid.
    ROTATION_SNAP_CONFIG_ERROR,

    /// \brief Indicates that reading an SDF string failed.
    STRING_READ,

    /// \brief The specified placement frame is invalid
    MODEL_PLACEMENT_FRAME_INVALID,

    /// \brief The provided version has been deprecated or it is pre-versioning
    VERSION_DEPRECATED,

    /// \brief Merge include is unsupported for the type of entity being
    /// included, or the custom parser does not support merge includes.
    MERGE_INCLUDE_UNSUPPORTED,

    /// \brief Generic error type for parameters (values of SDFormat elements
    /// or attributes).
    PARAMETER_ERROR,

    /// \brief The specified parameter (values of SDFormat elements
    /// or attributes) type is unknown.
    UNKNOWN_PARAMETER_TYPE,

    /// \brief Generic error to be thrown with SDF_ASSERT by the caller.
    /// This has been created to help preserve behavior.
    FATAL_ERROR,

    /// \brief Generic warning saved as error due to WarningsPolicy config.
    WARNING,

    /// \brief The joint axis expressed-in value does not match the name of an
    /// existing frame in the current scope.
    JOINT_AXIS_EXPRESSED_IN_INVALID,

    /// \brief SDF conversion generic error.
    CONVERSION_ERROR,

    /// \brief Generic error during parsing.
    PARSING_ERROR,

    /// \brief The joint axis mimic does not refer to a valid joint in the
    /// current scope.
    JOINT_AXIS_MIMIC_INVALID,

    /// \brief Error at the XML level.
    XML_ERROR,
  };

  class SDFORMAT_VISIBLE Error
  {
    /// \brief Default constructor
    public: Error();

    /// \brief Constructor.
    /// \param[in] _code The error code.
    /// \param[in] _message A description of the error.
    /// \sa ErrorCode.
    public: Error(const ErrorCode _code, const std::string &_message);

    /// \brief Constructor.
    /// \param[in] _code The error code.
    /// \param[in] _message A description of the error.
    /// \param[in] _filePath The file path that is related to this error.
    /// \sa ErrorCode.
    public: Error(const ErrorCode _code, const std::string &_message,
                  const std::string &_filePath);

    /// \brief Constructor.
    /// \param[in] _code The error code.
    /// \param[in] _message A description of the error.
    /// \param[in] _filePath The file path that is related to this error.
    /// \param[in] _lineNumber The line number in the provided file path where
    /// this error was raised.
    /// \sa ErrorCode.
    public: Error(const ErrorCode _code, const std::string &_message,
                  const std::string &_filePath, int _lineNumber);

    /// \brief Get the error code.
    /// \return An error code.
    /// \sa ErrorCode.
    public: ErrorCode Code() const;

    /// \brief Get the error message, which is a description of the error.
    /// \return Error message.
    public: std::string Message() const;

    /// \brief Sets the message associated with this error.
    /// \param [in] _message Message that describes this error.
    public: void SetMessage(const std::string &_message);

    /// \brief Get the file path associated with this error.
    /// \return Returns the path of the file that this error is related to,
    /// nullopt otherwise.
    public: std::optional<std::string> FilePath() const;

    /// \brief Sets the file path that is associated with this error.
    /// \param[in] _filePath The file path that is related to this error. (e.g.
    /// /tmp/test_file.sdf)
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Get the line number associated with this error.
    /// \return Returns the line number. nullopt otherwise.
    public: std::optional<int> LineNumber() const;

    /// \brief Sets the line number that is associated with this error.
    /// \param[in] _lineNumber The line number that is related to this error.
    public: void SetLineNumber(int _lineNumber);

    /// \brief Get the XPath-like trace that is associated with this error.
    /// \return Returns the XPath-like trace that this error is related to,
    /// nullopt otherwise.
    public: std::optional<std::string> XmlPath() const;

    /// \brief Sets the XML path that is associated with this error.
    /// \param[in] _xmlPath The XML path that is related to this error. (e.g.
    /// /sdf/world[@name="default"]/model[@name="robot1"]/link[@name="link"])
    /// \sa Element::SetXmlPath
    public: void SetXmlPath(const std::string &_xmlPath);

    /// \brief Safe bool conversion.
    /// \return True if this Error's Code() != NONE. In otherwords, this is
    /// true when there is an error.
    public: explicit operator bool() const;

    /// \brief Compare this Error to a boolean value.
    /// \return True if the boolean evaluation of this Error equals _value.
    /// If _value == false, then true is returned when this Error's Code() is
    /// equal to NONE and false is returned otherwise. If _value == true,
    /// then true is returned when this Error's Code() is not equal to NONE
    /// and false is returned otherwise.
    /// \sa explicit operator bool() const
    public: bool operator==(const bool _value) const;

    /// \brief Output operator for an error.
    /// \param[in,out] _out The output stream.
    /// \param[in] _err The error to output.
    /// \return Reference to the given output stream
    public: friend SDFORMAT_VISIBLE std::ostream &operator<<(
        std::ostream &_out, const sdf::Error &_err);

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Internal namespace. Functions and classes defined in this namespace
  /// are for internal use only and maybe removed without a deprecation cycle.
  namespace internal
  {
  /// \brief Prints the error to _out or throw using SDF_ASSERT depending on the
  /// ErrorCode in _error.
  /// \param[out] _out ostream to use for printing errors.
  /// \param[in] _error _error The error object to be printed
  void SDFORMAT_VISIBLE throwOrPrintError(sdf::Console::ConsoleStream &_out,
                                          const sdf::Error &_error);
  }  // namespace internal
  }  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
#ifdef _WIN32
#pragma warning(pop)
#endif


#endif
