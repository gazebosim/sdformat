/*
 * Copyright 2022 Open Source Robotics Foundation
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
#ifndef SDF_USD_USDERROR_HH_
#define SDF_USD_USDERROR_HH_

#include <optional>
#include <string>
#include <vector>

#include <ignition/utils/ImplPtr.hh>

#include <sdf/Error.hh>
#include <sdf/usd/Export.hh>
#include <sdf/config.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE
  {
  namespace usd
  {
  //
  /// \enum ErrorCode
  /// \brief Set of error codes. Usually one or more errors are returned in
  /// an Errors vector. The collection of Errors should be taken as a whole,
  /// where an error toward the beginning of the vector can inform errors
  /// toward the end of the vector.
  /// \sa Errors
  enum class UsdErrorCode
  {
    // \brief No error
    NONE = 0,

    /// \brief A wrapped SDF error.
    SDF_ERROR,

    /// \brief Parsing of SDF object to a USD object failed.
    SDF_TO_USD_PARSING_ERROR,

    /// \brief Parsing of USD object to a SDF object failed.
    USD_TO_SDF_PARSING_ERROR,

    /// \brief The pxr::SdfPath does not point to a valid USD prim.
    INVALID_PRIM_PATH,

    /// \brief A pxr API was not able to be applied to a USD prim.
    FAILED_PRIM_API_APPLY,

    /// \brief Attempt to define a USD prim or prim schema failed.
    FAILED_USD_DEFINITION,

    /// \brief Failure to load a mesh and/or submesh
    MESH_LOAD_FAILURE,

    /// \brief Invalid submesh primitive type
    INVALID_SUBMESH_PRIMITIVE_TYPE,

    /// \brief Invalid material
    INVALID_MATERIAL,

    /// \brief Invalid usd file
    INVALID_USD_FILE,

    /// \brief Invalid up axis
    INVALID_UP_AXIS,

    /// \brief Prim is missing a particular attribute
    PRIM_MISSING_ATTRIBUTE,

    /// \brief Prim is of the incorrect schema type
    PRIM_INCORRECT_SCHEMA_TYPE,
  };

  class IGNITION_SDFORMAT_USD_VISIBLE UsdError
  {
    /// \brief Default constructor
    public: UsdError();

    /// \brief Constructor.
    /// \param[in] _code The error code. If the error code is SDF_ERROR, the
    /// constructor that takes an sdf::Error object should be used instead.
    /// \param[in] _message A description of the error.
    /// \sa ErrorCode.
    public: UsdError(const UsdErrorCode _code, const std::string &_message);

    /// \brief Constructor.
    /// \param[in] _code The error code. If the error code is SDF_ERROR, the
    /// constructor that takes an sdf::Error object should be used instead.
    /// \param[in] _message A description of the error.
    /// \param[in] _filePath The file path that is related to this error.
    /// \sa ErrorCode.
    public: UsdError(const UsdErrorCode _code, const std::string &_message,
                     const std::string &_filePath);

    /// \brief Constructor.
    /// \param[in] _code The error code. If the error code is SDF_ERROR, the
    /// constructor that takes an sdf::Error object should be used instead.
    /// \param[in] _message A description of the error.
    /// \param[in] _filePath The file path that is related to this error.
    /// \param[in] _lineNumber The line number in the provided file path where
    /// this error was raised.
    /// \sa ErrorCode.
   public: UsdError(const UsdErrorCode _code, const std::string &_message,
                    const std::string &_filePath, int _lineNumber);

    /// \brief Constructor.
    /// \param[in] _sdf_error Wrap an sdf error.
    /// \sa ErrorCode.
    public: explicit UsdError(const sdf::Error &_sdfError);

    /// \brief Get the error code.
    /// \return An error code.
    /// \sa ErrorCode.
    public: UsdErrorCode Code() const;

    /// \brief Get the error message, which is a description of the error.
    /// \return Error message.
    public: const std::string &Message() const;

    /// \brief Get the file path associated with this error.
    /// \return Returns the path of the file that this error is related to,
    /// nullopt otherwise.
    public: const std::optional<std::string> &FilePath() const;

    /// \brief Sets the file path that is associated with this error.
    /// \param[in] _filePath The file path that is related to this error. (e.g.
    /// /tmp/test_file.usd)
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Get the line number associated with this error.
    /// \return Returns the line number. nullopt otherwise.
    public: std::optional<int> LineNumber() const;

    /// \brief Sets the line number that is associated with this error.
    /// \param[in] _lineNumber The line number that is related to this error.
    public: void SetLineNumber(int _lineNumber);

    /// \brief Get the underlying sdf error.
    /// \return The underlying sdf error or nullopt otherwise.
    public: std::optional<sdf::Error> SdfError() const;

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
   public: friend IGNITION_SDFORMAT_VISIBLE std::ostream &operator<<(
        std::ostream &_out, const sdf::usd::UsdError &_err);

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };

  using UsdErrors = std::vector<UsdError>;

  }  // namespace usd
  }  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
