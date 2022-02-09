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
#ifndef SDF_USD_ERROR_HH_
#define SDF_USD_ERROR_HH_

#include <sdf/Error.hh>
#include <sdf/Export.hh>
#include <sdf/config.hh>
#include <ignition/utils/ImplPtr.hh>

#include <optional>
#include <string>
#include <vector>

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::string
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

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
/// an Errors vector. The collection of Errors should be take as a whole,
/// where an error toward the beginning of the vector can inform errors
/// toward the end of the vector.
/// \sa Errors
enum class ErrorCode
{
  // \brief No error
  NONE = 0,

  /// \brief A wrapped SDF error.
  SDF_ERROR,

  /// \brief Parsing a SDF object to a USD object failed.
  /// This error type is specific to the USD component.
  SDF_TO_USD_PARSING_ERROR,

  /// \brief The pxr::SdfPath does not point to a valid USD prim.
  /// This error type is specific to the USD component.
  INVALID_PRIM_PATH,

  /// \brief A pxr API was not able to be applied to a USD prim.
  /// This error type is specific to the USD component.
  FAILED_PRIM_API_APPLY,
};

class IGNITION_SDFORMAT_VISIBLE Error
{
  /// \brief Default constructor
 public:
  Error();

  /// \brief Constructor.
  /// \param[in] _code The error code.
  /// \param[in] _message A description of the error.
  /// \sa ErrorCode.
 public:
  Error(const ErrorCode _code, const std::string &_message);

  /// \brief Constructor.
  /// \param[in] _code The error code.
  /// \param[in] _message A description of the error.
  /// \param[in] _filePath The file path that is related to this error.
  /// \sa ErrorCode.
 public:
  Error(const ErrorCode _code, const std::string &_message,
        const std::string &_filePath);

  /// \brief Constructor.
  /// \param[in] _code The error code.
  /// \param[in] _message A description of the error.
  /// \param[in] _filePath The file path that is related to this error.
  /// \param[in] _lineNumber The line number in the provided file path where
  /// this error was raised.
  /// \sa ErrorCode.
 public:
  Error(const ErrorCode _code, const std::string &_message,
        const std::string &_filePath, int _lineNumber);

  /// \brief Constructor.
  /// \param[in] _sdf_error Wrap a sdf error.
  /// \sa ErrorCode.
 public:
  Error(const sdf::Error& _sdf_error);

  /// \brief Get the error code.
  /// \return An error code.
  /// \sa ErrorCode.
 public:
  ErrorCode Code() const;

  /// \brief Get the error message, which is a description of the error.
  /// \return Error message.
 public:
  std::string Message() const;

  /// \brief Get the file path associated with this error.
  /// \return Returns the path of the file that this error is related to,
  /// nullopt otherwise.
 public:
  std::optional<std::string> FilePath() const;

  /// \brief Sets the file path that is associated with this error.
  /// \param[in] _filePath The file path that is related to this error. (e.g.
  /// /tmp/test_file.sdf)
 public:
  void SetFilePath(const std::string &_filePath);

  /// \brief Get the line number associated with this error.
  /// \return Returns the line number. nullopt otherwise.
 public:
  std::optional<int> LineNumber() const;

  /// \brief Sets the line number that is associated with this error.
  /// \param[in] _lineNumber The line number that is related to this error.
 public:
  void SetLineNumber(int _lineNumber);

  /// \brief Get the underlying sdf error.
  /// \return The underlying sdf error.
 public:
  std::optional<sdf::Error> SdfError() const;

  /// \brief Safe bool conversion.
  /// \return True if this Error's Code() != NONE. In otherwords, this is
  /// true when there is an error.
 public:
  explicit operator bool() const;

  /// \brief Compare this Error to a boolean value.
  /// \return True if the boolean evaluation of this Error equals _value.
  /// If _value == false, then true is returned when this Error's Code() is
  /// equal to NONE and false is returned otherwise. If _value == true,
  /// then true is returned when this Error's Code() is not equal to NONE
  /// and false is returned otherwise.
  /// \sa explicit operator bool() const
 public:
  bool operator==(const bool _value) const;

  /// \brief Output operator for an error.
  /// \param[in,out] _out The output stream.
  /// \param[in] _err The error to output.
  /// \return Reference to the given output stream
 public:
  friend IGNITION_SDFORMAT_VISIBLE std::ostream &operator<<(
      std::ostream &_out, const sdf::usd::Error &_err);

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};

using Errors = std::vector<Error>;

}  // namespace usd
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
