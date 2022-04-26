/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
 */

#ifndef SDFORMAT_PYTHON_EXCEPTIONS_HH_
#define SDFORMAT_PYTHON_EXCEPTIONS_HH_

#include "sdf/Error.hh"
#include "sdf/config.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{

std::string ErrorString(const sdf::Errors &_errors)
{
  std::string errorStr = "\n";
  for (const auto & error : _errors)
  {
    errorStr += std::string("Error code: ") +
                std::to_string(static_cast<int>(error.Code()));
    auto lineNumber = error.LineNumber();
    if (lineNumber)
    {
      errorStr += std::string("Line: ") +
                  std::to_string(error.LineNumber().value());
    }
    errorStr += std::string(" Message: ") + error.Message() + "\n";
  }
  return errorStr;
}

class SDFErrorsException : public std::runtime_error
{
public:
  explicit SDFErrorsException(const sdf::Errors &_errors)
   : std::runtime_error(ErrorString(_errors))
  {}

  ~SDFErrorsException() = default;
};

}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf

#endif  // SDFORMAT_PYTHON_EXCEPTIONS_HH_
