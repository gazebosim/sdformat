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

#include "sdf/Error.hh"

using namespace sdf;

/////////////////////////////////////////////////
Error::Error(const ErrorCode _code, const std::string &_message)
{
  this->code = _code;
  this->message = _message;
}

/////////////////////////////////////////////////
ErrorCode Error::Code() const
{
  return this->code;
}

/////////////////////////////////////////////////
std::string Error::Message() const
{
  return this->message;
}

/////////////////////////////////////////////////
Error::operator bool() const
{
  return this->code != ErrorCode::NONE;
}

/////////////////////////////////////////////////
bool Error::operator==(const bool _value) const
{
  return ((this->code != ErrorCode::NONE) && _value) ||
         ((this->code == ErrorCode::NONE) && !_value);
}

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
std::ostream &operator<<(std::ostream &_out, const sdf::Error &_err)
{
  _out << "Error Code "
    << static_cast<std::underlying_type<sdf::ErrorCode>::type>(_err.Code())
    << " Msg: " << _err.Message();
  return _out;
}
}
}
