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

#include <sdf/sdf_config.h>

#include <sdf/Types.hh>

#include "pyExceptions.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE
{
namespace python
{
/// \brief Throw an exception if the `_errors` container is not empty.
/// \param[in] _errors The list of errors to check.
/// \throws PySDFErrorsException
void ThrowIfErrors(const sdf::Errors &_errors);
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
