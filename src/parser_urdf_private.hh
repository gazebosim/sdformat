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
#ifndef SDF_PARSER_URDF_PRIVATE_HH
#define SDF_PARSER_URDF_PRIVATE_HH

#include <sdf/sdf_config.h>

#include "sdf/Types.hh"
#include "sdf/parser_urdf.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  namespace internal
  {
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    // Ignore deprecation warning for internal usage
    using URDF2SDF = sdf::URDF2SDF;
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  }  // namespace internal
  }
}

#endif
