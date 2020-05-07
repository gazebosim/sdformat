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

#ifndef SDF_EMBEDDEDSDF_HH_
#define SDF_EMBEDDEDSDF_HH_

#include <map>
#include <string>

#include "sdf/Types.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \internal

  /// A map where the keys are a source-relative pathnames within the "sdf"
  /// directory such as "1.8/root.sdf", and the values are the contents of
  /// that source file.
  const std::map<std::string, std::string> &GetEmbeddedSdf();
}
}
#endif
