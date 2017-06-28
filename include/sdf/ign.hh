/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef SDF_IGN_HH_
#define SDF_IGN_HH_

#include <cstring>

#include "sdf/system_util.hh"

/// \brief External hook to execute 'ign sdf -k' from the command line.
/// \param[in] _path Path to the file to validate.
/// \return Zero on success, negative one otherwise.
extern "C" SDFORMAT_VISIBLE int cmdCheck(const char *_path);

/// \brief External hook to read the library version.
/// \return C-string representing the version. Ex.: 0.1.2
extern "C" SDFORMAT_VISIBLE char *ignitionVersion();

#endif
