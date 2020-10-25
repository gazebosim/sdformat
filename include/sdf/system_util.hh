/*
 * Copyright 2013 Open Source Robotics Foundation
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
#ifndef SDF_VISIBLE_HH_
#define SDF_VISIBLE_HH_

/** \def SDFORMAT_VISIBLE
 * Use to represent "symbol visible" if supported
 */

/** \def SDFORMAT_HIDDEN
 * Use to represent "symbol hidden" if supported
 */

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_SDFORMAT_SHARED
    #define SDFORMAT_VISIBLE __declspec(dllexport)
  #elif !defined SDFORMAT_STATIC_DEFINE
    #define SDFORMAT_VISIBLE __declspec(dllimport)
  #else
    #define SDFORMAT_VISIBLE
  #endif
#else
  #if __GNUC__ >= 4 && !defined SDFORMAT_STATIC_DEFINE
    #define SDFORMAT_VISIBLE __attribute__ ((visibility ("default")))
    #define SDFORMAT_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define SDFORMAT_VISIBLE
    #define SDFORMAT_HIDDEN
  #endif
#endif

// SDF_VISIBLE_HH_
#endif
