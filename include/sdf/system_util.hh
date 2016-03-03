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
#ifndef _SDF_VISIBLE_HH_
#define _SDF_VISIBLE_HH_

/** \def SDFORMAT_VISIBLE
 * Use to represent "symbol visible" if supported
 */

/** \def SDFORMAT_HIDDEN
 * Use to represent "symbol hidden" if supported
 */

#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define SDFORMAT_VISIBLE __attribute__ ((dllexport))
    #else
      #define SDFORMAT_VISIBLE __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define SDFORMAT_VISIBLE __attribute__ ((dllimport))
    #else
      #define SDFORMAT_VISIBLE __declspec(dllimport)
    #endif
  #endif
  #define SDFORMAT_HIDDEN
#else
  #if __GNUC__ >= 4
    #define SDFORMAT_VISIBLE __attribute__ ((visibility ("default")))
    #define SDFORMAT_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define SDFORMAT_VISIBLE
    #define SDFORMAT_HIDDEN
  #endif
#endif

#endif /* SDFORMAT_VISIBLE_HH */
