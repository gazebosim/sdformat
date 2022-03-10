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
 *
*/

#ifndef SDF_USD_USD_PARSER_USD_MODEL_WORLD_INTERFACE_HH_
#define SDF_USD_USD_PARSER_USD_MODEL_WORLD_INTERFACE_HH_

#include <ostream>
#include <string>
#include <vector>

#include <ignition/math/Vector3.hh>

#include <sdf/Light.hh>
#include <sdf/sdf_config.h>

#include "ModelInterface.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief This class store data about the world
    class WorldInterface {
      public:
        /// \brief World name
        std::string worldName;

        /// \brief Magnitude of the gravity
        float magnitude;

        /// \brief Gravity (X, Y, Z)
        ignition::math::Vector3d gravity;

        /// \brief World's lights
        std::map<std::string, std::shared_ptr<sdf::Light>> lights;

        /// \brief World model
        std::vector<std::shared_ptr<ModelInterface>> models;

        friend std::ostream& operator<<(
          std::ostream& os, const WorldInterface& _world)
        {
          os << "World name: " << _world.worldName;
          os << "Gravity: " << _world.gravity * _world.magnitude << "\n";
          return os;
        }
    };
  }
  }
}

#endif
