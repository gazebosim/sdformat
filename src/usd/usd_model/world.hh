/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef USD_MODEL_WORLD_HH
#define USD_MODEL_WORLD_HH

#include <ostream>
#include <string>
#include <vector>

#include <ignition/math/Vector3.hh>

#include <sdf/Light.hh>

#include "types.hh"

namespace usd {
  class WorldInterface {
    public:
      std::string _worldName;
      std::vector<ModelInterfaceSharedPtr> _models;
      std::map<std::string, std::shared_ptr<sdf::Light>> _lights;

      float magnitude;
      ignition::math::Vector3d gravity;

      friend std::ostream& operator<<(std::ostream& os, const WorldInterface& _world)
      {
        os << "World name: " << _world._worldName;
        os << _world.gravity * _world.magnitude << "\n";
        return os;
      }
  };
}

#endif
