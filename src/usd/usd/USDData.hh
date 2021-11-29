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

#ifndef USD_USDDATA_HH
#define USD_USDDATA_HH

#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "USDStage.hh"

#include "sdf/Material.hh"

namespace usd {

  class USDData {
    public:
      USDData(const std::string &_filename);
      bool Init();
      bool AddStage(const std::string &_ref);

      int ParseMaterials();

      std::pair<std::string, std::shared_ptr<USDStage>> findStage(const std::string &_name);

      friend std::ostream& operator<<(std::ostream& output, const USDData& H);
      std::string filename;
      std::string directoryPath;
      std::vector<std::string> _subDirectories;
      std::unordered_map<std::string, std::shared_ptr<USDStage>> _references;

      std::set<std::string> _models;
      std::unordered_map<std::string, sdf::Material> materials;
  };
}

#endif
