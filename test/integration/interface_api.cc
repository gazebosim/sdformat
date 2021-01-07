/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <string>
#include <variant>

#include "sdf/Filesystem.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "test_config.h"


// A struct is needed to create a recursive std::variant based data structure
struct TomlValue
{
  using KeyValue = std::map<std::string, TomlValue>;
  std::variant<std::monostate, sdf::Param, std::vector<TomlValue>, KeyValue>
      data;
};

using TomlDocument = std::map<std::string, TomlValue>;

void PrintTomlValue(const std::string &_key, const TomlValue &_value)
{
  std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, sdf::Param>)
        {
          std::cout << _key  << " = " << arg << std::endl;
        }
        else if constexpr (std::is_same_v<T, std::vector<TomlValue>>)
        {
          for (size_t i = 0; i < arg.size(); ++i)
          {
            // std::cout << "\t param: " << param.GetKey() << " = " << param
            //           << std::endl;
            PrintTomlValue(_key + "[" + std::to_string(i) + "]", arg[i]);
          }
        }
        else if constexpr (std::is_same_v<T, std::map<std::string, TomlValue>>)
        {
          for (const auto &[key, val] : arg)
          {
            // std::cout << "\t param: " << param.GetKey() << " = " << param
            //           << std::endl;
            PrintTomlValue(_key, val);
          }
        }
      },
      _value.data);
}

TomlDocument parseToml(const std::string &_filePath, sdf::Errors &_errors)
{
  std::fstream fs;
  fs.open(_filePath);
  if (!fs.good())
  {
    _errors.emplace_back(sdf::ErrorCode::FILE_READ,
        "File [" + _filePath + "] could not be read.");
    return {};
  }

  std::map<std::string, TomlValue> doc;
  TomlValue curValue;
  curValue.data = std::map<std::string, TomlValue>();

  std::unordered_map<std::string, std::string> keyType {
    {"name", "string"},
    {"pose", "pose"},
    {"parent", "string"},
    {"child", "string"},
  };

  auto readValue = [](const std::string &_inp)
  {
    const std::string trimmed = sdf::trim(_inp);
    // Find the quotes
    auto begInd = trimmed.find('"');
    auto endInd = trimmed.rfind('"');
    return trimmed.substr(begInd + 1, endInd - 1);
  };
  auto readArrayName = [](const std::string &_inp)
  {
    const std::string trimmed = sdf::trim(_inp);
    // Find the quotes
    auto begInd = trimmed.find("[[");
    auto endInd = trimmed.rfind("]]");
    return trimmed.substr(begInd + 2, endInd - 2);
  };

  std::string curArrayName = "";
  while (fs.good())
  {
    std::string line;
    std::getline(fs, line);
    sdf::trim(line);
    if (line.empty() || line[0] == '#' )
    {
      continue;
    }
    else
    {
      std::size_t eqInd = line.find('=');
      if (eqInd != std::string::npos)
      {
        const std::string key = sdf::trim(line.substr(0, eqInd));
        const std::string value = readValue(line.substr(eqInd + 1));
        sdf::Param param(key, keyType[key], "", true);
        param.SetFromString(value);
        if (curArrayName.empty())
        {
          doc[key] = {param};
        }
        else
        {
          auto &entry = doc[curArrayName].data;
          if (auto array = std::get_if<std::vector<TomlValue>>(&entry))
          {
            auto &data = std::get<TomlValue::KeyValue>(array->back().data);
            data[key] = {param};
          }
        }
      }
      else if (line.find("[[") != std::string::npos)
      {
        const std::string arrayName = readArrayName(line);
        if (arrayName.empty())
        {
          // ERROR
        }
        else
        {
          auto &entry = doc[arrayName].data;
          if (std::holds_alternative<std::vector<TomlValue>>(entry))
          {
            auto &array = std::get<std::vector<TomlValue>>(entry);
            // array.emplace_back(curValue);
            array.push_back({});
            array.back().data = std::map<std::string, TomlValue>();
          }
          else
          {
            std::vector<TomlValue> array;
            array.push_back({});
            array.back().data = std::map<std::string, TomlValue>();
            entry = array;
          }
          // if there was an array being built, add it to the map
          curArrayName = arrayName;
        }
      }
    }
  }

  // if (!curArrayName.empty())
  // {
  //   doc.emplace(curArrayName, curArray);
  // }

  fs.close();
  std::cout << "Parsed toml:" << std::endl;
  for (const auto &entry : doc)
  {
    PrintTomlValue(entry.first, entry.second);
  }

  return doc;
}

sdf::InterfaceModelPtr customTomlParser(
    const sdf::NestedInclude &_include, sdf::Errors &_errors)
{
  std::cout << "Parsing..." << std::endl;
  std::cout << "uri: " << _include.uri << std::endl;
  std::cout << "resolvedFileName: " << _include.resolvedFileName << std::endl;
  std::cout << "localModelName: " << _include.localModelName << std::endl;
  std::cout << "isStatic: " << _include.isStatic << std::endl;
  std::cout << "virtualCustomElements: "
            << _include.virtualCustomElements->ToString("") << std::endl;

  TomlDocument doc = parseToml(_include.resolvedFileName, _errors);
  if (_errors.empty())
  {
    const std::string canonicalLink =
        std::get<sdf::Param>(doc["canonical_link"].data).GetAsString();

    ignition::math::Pose3d poseInParentModelFrame;
    std::get<sdf::Param>(doc["pose"].data).Get(poseInParentModelFrame);

    ignition::math::Pose3d poseInCanonicalLinkFrame;
    auto model = std::make_shared<sdf::InterfaceModel>(
        _include.localModelName, canonicalLink, pose, poseInParentModelFrame);

  }
  return nullptr;
}

/////////////////////////////////////////////////
TEST(InterfaceAPI, IncludeYAMLFile)
{
  const std::string modelDir = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "integration", "model");

  const std::string testFile = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "sdf", "include_with_interface_api.sdf");

  sdf::ParserConfig config;
  config.SetFindCallback(
      [&](const std::string &_file)
      {
        std::cout << "Find: " << _file << std::endl;
        return sdf::filesystem::append(modelDir, _file);
      });

  config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty()) << errors;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->InterfaceModelCount());
  auto interfaceModel = world->InterfaceModelByIndex(0);
  ASSERT_NE(nullptr, interfaceModel);
  EXPECT_EQ("box", interfaceModel->Name());
}
