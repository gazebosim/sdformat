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

#include <stdexcept>
#include <string>
#include <variant>

#include "sdf/Filesystem.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceLink.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

// Debugging
// #include "json.hpp"

#include "test_config.h"


namespace toml
{
std::string appendPrefix(const std::string &_input)
{
  return _input;
}

template <typename T, typename... Args>
std::string appendPrefix(
    const std::string &_prefix, const T &_key, Args... others)
{
  if (_prefix.empty())
    return appendPrefix(_key, others...);
  return appendPrefix(_prefix + '.' + _key, others...);
}

std::string keyType(const std::string &_key)
{
  static std::unordered_map<std::string, std::string> keyTypes {
      {"pose", "pose"},
  };

  if (auto it = keyTypes.find(_key); it != keyTypes.end())
  {
    return it->second;
  }
  return "string";
}

// A struct is needed to create a recursive std::variant based data structure
struct Value
{
  using KeyValue = std::map<std::string, Value>;
  // using VariantType = std::variant<std::monostate, sdf::Param, KeyValue>;
  using VariantType = std::variant<KeyValue, sdf::Param>;
  VariantType data;

  public: KeyValue *Map()
  {
    return std::get_if<KeyValue>(&data);
  }

  public: sdf::Param *Param()
  {
    return std::get_if<sdf::Param>(&data);
  }

  public: Value &operator[](const std::string &_key)
  {
    auto keys = sdf::split(_key, ".");
    Value *curValue = &(this->Map()->operator[](keys[0]));

    for (std::size_t i = 1; i < keys.size(); ++i)
    {
      if (auto kval = std::get_if<KeyValue>(&curValue->data))
      {
        curValue = &kval->operator[](keys[i]);
      }
      else
      {
        throw std::runtime_error("Unable to find key [" + _key + "]");
      }
    }
    return *curValue;
  }

  // DEBUG
  // using json = nlohmann::json;

  // public: operator nlohmann::json() const
  // {
  //     if (auto param = std::get_if<sdf::Param>(&data))
  //     {
  //       return json(param->GetAsString());
  //     }
  //     else if (auto kval = std::get_if<Value::KeyValue>(&data))
  //     {
  //       return json(*kval);
  //     }
  //     return {};
  // }

  public: void PrintValue(std::ostream &os, const std::string &_key) const
  {
    std::visit(
        [&](auto &&arg)
        {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, sdf::Param>)
          {
            os << _key << " = " << arg << std::endl;
          }
          else if constexpr (std::is_same_v<T, Value::KeyValue>)
          {
            for (const auto &[key, val] : arg)
            {
              val.PrintValue(os, appendPrefix(_key, key));
            }
          }
        },
        data);
  }

  friend std::ostream& operator<<(std::ostream& os, const Value &_value)
  {
    _value.PrintValue(os, "");
    // nlohmann::json jmap(_doc);
    // os << jmap.dump(2) << std::endl;
    return os;
  }
};

struct Document: public Value
{
  Document()
      : Value({Value::KeyValue()})
  {
  }
};

Document parseToml(const std::string &_filePath, sdf::Errors &_errors)
{
  std::fstream fs;
  fs.open(_filePath);
  if (!fs.good())
  {
    _errors.emplace_back(sdf::ErrorCode::FILE_READ,
        "File [" + _filePath + "] could not be read.");
    return {};
  }

  Document doc;

  auto readValue = [](const std::string &_inp)
  {
    const std::string trimmed = sdf::trim(_inp);
    // Find the quotes
    auto begInd = trimmed.find('"');
    auto endInd = trimmed.rfind('"');
    return trimmed.substr(begInd + 1, endInd - 1);
  };
  auto readTableName = [](const std::string &_inp)
  {
    const std::string trimmed = sdf::trim(_inp);
    // Find the quotes
    auto begInd = trimmed.find('[');
    auto endInd = trimmed.rfind(']');
    return trimmed.substr(begInd + 1, endInd - 1);
  };

  std::string curPrefix = "";
  std::string line;
  while (std::getline(fs, line))
  {
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

        sdf::Param param(key, keyType(key), "", true);
        param.SetFromString(value);

        doc[appendPrefix(curPrefix, key)] = {param};
      }
      else if (line.find('[') != std::string::npos)
      {
        const std::string tableName = readTableName(line);
        if (tableName.empty())
        {
          // ERROR
        }
        else
        {
          curPrefix = tableName;
        }
      }
    }
  }

  fs.close();
  std::cout << "Parsed toml:" << "\n" << doc << std::endl;
  return doc;
}
}

sdf::InterfaceModelPtr customTomlParser(
    const sdf::NestedInclude &_include, sdf::Errors &_errors)
{
  toml::Document doc = toml::parseToml(_include.resolvedFileName, _errors);
  if (_errors.empty())
  {
    const std::string modelName = _include.localModelName.empty()
        ? doc["name"].Param()->GetAsString()
        : _include.localModelName;

    const std::string canonicalLink =
        doc["canonical_link"].Param()->GetAsString();

    // Pose of model (M) in parent (P) frame
    ignition::math::Pose3d X_PM;
    doc["pose"].Param()->Get(X_PM);

    // Pose of canonical link (C) in model (M) frame
    ignition::math::Pose3d X_MC;
    doc[toml::appendPrefix("links", canonicalLink, "pose")].Param()->Get(X_MC);

    auto model = std::make_shared<sdf::InterfaceModel>(
        modelName, canonicalLink, X_MC.Inverse(), X_PM);
    for (auto &[name, link] : *doc["links"].Map())
    {
      ignition::math::Pose3d pose;
      link["pose"].Param()->Get(pose);
      // model->AddLink({name, pose});
    }
    for (auto &[name, joint] : *doc["joints"].Map())
    {
      ignition::math::Pose3d pose;
      if (auto poseParam = joint["pose"].Param())
        poseParam->Get(pose);
      const std::string parent =
          joint["parent"].Param()->GetAsString();
      const std::string child =
          joint["child"].Param()->GetAsString();
      // model->AddJoint({name, pose, parent, child});
    }
    return model;
  }
  return nullptr;
}

/////////////////////////////////////////////////
TEST(InterfaceAPI, NestedIncludeData)
{
  const std::string modelDir = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "integration", "model");

  const std::string testSdf = R"(
<sdf version="1.8">
  <world name="default">
    <include>
      <uri>test_file.yaml</uri>
      <name>box</name>
      <pose>1 0 0 0 0 0</pose>
      <extra>
        <info1>value1</info1>
        <info2>value2</info2>
      </extra>
      <static>1</static>
    </include>
    <include>
      <uri>test_file.toml</uri>
    </include>
  </world>
</sdf>)";

  sdf::ParserConfig config;
  config.SetFindCallback(
      [&](const std::string &_file)
      {
        return sdf::filesystem::append(modelDir, _file);
      });

  auto testYamlParser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    if (_include.uri.find(".yaml") == std::string::npos)
      return nullptr;

    const std::string fileName = "test_file.yaml";
    EXPECT_EQ(fileName, _include.uri);
    EXPECT_EQ(
        sdf::filesystem::append(modelDir, fileName), _include.resolvedFileName);
    EXPECT_EQ("box", _include.localModelName);
    EXPECT_TRUE(_include.isStatic);
    EXPECT_TRUE(_include.virtualCustomElements->HasElement("extra"));

    auto extra = _include.virtualCustomElements->GetElement("extra");
    EXPECT_TRUE(extra->HasElement("info1"));
    EXPECT_TRUE(extra->HasElement("info2"));
    EXPECT_EQ("value1", extra->Get<std::string>("info1"));
    EXPECT_EQ("value2", extra->Get<std::string>("info2"));

    // Add error for test expectation later on.
    _errors.emplace_back(
        sdf::ErrorCode::URI_INVALID, "Test YAML error message");
    return nullptr;
  };

  auto testTomlParser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    if (_include.uri.find(".toml") == std::string::npos)
      return nullptr;
    const std::string fileName = "test_file.toml";
    EXPECT_EQ(fileName, _include.uri);
    EXPECT_EQ(
        sdf::filesystem::append(modelDir, fileName), _include.resolvedFileName);
    EXPECT_EQ("", _include.localModelName);
    EXPECT_FALSE(_include.isStatic);
    EXPECT_EQ(nullptr, _include.virtualCustomElements->GetFirstElement());

    // Add error for test expectation later on.
    _errors.emplace_back(
        sdf::ErrorCode::URI_INVALID, "Test TOML error message");
    return nullptr;
  };

  config.RegisterCustomModelParser(testYamlParser);
  config.RegisterCustomModelParser(testTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, config);
  ASSERT_EQ(3u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::URI_INVALID, errors[0].Code());
  EXPECT_EQ("Test YAML error message", errors[0].Message());
  EXPECT_EQ(sdf::ErrorCode::URI_INVALID, errors[1].Code());
  EXPECT_EQ("Test TOML error message", errors[1].Message());
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  auto interfaceModel = world->InterfaceModelByIndex(0);
  EXPECT_EQ(nullptr, interfaceModel);
}

/////////////////////////////////////////////////
TEST(InterfaceAPI, IncludeTOMLFile)
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
  EXPECT_EQ("double_pendulum", interfaceModel->Name());
  EXPECT_EQ("base", interfaceModel->CanonicalLinkName());
  using ignition::math::Pose3d;
  EXPECT_EQ(
      Pose3d(1, 0, 0, 0, 0, 0), interfaceModel->ModelFramePoseInParentFrame());
  EXPECT_EQ(Pose3d(-1, 0, -0.5, 0, 0, 0),
      interfaceModel->ModelFramePoseInCanonicalLinkFrame());
}
