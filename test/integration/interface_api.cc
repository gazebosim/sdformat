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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <variant>

#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

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
      {"bool", "bool"},
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
  using VariantType = std::variant<KeyValue, sdf::Param>;
  VariantType data;

  public: KeyValue &Map()
  {
    return std::get<KeyValue>(this->data);
  }

  public: template <typename T>
  T ParamGet(const T &_defaultVal = T {})
  {
    T out{_defaultVal};
    if (auto param = std::get_if<sdf::Param>(&this->data))
    {
      param->Get(out);
    }
    return out;
  }

  public: Value &operator[](const std::string &_key)
  {
    auto keys = sdf::split(_key, ".");
    Value *curValue = &(this->Map()[keys[0]]);

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
          throw std::runtime_error("Empty table name encountered");
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

sdf::InterfaceModelPtr parseModel(toml::Value &_doc,
    const std::string &_modelName, const std::optional<bool> &_isStatic)
{
  const auto canonicalLink = _doc["canonical_link"].ParamGet<std::string>();

  // Pose of model (M) in parent (P) frame
  const auto X_PM = _doc["pose"].ParamGet<ignition::math::Pose3d>();

  // Pose of canonical link (C) in model (M) frame
  const auto X_MC = _doc[toml::appendPrefix("links", canonicalLink, "pose")]
                        .ParamGet<ignition::math::Pose3d>();

  bool isStaticActual = _isStatic.has_value()
      ? *_isStatic
      : _doc["static"].ParamGet<bool>(false);

  auto model = std::make_shared<sdf::InterfaceModel>(
      _modelName, isStaticActual, canonicalLink, X_MC.Inverse(), X_PM);

  for (auto &[name, link] : _doc["links"].Map())
  {
    const auto pose = link["pose"].ParamGet<ignition::math::Pose3d>();
    model->AddLink({name, pose});
  }

  for (auto &[name, frame] : _doc["frames"].Map())
  {
    const auto attachedTo = frame["attached_to"].ParamGet<std::string>();
    const auto pose = frame["pose"].ParamGet<ignition::math::Pose3d>();
    model->AddFrame({name, attachedTo, pose});
  }

  for (auto &[name, joint] : _doc["joints"].Map())
  {
    const auto pose = joint["pose"].ParamGet<ignition::math::Pose3d>();
    const auto child = joint["child"].ParamGet<std::string>();
    model->AddJoint({name, child, pose});
  }
  for (auto &[name, nestedModel] : _doc["models"].Map())
  {
    model->AddNestedModel(parseModel(nestedModel, name, std::nullopt));
  }

  return model;
}

sdf::InterfaceModelPtr customTomlParser(
    const sdf::NestedInclude &_include, sdf::Errors &_errors)
{
  toml::Document doc = toml::parseToml(_include.resolvedFileName, _errors);
  if (_errors.empty())
  {
    const std::string modelName = _include.localModelName.empty()
        ? doc["name"].ParamGet<std::string>()
        : _include.localModelName;

    return parseModel(doc, modelName, _include.isStatic);
  }
  return nullptr;
}

class InterfaceAPI : public ::testing::Test
{
  // cppcheck-suppress unusedFunction
  protected: void SetUp() override
  {
    this->modelDir = sdf::filesystem::append(
        PROJECT_SOURCE_PATH, "test", "integration", "model");

    this->config.SetFindCallback(
        [=](const std::string &_file)
        {
          return sdf::filesystem::append(modelDir, _file);
        });
  }

  public: std::string modelDir;
  public: sdf::ParserConfig config;
};

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, NestedIncludeData)
{
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

  auto testYamlParser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    if (_include.uri.find(".yaml") == std::string::npos)
      return nullptr;

    const std::string fileName = "test_file.yaml";
    EXPECT_EQ(fileName, _include.uri);
    EXPECT_EQ(sdf::filesystem::append(this->modelDir, fileName),
        _include.resolvedFileName);
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

  this->config.RegisterCustomModelParser(testYamlParser);
  this->config.RegisterCustomModelParser(testTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
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
TEST_F(InterfaceAPI, TomlParser)
{
  using ignition::math::Pose3d;
  const std::string testFile = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "sdf", "include_with_interface_api.sdf");

  this->config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->InterfaceModelCount());
  auto interfaceModel = world->InterfaceModelByIndex(0);
  ASSERT_NE(nullptr, interfaceModel);
  EXPECT_EQ("double_pendulum", interfaceModel->Name());
  EXPECT_EQ("base", interfaceModel->CanonicalLinkName());
  EXPECT_EQ(
      Pose3d(1, 0, 0, 0, 0, 0), interfaceModel->ModelFramePoseInParentFrame());
  EXPECT_EQ(Pose3d(-1, 0, -0.5, 0, 0, 0),
      interfaceModel->ModelFramePoseInCanonicalLinkFrame());

  EXPECT_EQ(3u, interfaceModel->Links().size());
  std::map <std::string, Pose3d> expLinks = {
      {"base", Pose3d(1, 0, 0.5, 0, 0, 0)},
      {"upper_link", Pose3d(0, 0, 2.1, -1.5708, 0, 0)},
      {"lower_link", Pose3d(0.25, 1.0, 2.1, -2, 0, 0)},
  };

  for (const auto &link : interfaceModel->Links())
  {
    ASSERT_EQ(1u, expLinks.count(link.Name()));
    EXPECT_EQ(expLinks[link.Name()], link.PoseInModelFrame());
  }
  std::map <std::string, std::pair<std::string, Pose3d>> expFrames = {
      {"frame_1", {"", Pose3d(0, 1, 0.0, 0, 0, 0)}},
      {"frame_2", {"lower_link", Pose3d(0, 0, 1, 0, 0, 0)}},
  };

  EXPECT_EQ(2u, interfaceModel->Frames().size());
  for (const auto &frame : interfaceModel->Frames())
  {
    ASSERT_EQ(1u, expFrames.count(frame.Name()));
    EXPECT_EQ(expFrames[frame.Name()].first, frame.AttachedTo());
    EXPECT_EQ(expFrames[frame.Name()].second, frame.PoseInAttachedToFrame());
  }

  std::map <std::string, std::pair<std::string, Pose3d>> expJoints = {
      {"upper_joint", {"upper_link", Pose3d(0.001, 0, 0, 0, 0, 0)}},
      {"lower_joint", {"lower_link", Pose3d(0, 0.001, 0, 0, 0, 0)}},
  };
  EXPECT_EQ(2u, interfaceModel->Joints().size());
  for (const auto &joint : interfaceModel->Joints())
  {
    ASSERT_EQ(1u, expJoints.count(joint.Name()));
    EXPECT_EQ(expJoints[joint.Name()].first, joint.ChildName());
    EXPECT_EQ(expJoints[joint.Name()].second, joint.PoseInChildFrame());
  }

  EXPECT_EQ(2u, interfaceModel->NestedModels().size());
  std::map <std::string, Pose3d> expNestedModels = {
      {"child_model", Pose3d(2, 0, 0, 0, 0, 0)},
      {"child_dp", Pose3d(3, 0, 0, 0, 0, 0)},
  };
  for (const auto &nestedModel : interfaceModel->NestedModels())
  {
    ASSERT_EQ(1u, expNestedModels.count(nestedModel->Name()));
    EXPECT_EQ(expNestedModels[nestedModel->Name()],
        nestedModel->ModelFramePoseInParentFrame());
  }
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, FrameSemantics)
{
  using ignition::math::Pose3d;
  const std::string testFile = sdf::filesystem::append(PROJECT_SOURCE_PATH,
      "test", "sdf", "include_with_interface_api_frame_semantics.sdf");

  this->config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty()) << errors;

  // std::cout << root.PoseGraph() << std::endl;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->InterfaceModelCount());

  auto resolvePoseNoErrors =
      [](const sdf::SemanticPose &_semPose, const std::string _relativeTo = "")
  {
    Pose3d pose;
    sdf::Errors resolveErrors = _semPose.Resolve(pose, _relativeTo);
    EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
    return pose;
  };
  auto resolveAttachedToNoErrors = [](const sdf::Frame &_frame)
  {
    std::string resolvedBody;
    sdf::Errors resolveErrors = _frame.ResolveAttachedToBody(resolvedBody);
    EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
    return resolvedBody;
  };

  const Pose3d dpPose(1, 0, 0, 0, 0, 0);
  {
    const sdf::Frame *frame = world->FrameByName("F1");
    const sdf::Frame *frameAttach = world->FrameByName("F1_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d expPose = dpPose;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::base", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F2");
    const sdf::Frame *frameAttach = world->FrameByName("F2_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d upperLinkPose = Pose3d(0, 0, 2.1, -1.5708, 0, 0);
    const Pose3d expPose = dpPose * upperLinkPose;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ(
        "double_pendulum::upper_link", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F3");
    const sdf::Frame *frameAttach = world->FrameByName("F3_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d upperLinkPose = Pose3d(0, 0, 2.1, -1.5708, 0, 0);
    const Pose3d upperJointPose = Pose3d(0.001, 0, 0, 0, 0, 0);
    const Pose3d expPose = dpPose * upperLinkPose * upperJointPose;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ(
        "double_pendulum::upper_link", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F4");
    const sdf::Frame *frameAttach = world->FrameByName("F4_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d frame1Pose = Pose3d(0, 1, 0.0, 0, 0, 0);
    const Pose3d expPose = dpPose * frame1Pose;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ(
        "double_pendulum::base", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F5");
    const sdf::Frame *frameAttach = world->FrameByName("F5_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childModel = Pose3d(2, 0, 0, 0, 0, 0);
    const Pose3d expPose = dpPose * childModel;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_model::base_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F6");
    const sdf::Frame *frameAttach = world->FrameByName("F6_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childModel = Pose3d(2, 0, 0, 0, 0, 0);
    const Pose3d childModelBaseLink = Pose3d(1, 0, 0, 0, 0, 0);
    const Pose3d expPose = dpPose * childModel * childModelBaseLink;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_model::base_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F7");
    const sdf::Frame *frameAttach = world->FrameByName("F7_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childDp = Pose3d(3, 0, 0, 0, 0, 0);
    const Pose3d childDpModel1 = Pose3d(0, 0, 0, 0, 0, 0);
    const Pose3d expPose = dpPose * childDp * childDpModel1;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_dp::model_1::lower_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F8");
    const sdf::Frame *frameAttach = world->FrameByName("F8_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childDp = Pose3d(3, 0, 0, 0, 0, 0);
    const Pose3d childDpModel1 = Pose3d(0, 0, 0, 0, 0, 0);
    const Pose3d childDpModel1LowerLink = Pose3d(0.25, 1.0, 2.1, -2, 0, 0);
    const Pose3d expPose =
        dpPose * childDp * childDpModel1 * childDpModel1LowerLink;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_dp::model_1::lower_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F9");
    const sdf::Frame *frameAttach = world->FrameByName("F9_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childDp = Pose3d(3, 0, 0, 0, 0, 0);
    const Pose3d childDpModel1 = Pose3d(0, 0, 0, 0, 0, 0);
    const Pose3d childDpModel1LowerLink = Pose3d(0.25, 1.0, 2.1, -2, 0, 0);
    // childDpLowerJoint is relative to childDpModel1LowerLink
    const Pose3d childDpLowerJoint = Pose3d(0, 0.001, 0, 0, 0, 0);
    const Pose3d expPose = dpPose * childDp * childDpModel1 *
        childDpModel1LowerLink * childDpLowerJoint;
    EXPECT_EQ(expPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_dp::model_1::lower_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
}
