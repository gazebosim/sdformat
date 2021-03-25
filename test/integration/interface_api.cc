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

/// \file
/// Provides a simple usage of the interface API by parsing models described in
/// TOML (https://toml.io/en/).

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
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
#include "toml_parser.hh"

sdf::InterfaceModelPtr parseModel(toml::Value &_doc,
    const std::string &_modelName)
{
  const auto canonicalLink = _doc["canonical_link"].ParamGet<std::string>();

  // Pose of model (M) in parent (P) frame
  const auto X_PM = _doc["pose"].ParamGet<ignition::math::Pose3d>();

  bool isStatic = _doc["static"].ParamGet<bool>(false);

  auto model = std::make_shared<sdf::InterfaceModel>(
      _modelName, nullptr, isStatic, canonicalLink, X_PM);

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
    model->AddNestedModel(parseModel(nestedModel, name));
  }

  return model;
}

sdf::InterfaceModelPtr customTomlParser(
    const sdf::NestedInclude &_include, sdf::Errors &_errors)
{
  toml::Document doc = toml::parseToml(_include.resolvedFileName, _errors);
  if (_errors.empty())
  {
    const std::string modelName =
        _include.localModelName.value_or(doc["name"].ParamGet<std::string>());

    if (_include.isStatic.has_value())
    {
      // if //include/static is set, override the value in the inluded model
      sdf::Param param("static", "bool", "false", false);
      param.Set(*_include.isStatic);
      doc["static"] = {param};
    }
    if (_include.includeRawPose.has_value())
    {
      // if //include/static is set, override the value in the inluded model
      sdf::Param poseParam("pose", "pose", "", false);
      poseParam.Set(*_include.includeRawPose);
      doc["pose"] = {poseParam};
    }

    return parseModel(doc, modelName);
  }
  return nullptr;
}


bool endsWith(const std::string &_str, const std::string &_suffix)
{
  if (_suffix.size() > _str.size())
  {
    return false;
  }
  const std::size_t startPos = _str.size() - _suffix.size();
  return _str.compare(startPos, _suffix.size(), _suffix) == 0;
}

class InterfaceAPI : public ::testing::Test
{
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
    <frame name="F1"/>
    <include>
      <uri>file_wont_be_parsed.nonce_1</uri>
      <name>box</name>
      <pose relative_to="F1">1 0 0 0 0 0</pose>
      <extra>
        <info1>value1</info1>
        <info2>value2</info2>
      </extra>
      <static>1</static>
    </include>
    <include>
      <uri>file_wont_be_parsed.nonce_2</uri>
    </include>
  </world>
</sdf>)";

  auto testNonce1Parser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    if (!endsWith(_include.uri, ".nonce_1"))
      return nullptr;

    const std::string fileName = "file_wont_be_parsed.nonce_1";
    EXPECT_EQ(fileName, _include.uri);
    EXPECT_EQ(sdf::filesystem::append(this->modelDir, fileName),
        _include.resolvedFileName);
    EXPECT_EQ("box", *_include.localModelName);
    EXPECT_TRUE(_include.isStatic.has_value());
    EXPECT_TRUE(_include.isStatic.value());

    EXPECT_TRUE(_include.includeRawPose.has_value());
    EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0),
        _include.includeRawPose.value());

    EXPECT_TRUE(_include.includePoseRelativeTo.has_value());
    EXPECT_EQ("F1", _include.includePoseRelativeTo.value());
    EXPECT_TRUE(_include.includeElement->HasElement("extra"));

    auto extra = _include.includeElement->GetElement("extra");
    EXPECT_TRUE(extra->HasElement("info1"));
    EXPECT_TRUE(extra->HasElement("info2"));
    EXPECT_EQ("value1", extra->Get<std::string>("info1"));
    EXPECT_EQ("value2", extra->Get<std::string>("info2"));

    // Add error for test expectation later on.
    _errors.emplace_back(
        sdf::ErrorCode::URI_INVALID, "Test nonce_1 error message");
    return nullptr;
  };

  auto testNonce2Parser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    if (!endsWith(_include.uri, ".nonce_2"))
      return nullptr;
    const std::string fileName = "file_wont_be_parsed.nonce_2";
    EXPECT_EQ(fileName, _include.uri);
    EXPECT_EQ(
        sdf::filesystem::append(modelDir, fileName), _include.resolvedFileName);
    EXPECT_FALSE(_include.localModelName.has_value());
    EXPECT_FALSE(_include.isStatic);

    // Add error for test expectation later on.
    _errors.emplace_back(
        sdf::ErrorCode::URI_INVALID, "Test nonce_2 error message");
    return nullptr;
  };

  this->config.RegisterCustomModelParser(testNonce1Parser);
  this->config.RegisterCustomModelParser(testNonce2Parser);
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  ASSERT_EQ(3u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::URI_INVALID, errors[0].Code());
  EXPECT_EQ("Test nonce_1 error message", errors[0].Message());
  EXPECT_EQ(sdf::ErrorCode::URI_INVALID, errors[1].Code());
  EXPECT_EQ("Test nonce_2 error message", errors[1].Message());
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  auto interfaceModel = world->InterfaceModelByIndex(0);
  EXPECT_EQ(nullptr, interfaceModel);
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, CustomParserPrecedence)
{
  const std::string testSdf = R"(
<sdf version="1.8">
  <world name="default">
    <include>
      <uri>file_wont_be_parsed.nonce_1</uri>
      <name>box</name>
    </include>
  </world>
</sdf>)";

  std::vector<int> customParserCallOrder;
  auto createCustomParser = [&](int _parserId)
  {
    auto testParser = [=, &customParserCallOrder](const sdf::NestedInclude &,
                          sdf::Errors &) -> sdf::InterfaceModelPtr
    {
      customParserCallOrder.push_back(_parserId);
      if (_parserId <= 1)
      {
        // Return a model if the parserId is 1 or 0. This would indicate the
        // custom parser was successful. However, since parser 1 will be visited
        // first, we do not expect parser 0 to be called.
        auto model = std::make_shared<sdf::InterfaceModel>(
            "test_model" + std::to_string(_parserId), nullptr, false,
            "base_link", ignition::math::Pose3d());
        model->AddLink({"base_link", {}});
        return model;
      }
      return nullptr;
    };
    return testParser;
  };

  this->config.RegisterCustomModelParser(createCustomParser(0));
  this->config.RegisterCustomModelParser(createCustomParser(1));
  this->config.RegisterCustomModelParser(createCustomParser(2));
  this->config.RegisterCustomModelParser(createCustomParser(3));
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty());
  // Check that custom parsers are called in reverse order of registration.
  ASSERT_EQ(3u, customParserCallOrder.size());
  // Parser 0 will not be visited because parser 1 will be successful.
  const std::vector<int> customParserCallOrderExpected {3, 2, 1};
  EXPECT_EQ(customParserCallOrderExpected, customParserCallOrder);
}

/////////////////////////////////////////////////
void TomlParserTest(const sdf::InterfaceModelConstPtr &_interfaceModel)
{
  using ignition::math::Pose3d;
  ASSERT_NE(nullptr, _interfaceModel);
  EXPECT_EQ("double_pendulum", _interfaceModel->Name());
  EXPECT_EQ("base", _interfaceModel->CanonicalLinkName());
  EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0),
      _interfaceModel->ModelFramePoseInParentFrame());

  EXPECT_EQ(3u, _interfaceModel->Links().size());
  std::map <std::string, Pose3d> expLinks = {
      {"base", Pose3d(1, 0, 0.5, 0, 0, 0)},
      {"upper_link", Pose3d(0, 0, 2.1, -1.5708, 0, 0)},
      {"lower_link", Pose3d(0.25, 1.0, 2.1, -2, 0, 0)},
  };

  for (const auto &link : _interfaceModel->Links())
  {
    ASSERT_EQ(1u, expLinks.count(link.Name()));
    EXPECT_EQ(expLinks[link.Name()], link.PoseInModelFrame());
  }
  std::map <std::string, std::pair<std::string, Pose3d>> expFrames = {
      {"frame_1", {"", Pose3d(0, 1, 0.0, 0, 0, 0)}},
      {"frame_2", {"lower_link", Pose3d(0, 0, 1, 0, 0, 0)}},
  };

  EXPECT_EQ(2u, _interfaceModel->Frames().size());
  for (const auto &frame : _interfaceModel->Frames())
  {
    ASSERT_EQ(1u, expFrames.count(frame.Name()));
    EXPECT_EQ(expFrames[frame.Name()].first, frame.AttachedTo());
    EXPECT_EQ(expFrames[frame.Name()].second, frame.PoseInAttachedToFrame());
  }

  std::map <std::string, std::pair<std::string, Pose3d>> expJoints = {
      {"upper_joint", {"upper_link", Pose3d(0.001, 0, 0, 0, 0, 0)}},
      {"lower_joint", {"lower_link", Pose3d(0, 0.001, 0, 0, 0, 0)}},
  };
  EXPECT_EQ(2u, _interfaceModel->Joints().size());
  for (const auto &joint : _interfaceModel->Joints())
  {
    ASSERT_EQ(1u, expJoints.count(joint.Name()));
    EXPECT_EQ(expJoints[joint.Name()].first, joint.ChildName());
    EXPECT_EQ(expJoints[joint.Name()].second, joint.PoseInChildFrame());
  }

  EXPECT_EQ(2u, _interfaceModel->NestedModels().size());
  std::map <std::string, Pose3d> expNestedModels = {
      {"child_model", Pose3d(2, 0, 0, 0, 0, 0)},
      {"child_dp", Pose3d(3, 0, 0, 0, 0, 0)},
  };
  for (const auto &nestedModel : _interfaceModel->NestedModels())
  {
    ASSERT_EQ(1u, expNestedModels.count(nestedModel->Name()));
    EXPECT_EQ(expNestedModels[nestedModel->Name()],
        nestedModel->ModelFramePoseInParentFrame());
  }
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, TomlParserWorldInclude)
{
  using ignition::math::Pose3d;
  const std::string testFile = sdf::filesystem::append(PROJECT_SOURCE_PATH,
      "test", "sdf", "world_include_with_interface_api.sdf");

  this->config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  ASSERT_EQ(1u, world->InterfaceModelCount());
  auto interfaceModel = world->InterfaceModelByIndex(0);
  SCOPED_TRACE("TomlParserWorldInclude");
  TomlParserTest(interfaceModel);
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, TomlParserModelInclude)
{
  using ignition::math::Pose3d;
  const std::string testFile = sdf::filesystem::append(PROJECT_SOURCE_PATH,
      "test", "sdf", "model_include_with_interface_api.sdf");

  this->config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(1u, model->InterfaceModelCount());
  auto interfaceModel = model->InterfaceModelByIndex(0);
  ASSERT_NE(nullptr, interfaceModel);
  SCOPED_TRACE("TomlParserModelInclude");
  TomlParserTest(interfaceModel);
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

  {
    const sdf::Frame * frame= world->FrameByName("F0");
    ASSERT_NE(nullptr, frame);
    // The pose of F0 relative to the double_pendulum interface model is the
    // inverse of the raw pose of double_pendulum
    const Pose3d expectedPose(-1, 0, 0, 0, 0, 0);
    EXPECT_EQ(expectedPose,
        resolvePoseNoErrors(frame->SemanticPose(), "double_pendulum"));
  }

  const Pose3d doublePendulumPose(1, 2, 0, 0, 0, 0);
  {
    const sdf::Frame *frame = world->FrameByName("F1");
    const sdf::Frame *frameAttach = world->FrameByName("F1_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d expectedPose = doublePendulumPose;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::base", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F2");
    const sdf::Frame *frameAttach = world->FrameByName("F2_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d upperLinkPose = Pose3d(0, 0, 2.1, -1.5708, 0, 0);
    const Pose3d expectedPose = doublePendulumPose * upperLinkPose;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
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
    const Pose3d expectedPose =
        doublePendulumPose * upperLinkPose * upperJointPose;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ(
        "double_pendulum::upper_link", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F4");
    const sdf::Frame *frameAttach = world->FrameByName("F4_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d frame1Pose = Pose3d(0, 1, 0.0, 0, 0, 0);
    const Pose3d expectedPose = doublePendulumPose * frame1Pose;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ(
        "double_pendulum::base", resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F5");
    const sdf::Frame *frameAttach = world->FrameByName("F5_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childModel = Pose3d(2, 0, 0, 0, 0, 0);
    const Pose3d expectedPose = doublePendulumPose * childModel;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
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
    const Pose3d expectedPose =
        doublePendulumPose * childModel * childModelBaseLink;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_model::base_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F7");
    const sdf::Frame *frameAttach = world->FrameByName("F7_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childDoublePendulum = Pose3d(3, 0, 0, 0, 0, 0);
    const Pose3d childDoublePendulumModel1 = Pose3d(0, 0, 0, 0, 0, 0);
    const Pose3d expectedPose =
        doublePendulumPose * childDoublePendulum * childDoublePendulumModel1;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_dp::model_1::lower_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F8");
    const sdf::Frame *frameAttach = world->FrameByName("F8_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childDoublePendulum = Pose3d(3, 0, 0, 0, 0, 0);
    const Pose3d childDoublePendulumModel1 = Pose3d(0, 0, 0, 0, 0, 0);
    const Pose3d childDoublePendulumModel1LowerLink =
        Pose3d(0.25, 1.0, 2.1, -2, 0, 0);
    const Pose3d expectedPose = doublePendulumPose * childDoublePendulum *
        childDoublePendulumModel1 * childDoublePendulumModel1LowerLink;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_dp::model_1::lower_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
  {
    const sdf::Frame *frame = world->FrameByName("F9");
    const sdf::Frame *frameAttach = world->FrameByName("F9_attach");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frameAttach);
    const Pose3d childDoublePendulum = Pose3d(3, 0, 0, 0, 0, 0);
    const Pose3d childDoublePendulumModel1 = Pose3d(0, 0, 0, 0, 0, 0);
    const Pose3d childDoublePendulumModel1LowerLink =
        Pose3d(0.25, 1.0, 2.1, -2, 0, 0);
    // childDpLowerJoint is relative to childDpModel1LowerLink
    const Pose3d childDoublePendulumLowerJoint = Pose3d(0, 0.001, 0, 0, 0, 0);
    const Pose3d expectedPose = doublePendulumPose * childDoublePendulum *
        childDoublePendulumModel1 * childDoublePendulumModel1LowerLink *
        childDoublePendulumLowerJoint;
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frame->SemanticPose()));
    EXPECT_EQ(expectedPose, resolvePoseNoErrors(frameAttach->SemanticPose()));
    EXPECT_EQ("double_pendulum::child_dp::model_1::lower_link",
        resolveAttachedToNoErrors(*frameAttach));
  }
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, Reposturing)
{
  using ignition::math::Pose3d;
  const std::string testFile = sdf::filesystem::append(PROJECT_SOURCE_PATH,
      "test", "sdf", "include_with_interface_api_reposture.sdf");

  std::unordered_map<std::string, sdf::InterfaceModelPtr> models;
  std::unordered_map<std::string, Pose3d> posesAfterReposture;

  // Create a resposture callback function for a given absolute model name. The
  // name is used to store poses in `posesAfterReposture` as well as to lookup
  // interface models in `models`
  auto makeRepostureFunc = [&](const std::string &_absoluteName)
  {
    auto repostureFunc =
        [modelName = _absoluteName, &models, &posesAfterReposture](
            const sdf::InterfaceModelPoseGraph &_graph)
    {
      {
        ignition::math::Pose3d pose;
        sdf::Errors errors =
            _graph.ResolveNestedModelFramePoseInWorldFrame(pose);
        EXPECT_TRUE(errors.empty()) << errors;
        posesAfterReposture[modelName] = pose;
      }

      auto modelIt = models.find(modelName);
      if (modelIt != models.end())
      {
        for (const auto &link : modelIt->second->Links())
        {
          ignition::math::Pose3d pose;
          sdf::Errors errors = _graph.ResolveNestedFramePose(pose, link.Name());
          EXPECT_TRUE(errors.empty()) << errors;
          posesAfterReposture[sdf::JoinName(modelName, link.Name())] = pose;
        }
      }
    };
    return repostureFunc;
  };

  auto repostureTestParser = [&](const sdf::NestedInclude &_include,
                                 sdf::Errors &) -> sdf::InterfaceModelPtr
  {
    bool fileHasCorrectSuffix = endsWith(_include.resolvedFileName, ".nonce_1");
    EXPECT_TRUE(fileHasCorrectSuffix) << "File: " << _include.resolvedFileName;
    if (!fileHasCorrectSuffix)
      return nullptr;

    const std::string absoluteModelName =
        sdf::JoinName(_include.absoluteParentName, *_include.localModelName);

    auto model = std::make_shared<sdf::InterfaceModel>(*_include.localModelName,
        makeRepostureFunc(absoluteModelName), false, "base_link",
        _include.includeRawPose.value_or(Pose3d {}));
    model->AddLink({"base_link", {}});
    models[absoluteModelName] = model;

    const std::string absoluteNestedModelName =
        sdf::JoinName(absoluteModelName, "nested_model");
    auto nestedModel = std::make_shared<sdf::InterfaceModel>("nested_model",
        makeRepostureFunc(absoluteNestedModelName), false, "nested_link",
        Pose3d(3, 0, 0, 0, 0, 0));
    nestedModel->AddLink({"nested_link", Pose3d(0, 0, 0, 0.1, 0, 0)});
    models[absoluteNestedModelName] = nestedModel;

    model->AddNestedModel(nestedModel);
    return model;
  };

  this->config.RegisterCustomModelParser(repostureTestParser);
  this->config.SetFindCallback(
      [](const auto &_fileName)
      {
        return _fileName;
      });
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  auto checkPose =
      [&posesAfterReposture](
          const std::string &_name, const Pose3d &_expectedPose)
  {
    auto it = posesAfterReposture.find(_name);
    if (it == posesAfterReposture.end())
      return testing::AssertionFailure() << _name << " not found in map";

    if (_expectedPose != it->second)
    {
      return testing::AssertionFailure()
          << "Expected pose: " << _expectedPose << " actual: " << it->second;
    }

    return testing::AssertionSuccess();
  };
  // There are two included models using a custom parser.
  // In each of the included models, there are two models and two links.
  ASSERT_EQ(8u, posesAfterReposture.size());
  EXPECT_TRUE(checkPose("M0", {1, 2, 0, 0, 0, 0}));
  EXPECT_TRUE(checkPose("M0::base_link", {1, 2, 0, 0, 0, 0}));
  EXPECT_TRUE(checkPose("M0::nested_model", {4, 2, 0, 0, 0, 0}));
  EXPECT_TRUE(checkPose("M0::nested_model::nested_link", {4, 2, 0, 0.1, 0, 0}));
  EXPECT_TRUE(checkPose("parent_model::M1", {1, 2, 3, 0.1, 0, 0}));
  EXPECT_TRUE(checkPose("parent_model::M1::base_link", {1, 2, 3, 0.1, 0, 0}));
  EXPECT_TRUE(
      checkPose("parent_model::M1::nested_model", {4, 2, 3, 0.1, 0, 0}));
  EXPECT_TRUE(checkPose(
      "parent_model::M1::nested_model::nested_link", {4, 2, 3, 0.2, 0, 0}));
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, PlacementFrame)
{
  using ignition::math::Pose3d;
  std::unordered_map<std::string, Pose3d> modelPosesAfterReposture;
  auto repostureTestParser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &)
  {
    std::string modelName =
        sdf::JoinName(_include.absoluteParentName, *_include.localModelName);
    auto repostureFunc = [modelName = modelName, &modelPosesAfterReposture](
                             const sdf::InterfaceModelPoseGraph &_graph)
    {
      ignition::math::Pose3d pose;
      sdf::Errors errors = _graph.ResolveNestedModelFramePoseInWorldFrame(pose);
      EXPECT_TRUE(errors.empty()) << errors;
      modelPosesAfterReposture[modelName] = pose;
    };

    auto model = std::make_shared<sdf::InterfaceModel>(*_include.localModelName,
        repostureFunc, false, "base_link",
        _include.includeRawPose.value_or(Pose3d {}));
    model->AddLink({"base_link", Pose3d(0, 1, 0, 0, 0, 0)});
    model->AddFrame({"frame_1", "__model__", Pose3d(0, 0, 1, 0, 0, 0)});
    return model;
  };

  this->config.RegisterCustomModelParser(repostureTestParser);
  this->config.SetFindCallback(
      [](const auto &_fileName)
      {
        return _fileName;
      });

  // ---------------- Placement frame in //world/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.8">
    <world name="default">
      <include>
        <uri>non_existent_file.test</uri>
        <name>test_model</name>
        <pose>1 0 0 0 0 0</pose>
        <placement_frame>frame_1</placement_frame>
      </include>
      <frame name="test_frame"/>
    </world>
  </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
    EXPECT_TRUE(errors.empty()) << errors;
    const auto *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    const auto *testFrame = world->FrameByIndex(0);
    ASSERT_NE(nullptr, testFrame);
    {
      // Since there is no InterfaceModel::SemanticPose, we resolve the pose of
      // test_frame relative to the test_model and take the inverse as the pose
      // of the test_model relative to the world.
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "test_model");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, -1, 0, 0, 0), pose.Inverse());
    }
    {
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "test_model::frame_1");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), pose.Inverse());
    }
  }
  // ---------------- Placement frame in //sdf/model/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.8">
    <model name="parent_model">
      <include>
        <uri>non_existent_file.test</uri>
        <name>test_model</name>
        <pose>1 0 0 0 0 0</pose>
        <placement_frame>frame_1</placement_frame>
      </include>
      <frame name="test_frame"/>
    </model>
  </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
    EXPECT_TRUE(errors.empty()) << errors;
    const auto *parentModel = root.Model();
    ASSERT_NE(nullptr, parentModel);
    const auto *testFrame = parentModel->FrameByIndex(0);
    ASSERT_NE(nullptr, testFrame);
    {
      // Since there is no InterfaceModel::SemanticPose, we resolve the pose of
      // test_frame relative to the test_model and take the inverse as the pose
      // of the test_model relative to the world.
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "test_model");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, -1, 0, 0, 0), pose.Inverse());
    }
    {
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "test_model::frame_1");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), pose.Inverse());
    }
  }

  // ---------------- Placement frame in //world//model/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.8">
    <world name="default">
      <model name="parent_model">
        <include>
          <uri>non_existent_file.test</uri>
          <name>test_model</name>
          <pose>1 0 0 0 0 0</pose>
          <placement_frame>frame_1</placement_frame>
        </include>
        <frame name="test_frame"/>
      </model>
    </world>
  </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
    EXPECT_TRUE(errors.empty()) << errors;
    const auto *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    const auto *parentModel = world->ModelByIndex(0);
    ASSERT_NE(nullptr, parentModel);
    const auto *testFrame = parentModel->FrameByIndex(0);
    ASSERT_NE(nullptr, testFrame);
    {
      // Since there is no InterfaceModel::SemanticPose, we resolve the pose of
      // test_frame relative to the test_model and take the inverse as the pose
      // of the test_model relative to the world.
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "test_model");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, -1, 0, 0, 0), pose.Inverse());
    }
    {
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "test_model::frame_1");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), pose.Inverse());
    }
  }
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, NameCollision)
{
  using ignition::math::Pose3d;

  this->config.RegisterCustomModelParser(customTomlParser);

  // ---------------- Name collision in //world/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.8">
    <world name="default">
      <include>
        <uri>double_pendulum.toml</uri>
        <name>test_name</name>
      </include>
      <model name="test_name">
        <static>true</static>
      </model>
    </world>
  </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::DUPLICATE_NAME, errors[0].Code());
  }
  // ---------------- Name collision in //model/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.8">
    <model name="parent_model">
      <include>
        <uri>double_pendulum.toml</uri>
        <name>test_name</name>
      </include>
      <model name="test_name">
        <static>true</static>
      </model>
    </model>
  </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::DUPLICATE_NAME, errors[0].Code());
  }
}
