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
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/PrintConfig.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "test_config.hh"
#include "toml_parser.hh"

sdf::InterfaceModelPtr parseModel(toml::Value &_doc,
    const std::string &_modelName)
{
  const auto canonicalLink = _doc["canonical_link"].ParamGet<std::string>();

  // Pose of model (M) in parent (P) frame
  const auto X_PM = _doc["pose"].ParamGet<gz::math::Pose3d>();

  bool isStatic = _doc["static"].ParamGet<bool>(false);

  auto model = std::make_shared<sdf::InterfaceModel>(
      _modelName, nullptr, isStatic, canonicalLink, X_PM);

  for (auto &[name, link] : _doc["links"].Map())
  {
    const auto pose = link["pose"].ParamGet<gz::math::Pose3d>();
    model->AddLink({name, pose});
  }

  for (auto &[name, frame] : _doc["frames"].Map())
  {
    const auto attachedTo =
        frame["attached_to"].ParamGet<std::string>("__model__");
    const auto pose = frame["pose"].ParamGet<gz::math::Pose3d>();
    model->AddFrame({name, attachedTo, pose});
  }

  for (auto &[name, joint] : _doc["joints"].Map())
  {
    const auto pose = joint["pose"].ParamGet<gz::math::Pose3d>();
    const auto child = joint["child"].ParamGet<std::string>();
    model->AddJoint({name, child, pose});
  }
  for (auto &[name, nestedModel] : _doc["models"].Map())
  {
    model->AddNestedModel(parseModel(nestedModel, name));
  }

  return model;
}

class CustomTomlParser
{
  /// \brief Constructor
  /// \param[in] _supportsMergeInclude Whether the parser supports merge include
  /// \param[in] _overridePoseInParser Whether the parser should apply pose
  /// overrides from //include/pose
  public: CustomTomlParser(bool _supportsMergeInclude = true,
                           bool _overridePoseInParser = true)
      : supportsMergeInclude(_supportsMergeInclude),
        overridePoseInParser(_overridePoseInParser)
  {
  }

  public: sdf::InterfaceModelPtr operator()(const sdf::NestedInclude &_include,
                                            sdf::Errors &_errors)
  {
    toml::Document doc = toml::parseToml(_include.ResolvedFileName(), _errors);
    if (_errors.empty())
    {
      const std::string modelName =
        _include.LocalModelName().value_or(doc["name"].ParamGet<std::string>());

      if (_include.IsStatic().has_value())
      {
        // if //include/static is set, override the value in the included model
        sdf::Param param("static", "bool", "false", false);
        param.Set(*_include.IsStatic());
        doc["static"] = {param};
      }
      if (this->overridePoseInParser && _include.IncludeRawPose().has_value())
      {
        // if //include/static is set, override the value in the included model
        sdf::Param poseParam("pose", "pose", "", false);
        poseParam.Set(*_include.IncludeRawPose());
        doc["pose"] = {poseParam};
      }

      auto model = parseModel(doc, modelName);
      model->SetParserSupportsMergeInclude(this->supportsMergeInclude);
      return model;
    }
    return nullptr;
  }

  public: bool supportsMergeInclude;
  public: bool overridePoseInParser{true};
};

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
    this->modelDir = sdf::testing::TestFile("integration", "model");

    this->config.SetFindCallback(
        [=](const std::string &_file)
        {
          return sdf::filesystem::append(modelDir, _file);
        });
  }
  public: void CheckFrameSemantics(const sdf::World *world);

  public: std::string modelDir;
  public: sdf::ParserConfig config;
  public: CustomTomlParser customTomlParser;
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
    if (!endsWith(_include.Uri(), ".nonce_1"))
      return nullptr;

    const std::string fileName = "file_wont_be_parsed.nonce_1";
    EXPECT_EQ(fileName, _include.Uri());
    EXPECT_EQ(sdf::filesystem::append(this->modelDir, fileName),
              _include.ResolvedFileName());
    EXPECT_EQ("box", *_include.LocalModelName());
    EXPECT_TRUE(_include.IsStatic().has_value());
    EXPECT_TRUE(_include.IsStatic().value());

    EXPECT_TRUE(_include.IncludeRawPose().has_value());
    EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0),
              _include.IncludeRawPose().value());

    EXPECT_TRUE(_include.IncludePoseRelativeTo().has_value());
    EXPECT_EQ("F1", _include.IncludePoseRelativeTo().value());
    EXPECT_TRUE(_include.IncludeElement()->HasElement("extra"));

    auto extra = _include.IncludeElement()->GetElement("extra");
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
    if (!endsWith(_include.Uri(), ".nonce_2"))
      return nullptr;
    const std::string fileName = "file_wont_be_parsed.nonce_2";
    EXPECT_EQ(fileName, _include.Uri());
    EXPECT_EQ(sdf::filesystem::append(modelDir, fileName),
              _include.ResolvedFileName());
    EXPECT_FALSE(_include.LocalModelName().has_value());
    EXPECT_FALSE(_include.IsStatic());

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
            "base_link", gz::math::Pose3d());
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
  EXPECT_TRUE(errors.empty()) << errors;
  // Check that custom parsers are called in reverse order of registration.
  ASSERT_EQ(3u, customParserCallOrder.size());
  // Parser 0 will not be visited because parser 1 will be successful.
  const std::vector<int> customParserCallOrderExpected {3, 2, 1};
  EXPECT_EQ(customParserCallOrderExpected, customParserCallOrder);
}

/////////////////////////////////////////////////
void TomlParserTest(const sdf::InterfaceModelConstPtr &_interfaceModel)
{
  using gz::math::Pose3d;
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
      {"frame_1", {"__model__", Pose3d(0, 1, 0.0, 0, 0, 0)}},
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
  using gz::math::Pose3d;
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "world_include_with_interface_api.sdf");

  this->config.RegisterCustomModelParser(this->customTomlParser);
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
  using gz::math::Pose3d;
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "model_include_with_interface_api.sdf");

  this->config.RegisterCustomModelParser(this->customTomlParser);
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
TEST_F(InterfaceAPI, DeeplyNestedModel)
{
  const std::string testSdf = R"(
  <sdf version="1.10">
    <model name="grand_parent_model">
      <include>
        <uri>model://model_include_with_interface_api.sdf</uri>
      </include>
    </model>
  </sdf>)";

  this->config.AddURIPath("model://", sdf::testing::TestFile("sdf"));
  this->config.RegisterCustomModelParser(this->customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const sdf::Model *grandParentModel = root.Model();
  ASSERT_NE(nullptr, grandParentModel);
  EXPECT_EQ("grand_parent_model", grandParentModel->Name());
  const sdf::Model *parentModel = grandParentModel->ModelByIndex(0);
  ASSERT_NE(nullptr, parentModel);
  EXPECT_EQ("parent_model", parentModel->Name());
  auto interfaceModel = parentModel->InterfaceModelByIndex(0);
  ASSERT_NE(nullptr, interfaceModel);
  SCOPED_TRACE("DeeplyNestedModel");
  TomlParserTest(interfaceModel);
}

gz::math::Pose3d resolvePoseNoErrors(const sdf::SemanticPose &_semPose,
                                     const std::string &_relativeTo = "")
{
  gz::math::Pose3d pose;
  sdf::Errors resolveErrors = _semPose.Resolve(pose, _relativeTo);
  EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
  return pose;
}

std::string resolveAttachedToNoErrors(const sdf::Frame &_frame)
{
  std::string resolvedBody;
  sdf::Errors resolveErrors = _frame.ResolveAttachedToBody(resolvedBody);
  EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
  return resolvedBody;
}

void InterfaceAPI::CheckFrameSemantics(const sdf::World *world)
{
  using gz::math::Pose3d;

  {
    const sdf::Frame *frame = world->FrameByName("F0");
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
TEST_F(InterfaceAPI, FrameSemantics)
{
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "include_with_interface_api_frame_semantics.sdf");
  this->config.RegisterCustomModelParser(this->customTomlParser);
  {
    sdf::Root root;
    sdf::Errors errors = root.Load(testFile, config);
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::World *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    EXPECT_EQ(1u, world->InterfaceModelCount());

    SCOPED_TRACE("InterfaceAPI.FrameSemantics");
    this->CheckFrameSemantics(world);
  }
  {
    // Check without //include/pose override applied in parser.
    sdf::Root root;
    sdf::ParserConfig newConfig = this->config;
    CustomTomlParser parserWithoutPoseOverride(true, false);
    newConfig.RegisterCustomModelParser(parserWithoutPoseOverride);
    sdf::Errors errors = root.Load(testFile, newConfig);
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::World *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    EXPECT_EQ(1u, world->InterfaceModelCount());

    SCOPED_TRACE("InterfaceAPI.FrameSemantics_NoPoseOverrideInParser");
    this->CheckFrameSemantics(world);
  }
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, Reposturing)
{
  using gz::math::Pose3d;
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "include_with_interface_api_reposture.sdf");

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
        gz::math::Pose3d pose;
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
          gz::math::Pose3d pose;
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
    bool fileHasCorrectSuffix =
        endsWith(_include.ResolvedFileName(), ".nonce_1");
    EXPECT_TRUE(fileHasCorrectSuffix)
        << "File: " << _include.ResolvedFileName();
    if (!fileHasCorrectSuffix)
      return nullptr;

    const std::string absoluteModelName = sdf::JoinName(
        _include.AbsoluteParentName(), *_include.LocalModelName());

    auto model = std::make_shared<sdf::InterfaceModel>(
        *_include.LocalModelName(), makeRepostureFunc(absoluteModelName), false,
        "base_link", _include.IncludeRawPose().value_or(Pose3d{}));
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
  using gz::math::Pose3d;
  std::unordered_map<std::string, Pose3d> modelPosesAfterReposture;
  auto repostureTestParser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &)
  {
    std::string modelName = sdf::JoinName(_include.AbsoluteParentName(),
                                          *_include.LocalModelName());
    auto repostureFunc = [modelName = modelName, &modelPosesAfterReposture](
                             const sdf::InterfaceModelPoseGraph &_graph)
    {
      gz::math::Pose3d pose;
      sdf::Errors errors = _graph.ResolveNestedModelFramePoseInWorldFrame(pose);
      EXPECT_TRUE(errors.empty()) << errors;
      modelPosesAfterReposture[modelName] = pose;
    };

    auto model = std::make_shared<sdf::InterfaceModel>(
        *_include.LocalModelName(), repostureFunc, false, "base_link",
        _include.IncludeRawPose().value_or(Pose3d{}));
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
  using gz::math::Pose3d;

  this->config.RegisterCustomModelParser(this->customTomlParser);

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

/////////////////////////////////////////////////
class InterfaceAPIMergeInclude : public InterfaceAPI
{
};

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, MergeIncludeNotSupported)
{
  const std::string testSdf = R"(
  <sdf version="1.9">
    <model name="parent_model">
      <include merge="true">
        <uri>double_pendulum.toml</uri>
      </include>
    </model>
  </sdf>)";
  CustomTomlParser parserWithNoMergeInclude(false);
  this->config.RegisterCustomModelParser(parserWithNoMergeInclude);
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::MERGE_INCLUDE_UNSUPPORTED, errors[0].Code());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, FrameSemantics)
{
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "merge_include_with_interface_api_frame_semantics.sdf");
  this->config.RegisterCustomModelParser(this->customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  SCOPED_TRACE("InterfaceAPIMergeInclude.FrameSemantics");
  this->CheckFrameSemantics(world);
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, Reposturing)
{
  using gz::math::Pose3d;
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "merge_include_with_interface_api_reposture.sdf");

  std::unordered_map<std::string, Pose3d> posesAfterReposture;
  std::unordered_map<std::string, std::vector<std::string>> elementsToReposture;

  // Create a resposture callback function for a given absolute model name. The
  // name is used to store poses in `posesAfterReposture`.
  auto makeRepostureFunc = [&](const std::string &_absoluteName)
  {
    auto repostureFunc =
        [modelName = _absoluteName, &elementsToReposture, &posesAfterReposture](
            const sdf::InterfaceModelPoseGraph &_graph)
    {
      auto modelIt = elementsToReposture.find(modelName);
      ASSERT_TRUE(modelIt != elementsToReposture.end());

      for (const auto &elem : modelIt->second)
      {
        gz::math::Pose3d pose;
        sdf::Errors errors =
            _graph.ResolveNestedFramePose(pose, elem);
        EXPECT_TRUE(errors.empty()) << errors;
        posesAfterReposture[sdf::JoinName(modelName, elem)] = pose;
      }
    };
    return repostureFunc;
  };

  auto repostureTestParser = [&](const sdf::NestedInclude &_include,
                                 sdf::Errors &) -> sdf::InterfaceModelPtr
  {
    bool fileHasCorrectSuffix =
        endsWith(_include.ResolvedFileName(), ".nonce_1");
    EXPECT_TRUE(fileHasCorrectSuffix)
        << "File: " << _include.ResolvedFileName();
    if (!fileHasCorrectSuffix)
      return nullptr;

    // Use parent name because we know merge=true
    const std::string &absoluteModelName = _include.AbsoluteParentName();
    // The following is equivalent to
    // <model name="M0"> <!-- Merged into parent model
    //   <pose relative_to="F1">0 0 0  π/2 0 0</pose> <!-- From //include -->
    //   <!-- World pose of M0: (1 2 3   π/2 0 0)
    //   <link name="base_link"/> <!-- World pose: (1 2 3   π/2 0 0) -->
    //   <link name="top_link">
    //     <pose>0 0 1   0 0 0</pose> <!-- World pose: (1 1 3   π/2 0 0) -->
    //   </link>
    //   <joint name="j1" type="fixed">
    //     <pose>1 0 0   0 0 0</pose> <!-- World pose: (2 1 3   0 0 0) -->
    //     <parent>base_link</parent>
    //     <child>top_link</child>
    //   </joint>
    //   <frame name="frame1">
    //     <pose>0 1 0   0 0 0</pose> <!-- World pose: (1 2 4   π/2 0 0) -->
    //   </frame>
    //   <frame name="frame2" attached_to="frame1">
    //     <pose>0 0 1   0 0 0</pose> <!-- World pose: (1 1 4   π/2 0 0) -->
    //   </frame>
    //   <model name="nested_model">
    //     <pose>3 0 0   0 0 0</pose> <!-- World pose: (4 2 3   π/2 0 0) -->
    //     <link name="nested_link">
    //       <pose>0 0 0   π/2 0 0</pose> <!-- World pose: (4 2 3   π 0 0) -->
    //     </link>
    //   </model>
    // </model>
    auto model = std::make_shared<sdf::InterfaceModel>(
        *_include.LocalModelName(), makeRepostureFunc(absoluteModelName), false,
        "base_link", Pose3d{});
    model->AddLink({"base_link", {}});
    model->AddLink({"top_link", Pose3d(0, 0, 1, 0, 0, 0)});
    model->AddJoint({"j1", "top_link", Pose3d(1, 0, 0, 0, 0, 0)});
    model->AddFrame({"frame1", "__model__", Pose3d(0, 1, 0, 0, 0, 0)});
    model->AddFrame({"frame2", "frame1", Pose3d(0, 0, 1, 0, 0, 0)});
    elementsToReposture[absoluteModelName].emplace_back("base_link");
    elementsToReposture[absoluteModelName].emplace_back("top_link");
    elementsToReposture[absoluteModelName].emplace_back("j1");
    elementsToReposture[absoluteModelName].emplace_back("frame1");
    elementsToReposture[absoluteModelName].emplace_back("frame2");

    elementsToReposture[absoluteModelName].emplace_back("nested_model");
    const std::string absoluteNestedModelName =
        sdf::JoinName(absoluteModelName, "nested_model");
    auto nestedModel = std::make_shared<sdf::InterfaceModel>("nested_model",
        makeRepostureFunc(absoluteNestedModelName), false, "nested_link",
        Pose3d(3, 0, 0, 0, 0, 0));

    nestedModel->AddLink({"nested_link", Pose3d(0, 0, 0, GZ_PI_2, 0, 0)});
    elementsToReposture[absoluteNestedModelName].emplace_back("nested_link");

    model->AddNestedModel(nestedModel);
    model->SetParserSupportsMergeInclude(true);
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
  // There is one included model using a custom parser containing two models and
  // two links.
  ASSERT_EQ(7u, posesAfterReposture.size());
  EXPECT_TRUE(checkPose("parent_model::base_link", {1, 2, 3, GZ_PI_2, 0, 0}));
  EXPECT_TRUE(checkPose("parent_model::top_link", {1, 1, 3, GZ_PI_2, 0, 0}));
  EXPECT_TRUE(checkPose("parent_model::j1", {2, 1, 3, GZ_PI_2, 0, 0}));
  EXPECT_TRUE(checkPose("parent_model::frame1", {1, 2, 4, GZ_PI_2, 0, 0}));
  EXPECT_TRUE(checkPose("parent_model::frame2", {1, 1, 4, GZ_PI_2, 0, 0}));
  EXPECT_TRUE(
      checkPose("parent_model::nested_model", {4, 2, 3, GZ_PI_2, 0, 0}));
  EXPECT_TRUE(checkPose(
      "parent_model::nested_model::nested_link", {4, 2, 3, GZ_PI, 0, 0}));
}


/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, PlacementFrame)
{
  using gz::math::Pose3d;
  std::unordered_map<std::string, Pose3d> modelPosesAfterReposture;
  auto repostureTestParser =
      [&](const sdf::NestedInclude &_include, sdf::Errors &)
  {
    std::string modelName = sdf::JoinName(_include.AbsoluteParentName(),
                                          *_include.LocalModelName());
    auto repostureFunc = [modelName = modelName, &modelPosesAfterReposture](
                             const sdf::InterfaceModelPoseGraph &_graph)
    {
      gz::math::Pose3d pose;
      sdf::Errors errors = _graph.ResolveNestedModelFramePoseInWorldFrame(pose);
      EXPECT_TRUE(errors.empty()) << errors;
      modelPosesAfterReposture[modelName] = pose;
    };

    auto model = std::make_shared<sdf::InterfaceModel>(
        *_include.LocalModelName(), repostureFunc, false, "base_link",
        _include.IncludeRawPose().value_or(Pose3d{}));
    model->AddLink({"base_link", Pose3d(0, 1, 0, 0, 0, 0)});
    model->AddFrame({"frame_1", "__model__", Pose3d(0, 0, 1, 0, 0, 0)});
    model->SetParserSupportsMergeInclude(true);
    return model;
  };

  this->config.RegisterCustomModelParser(repostureTestParser);
  this->config.SetFindCallback(
      [](const auto &_fileName)
      {
        return _fileName;
      });

  // ---------------- Placement frame in //sdf/model/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.9">
    <model name="parent_model">
      <include merge="true">
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
    const auto *testFrame = parentModel->FrameByName("test_frame");
    ASSERT_NE(nullptr, testFrame);
    {
      // Since there is no InterfaceModel::SemanticPose, we resolve the pose of
      // test_frame relative to the test_model and take the inverse as the pose
      // of the test_model relative to the world.
      Pose3d pose;
      sdf::Errors resolveErrors = testFrame->SemanticPose().Resolve(
          pose, "_merged__test_model__model__");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, -1, 0, 0, 0), pose.Inverse());
    }
    {
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "frame_1");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), pose.Inverse());
    }
  }

  // ---------------- Placement frame in //world//model/include ----------------
  {
    const std::string testSdf = R"(
  <sdf version="1.9">
    <world name="default">
      <model name="parent_model">
        <include merge="true">
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
    const auto *testFrame = parentModel->FrameByName("test_frame");
    ASSERT_NE(nullptr, testFrame);
    {
      // Since there is no InterfaceModel::SemanticPose, we resolve the pose of
      // test_frame relative to the test_model and take the inverse as the pose
      // of the test_model relative to the world.
      Pose3d pose;
      sdf::Errors resolveErrors = testFrame->SemanticPose().Resolve(
          pose, "_merged__test_model__model__");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, -1, 0, 0, 0), pose.Inverse());
    }
    {
      Pose3d pose;
      sdf::Errors resolveErrors =
          testFrame->SemanticPose().Resolve(pose, "frame_1");
      EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
      EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), pose.Inverse());
    }
  }
}

/////////////////////////////////////////////////
// Tests PrintConfig
TEST_F(InterfaceAPI, TomlParserModelIncludePrintConfig)
{
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "model_include_with_interface_api.sdf");

  this->config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile, this->config);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  const sdf::ElementPtr includeElem = model->Element()->GetFirstElement();
  ASSERT_NE(nullptr, includeElem);

  const std::string expectedIncludeStr =
R"(<include>
  <uri>double_pendulum.toml</uri>
  <pose>1 0 0 0 0 0</pose>
  <extra>
    <info1>value1</info1>
    <info2>value2</info2>
  </extra>
</include>
)";

  sdf::PrintConfig printConfig;
  EXPECT_EQ(includeElem->ToString("", printConfig), expectedIncludeStr);
  printConfig.SetPreserveIncludes(true);
  EXPECT_EQ(includeElem->ToString("", printConfig), expectedIncludeStr);
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, JointModelChild)
{
  const std::string testSdf = R"(
  <sdf version="1.9">
    <model name="parent_model">
      <link name="L1"/>
      <include merge="true">
        <uri>joint_child_model_frame.toml</uri>
      </include>
      <frame name="frame_1" attached_to="joint_model_child"/>
    </model>
  </sdf>)";

  this->config.RegisterCustomModelParser(customTomlParser);
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  {
    auto frame = model->FrameByName("frame_1");
    std::string body;
    frame->ResolveAttachedToBody(body);
    EXPECT_EQ("base", body);
  }
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeInclude1a)
{
  auto checkParentNameParser =
      [this](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    EXPECT_EQ("parent_model::intermediate_model",
              _include.AbsoluteParentName());
    return this->customTomlParser(_include, _errors);
  };

  this->config.RegisterCustomModelParser(checkParentNameParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <model name="parent_model">
      <link name="link1"/>
      <include>
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </model>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto* parentModel = root.Model();
  ASSERT_NE(nullptr, parentModel);
  EXPECT_NE(nullptr, parentModel->ModelByName("intermediate_model"));
  using gz::math::Pose3d;
  EXPECT_EQ(
      Pose3d(0, 10, 10, 0, 0, 0),
      resolvePoseNoErrors(parentModel->SemanticPose(),
                          "parent_model::intermediate_model::double_pendulum")
          .Inverse());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeInclude1b)
{
  auto checkParentNameParser =
      [this](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    EXPECT_EQ("parent_model",
              _include.AbsoluteParentName());
    return this->customTomlParser(_include, _errors);
  };

  this->config.RegisterCustomModelParser(checkParentNameParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <model name="parent_model">
      <include merge="true">
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </model>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto* parentModel = root.Model();
  ASSERT_NE(nullptr, parentModel);
  EXPECT_EQ(nullptr, parentModel->ModelByName("intermediate_model"));
  using gz::math::Pose3d;
  EXPECT_EQ(Pose3d(0, 10, 10, 0, 0, 0),
            resolvePoseNoErrors(parentModel->SemanticPose(),
                                "parent_model::double_pendulum")
                .Inverse());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeInclude2)
{
  auto checkParentNameParser =
      [this](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    EXPECT_EQ("parent_model", _include.AbsoluteParentName());
    return this->customTomlParser(_include, _errors);
  };

  this->config.RegisterCustomModelParser(checkParentNameParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <model name="parent_model">
      <include merge="true">
        <uri>intermediate_model_with_interface_api_2.sdf</uri>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </model>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto* parentModel = root.Model();
  ASSERT_NE(nullptr, parentModel);
  using gz::math::Pose3d;
  EXPECT_EQ(
      Pose3d(1, 10, 10.5, 0, 0, 0),
      resolvePoseNoErrors(parentModel->SemanticPose(), "parent_model::base")
          .Inverse());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeIncludePlacementFrame)
{
  this->config.RegisterCustomModelParser(this->customTomlParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <model name="parent_model">
      <include merge="true">
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <placement_frame>double_pendulum::lower_link</placement_frame>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </model>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto* parentModel = root.Model();
  ASSERT_NE(nullptr, parentModel);
  using gz::math::Pose3d;
  EXPECT_EQ(Pose3d(0, 10, 0, 0, 0, 0),
            resolvePoseNoErrors(parentModel->SemanticPose(),
                                "parent_model::double_pendulum::lower_link")
                .Inverse());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeIncludeInWorldPlacementFrame)
{
  this->config.RegisterCustomModelParser(this->customTomlParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <world name="default">
      <frame name="world_frame"/>
      <include merge="true">
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <placement_frame>double_pendulum::lower_link</placement_frame>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </world>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  const auto *worldFrame = world->FrameByName("world_frame");
  ASSERT_NE(nullptr, worldFrame);
  using gz::math::Pose3d;
  EXPECT_EQ(Pose3d(0, 10, 0, 0, 0, 0),
            resolvePoseNoErrors(worldFrame->SemanticPose(),
                                "double_pendulum::lower_link")
                .Inverse());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeIncludeInWorld)
{
  auto checkParentNameParser =
      [this](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  { return this->customTomlParser(_include, _errors); };

  this->config.RegisterCustomModelParser(checkParentNameParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <world name="merge_world">
      <include merge="true">
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <pose>0 10 0   0 0 0</pose>
      </include>
      <frame name="world_frame"/>
    </world>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(nullptr, world->ModelByName("intermediate_model"));
  const auto *worldFrame = world->FrameByName("world_frame");
  ASSERT_NE(nullptr, worldFrame);
  using gz::math::Pose3d;
  EXPECT_EQ(Pose3d(0, 10, 10, 0, 0, 0),
            resolvePoseNoErrors(worldFrame->SemanticPose(), "double_pendulum")
                .Inverse());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeIncludeElementOrder)
{
  auto checkParentNameParser =
      [this](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    return this->customTomlParser(_include, _errors);
  };

  this->config.RegisterCustomModelParser(checkParentNameParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <model name="parent_model">
      <link name="L0"/>
      <frame name="F0"/>
      <joint name="J0" type="fixed">
        <parent>world</parent>
        <child>L0</child>
      </joint>
      <include merge="true">
        <uri>test_model_with_frames</uri>
      </include>
      <include>
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </model>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto* parentModel = root.Model();
  ASSERT_NE(nullptr, parentModel);
  EXPECT_NE(nullptr, parentModel->ModelByName("M2"));
  EXPECT_NE(nullptr, parentModel->ModelByName("intermediate_model"));
  ASSERT_GE(parentModel->LinkCount(), 2u);
  EXPECT_EQ("L0", parentModel->LinkByIndex(0)->Name());
  EXPECT_EQ("L1", parentModel->LinkByIndex(1)->Name());
  EXPECT_NE(nullptr, parentModel->CanonicalLinkAndRelativeName().first);
  EXPECT_EQ("L0", parentModel->CanonicalLinkAndRelativeName().second);
  ASSERT_GE(parentModel->FrameCount(), 1u);
  EXPECT_EQ("F0", parentModel->FrameByIndex(0)->Name());
  ASSERT_GE(parentModel->JointCount(), 1u);
  EXPECT_EQ("J0", parentModel->JointByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPIMergeInclude, DeeplyNestedMergeIncludeElementOrderInWorld)
{
  auto checkParentNameParser =
      [this](const sdf::NestedInclude &_include, sdf::Errors &_errors)
  {
    return this->customTomlParser(_include, _errors);
  };

  this->config.RegisterCustomModelParser(checkParentNameParser);

  const std::string testSdf = R"(
  <sdf version="1.10">
    <world name="default">
      <frame name="F0"/>
      <joint name="J0" type="fixed">
        <parent>world</parent>
        <child>M1</child>
      </joint>
      <include merge="true">
        <uri>model_for_world_merge_include.sdf</uri>
      </include>
      <include>
        <uri>intermediate_model_with_interface_api_1.sdf</uri>
        <pose>0 10 0   0 0 0</pose>
      </include>
    </world>
  </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
  const auto* world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_NE(nullptr, world->ModelByName("M1"));
  EXPECT_NE(nullptr, world->ModelByName("intermediate_model"));
  ASSERT_GE(world->FrameCount(), 1u);
  EXPECT_EQ("F0", world->FrameByIndex(0)->Name());
  ASSERT_GE(world->JointCount(), 1u);
  EXPECT_EQ("J0", world->JointByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST_F(InterfaceAPI, JointParentOrChildInNestedModel)
{
  this->config.RegisterCustomModelParser(customTomlParser);

  const std::string testSdf = R"(
  <sdf version="1.8">
    <model name="parent_model">
      <link name="L1"/>

      <joint name="J1" type="fixed">
        <parent>L1</parent>
        <child>double_pendulum::base</child>
      </joint>
      <include>
        <uri>double_pendulum.toml</uri>
        <name>double_pendulum</name>
      </include>

      <joint name="J2" type="fixed">
        <parent>double_pendulum::child_dp::base</parent>
        <child>L2</child>
      </joint>
      <link name="L2"/>

    </model>
  </sdf>)";
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testSdf, this->config);
  EXPECT_TRUE(errors.empty()) << errors;
}
