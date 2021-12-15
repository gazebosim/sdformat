/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/math/Pose3.hh>
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"

/////////////////////////////////////////////////
/// Test default construction of sdf::Model.
TEST(DOMModel, Construction)
{
  sdf::Model model;
  EXPECT_EQ(nullptr, model.Element());
  EXPECT_TRUE(model.Name().empty());

  model.SetName("test_model");
  EXPECT_EQ("test_model", model.Name());

  EXPECT_FALSE(model.Static());
  model.SetStatic(true);
  EXPECT_TRUE(model.Static());

  EXPECT_FALSE(model.SelfCollide());
  model.SetSelfCollide(true);
  EXPECT_TRUE(model.SelfCollide());

  EXPECT_TRUE(model.AllowAutoDisable());
  model.SetAllowAutoDisable(false);
  EXPECT_FALSE(model.AllowAutoDisable());

  EXPECT_FALSE(model.EnableWind());
  model.SetEnableWind(true);
  EXPECT_TRUE(model.EnableWind());

  EXPECT_EQ(0u, model.ModelCount());
  EXPECT_EQ(nullptr, model.ModelByIndex(0));
  EXPECT_EQ(nullptr, model.ModelByIndex(1));
  EXPECT_EQ(nullptr, model.ModelByName(""));
  EXPECT_EQ(nullptr, model.ModelByName("default"));
  EXPECT_EQ(nullptr, model.ModelByName("a::b"));
  EXPECT_EQ(nullptr, model.ModelByName("a::b::c"));
  EXPECT_EQ(nullptr, model.ModelByName("::::"));
  EXPECT_FALSE(model.ModelNameExists(""));
  EXPECT_FALSE(model.ModelNameExists("default"));
  EXPECT_FALSE(model.ModelNameExists("a::b"));
  EXPECT_FALSE(model.ModelNameExists("a::b::c"));
  EXPECT_FALSE(model.ModelNameExists("::::"));

  EXPECT_EQ(0u, model.LinkCount());
  EXPECT_EQ(nullptr, model.LinkByIndex(0));
  EXPECT_EQ(nullptr, model.LinkByIndex(1));
  EXPECT_EQ(nullptr, model.LinkByName(""));
  EXPECT_EQ(nullptr, model.LinkByName("default"));
  EXPECT_EQ(nullptr, model.LinkByName("a::b"));
  EXPECT_EQ(nullptr, model.LinkByName("a::b::c"));
  EXPECT_EQ(nullptr, model.LinkByName("::::"));
  EXPECT_FALSE(model.LinkNameExists(""));
  EXPECT_FALSE(model.LinkNameExists("default"));
  EXPECT_FALSE(model.LinkNameExists("a::b"));
  EXPECT_FALSE(model.LinkNameExists("a::b::c"));
  EXPECT_FALSE(model.LinkNameExists("::::"));

  EXPECT_EQ(0u, model.JointCount());
  EXPECT_EQ(nullptr, model.JointByIndex(0));
  EXPECT_EQ(nullptr, model.JointByIndex(1));
  EXPECT_EQ(nullptr, model.JointByName(""));
  EXPECT_EQ(nullptr, model.JointByName("default"));
  EXPECT_EQ(nullptr, model.JointByName("a::b"));
  EXPECT_EQ(nullptr, model.JointByName("a::b::c"));
  EXPECT_EQ(nullptr, model.JointByName("::::"));
  EXPECT_FALSE(model.JointNameExists(""));
  EXPECT_FALSE(model.JointNameExists("default"));
  EXPECT_FALSE(model.JointNameExists("a::b"));
  EXPECT_FALSE(model.JointNameExists("a::b::c"));
  EXPECT_FALSE(model.JointNameExists("::::"));

  EXPECT_EQ(0u, model.FrameCount());
  EXPECT_EQ(nullptr, model.FrameByIndex(0));
  EXPECT_EQ(nullptr, model.FrameByIndex(1));
  EXPECT_EQ(nullptr, model.FrameByName(""));
  EXPECT_EQ(nullptr, model.FrameByName("default"));
  EXPECT_EQ(nullptr, model.FrameByName("a::b"));
  EXPECT_EQ(nullptr, model.FrameByName("a::b::c"));
  EXPECT_EQ(nullptr, model.FrameByName("::::"));
  EXPECT_FALSE(model.FrameNameExists(""));
  EXPECT_FALSE(model.FrameNameExists("default"));
  EXPECT_FALSE(model.FrameNameExists("a::b"));
  EXPECT_FALSE(model.FrameNameExists("a::b::c"));
  EXPECT_FALSE(model.FrameNameExists("::::"));

  EXPECT_TRUE(model.CanonicalLinkName().empty());
  EXPECT_EQ(nullptr, model.CanonicalLink());
  model.SetCanonicalLinkName("link");
  EXPECT_EQ("link", model.CanonicalLinkName());
  EXPECT_EQ(nullptr, model.CanonicalLink());

  EXPECT_TRUE(model.PlacementFrameName().empty());
  model.SetPlacementFrameName("test_frame");
  EXPECT_EQ("test_frame", model.PlacementFrameName());

  EXPECT_EQ(ignition::math::Pose3d::Zero, model.RawPose());
  EXPECT_TRUE(model.PoseRelativeTo().empty());
  {
    auto semanticPose = model.SemanticPose();
    EXPECT_EQ(model.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  model.SetRawPose({1, 2, 3, 0, 0, IGN_PI});
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, IGN_PI), model.RawPose());

  model.SetPoseRelativeTo("world");
  EXPECT_EQ("world", model.PoseRelativeTo());
  {
    auto semanticPose = model.SemanticPose();
    EXPECT_EQ(model.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("world", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  auto errors = model.ValidateGraphs();
  EXPECT_EQ(2u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "FrameAttachedToGraph error: scope does not point to a valid graph"))
      << errors[0];
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR);
  EXPECT_NE(std::string::npos,
    errors[1].Message().find(
      "PoseRelativeToGraph error: scope does not point to a valid graph"))
      << errors[1];
}

/////////////////////////////////////////////////
TEST(DOMModel, CopyConstructor)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2(model);
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, CopyAssignmentOperator)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2;
  model2 = model;
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, MoveConstructor)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2(std::move(model));
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, MoveAssignmentOperator)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2;
  model2 = std::move(model);
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, CopyAssignmentAfterMove)
{
  sdf::Model model1;
  model1.SetName("model1");

  sdf::Model model2;
  model2.SetName("model2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Model tmp = std::move(model1);
  model1 = model2;
  model2 = tmp;

  EXPECT_EQ("model2", model1.Name());
  EXPECT_EQ("model1", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, AddLink)
{
  sdf::Model model;
  EXPECT_EQ(0u, model.LinkCount());

  sdf::Link link;
  link.SetName("link1");
  EXPECT_TRUE(model.AddLink(link));
  EXPECT_EQ(1u, model.LinkCount());
  EXPECT_FALSE(model.AddLink(link));
  EXPECT_EQ(1u, model.LinkCount());

  model.ClearLinks();
  EXPECT_EQ(0u, model.LinkCount());

  EXPECT_TRUE(model.AddLink(link));
  EXPECT_EQ(1u, model.LinkCount());
  const sdf::Link *linkFromModel = model.LinkByIndex(0);
  ASSERT_NE(nullptr, linkFromModel);
  EXPECT_EQ(linkFromModel->Name(), link.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, AddJoint)
{
  sdf::Model model;
  EXPECT_EQ(0u, model.JointCount());

  sdf::Joint joint;
  joint.SetName("joint1");
  EXPECT_TRUE(model.AddJoint(joint));
  EXPECT_EQ(1u, model.JointCount());
  EXPECT_FALSE(model.AddJoint(joint));
  EXPECT_EQ(1u, model.JointCount());

  model.ClearJoints();
  EXPECT_EQ(0u, model.JointCount());

  EXPECT_TRUE(model.AddJoint(joint));
  EXPECT_EQ(1u, model.JointCount());
  const sdf::Joint *jointFromModel = model.JointByIndex(0);
  ASSERT_NE(nullptr, jointFromModel);
  EXPECT_EQ(jointFromModel->Name(), joint.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, AddModel)
{
  sdf::Model model;
  EXPECT_EQ(0u, model.ModelCount());

  sdf::Model nestedModel;
  nestedModel.SetName("model1");
  EXPECT_TRUE(model.AddModel(nestedModel));
  EXPECT_EQ(1u, model.ModelCount());
  EXPECT_FALSE(model.AddModel(nestedModel));
  EXPECT_EQ(1u, model.ModelCount());

  model.ClearModels();
  EXPECT_EQ(0u, model.ModelCount());

  EXPECT_TRUE(model.AddModel(nestedModel));
  EXPECT_EQ(1u, model.ModelCount());
  const sdf::Model *modelFromModel = model.ModelByIndex(0);
  ASSERT_NE(nullptr, modelFromModel);
  EXPECT_EQ(modelFromModel->Name(), nestedModel.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, ToElement)
{
  sdf::Model model;

  model.SetName("my-model");
  model.SetStatic(true);
  model.SetSelfCollide(true);
  model.SetAllowAutoDisable(true);
  model.SetEnableWind(true);
  model.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 1; ++i)
    {
      sdf::Link link;
      link.SetName("link" + std::to_string(i));
      EXPECT_TRUE(model.AddLink(link));
      EXPECT_FALSE(model.AddLink(link));
    }
    if (j == 0)
      model.ClearLinks();
  }
  model.SetCanonicalLinkName("link1");
  model.SetPlacementFrameName("link0");

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 2; ++i)
    {
      sdf::Joint joint;
      joint.SetName("joint" + std::to_string(i));
      EXPECT_TRUE(model.AddJoint(joint));
      EXPECT_FALSE(model.AddJoint(joint));
    }
    if (j == 0)
      model.ClearJoints();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      sdf::Model nestedModel;
      nestedModel.SetName("model" + std::to_string(i));
      EXPECT_TRUE(model.AddModel(nestedModel));
      EXPECT_FALSE(model.AddModel(nestedModel));
    }
    if (j == 0)
      model.ClearModels();
  }

  sdf::ElementPtr elem = model.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Model model2;
  model2.Load(elem);

  EXPECT_EQ(model.Name(), model2.Name());
  EXPECT_EQ(model.Static(), model2.Static());
  EXPECT_EQ(model.SelfCollide(), model2.SelfCollide());
  EXPECT_EQ(model.AllowAutoDisable(), model2.AllowAutoDisable());
  EXPECT_EQ(model.EnableWind(), model2.EnableWind());
  EXPECT_EQ(model.RawPose(), model2.RawPose());
  EXPECT_EQ(model.CanonicalLinkName(), model2.CanonicalLinkName());
  EXPECT_EQ(model.PlacementFrameName(), model2.PlacementFrameName());

  EXPECT_EQ(model.LinkCount(), model2.LinkCount());
  for (uint64_t i = 0; i < model2.LinkCount(); ++i)
    EXPECT_NE(nullptr, model2.LinkByIndex(i));

  EXPECT_EQ(model.JointCount(), model2.JointCount());
  for (uint64_t i = 0; i < model2.JointCount(); ++i)
    EXPECT_NE(nullptr, model2.JointByIndex(i));

  EXPECT_EQ(model.ModelCount(), model2.ModelCount());
  for (uint64_t i = 0; i < model2.ModelCount(); ++i)
    EXPECT_NE(nullptr, model2.ModelByIndex(i));
}

/////////////////////////////////////////////////
TEST(DOMModel, Uri)
{
  sdf::Model model;
  std::string name = "my-model";
  ignition::math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  std::string uri =
    "https://fuel.ignitionrobotics.org/1.0/openrobotics/models/my-model";

  model.SetName(name);
  model.SetRawPose(pose);
  model.SetStatic(true);
  model.SetPlacementFrameName("link0");
  model.SetPoseRelativeTo("other");
  model.SetUri(uri);
  EXPECT_EQ(uri, model.Uri());

  // ToElement using the URI, which should result in an <include>
  {
    sdf::ElementPtr elem = model.ToElement();
    EXPECT_EQ("include", elem->GetName());

    sdf::ElementPtr uriElem = elem->GetElement("uri");
    ASSERT_NE(nullptr, uriElem);
    EXPECT_EQ(uri, uriElem->Get<std::string>());

    sdf::ElementPtr nameElem = elem->GetElement("name");
    ASSERT_NE(nullptr, nameElem);
    EXPECT_EQ(name, nameElem->Get<std::string>());

    sdf::ElementPtr poseElem = elem->GetElement("pose");
    ASSERT_NE(nullptr, poseElem);
    EXPECT_EQ(pose, poseElem->Get<ignition::math::Pose3d>());
    EXPECT_EQ("other", poseElem->GetAttribute("relative_to")->GetAsString());

    EXPECT_EQ("link0", elem->GetElement("placement_frame")->Get<std::string>());

    sdf::ElementPtr staticElem = elem->GetElement("static");
    ASSERT_NE(nullptr, staticElem);
    EXPECT_EQ(true, staticElem->Get<bool>());
  }

  // ToElement NOT using the URI, which should result in a <model>
  {
    sdf::ElementPtr elem = model.ToElement(false);
    elem->PrintValues("  ");

    // Should be a <model>
    EXPECT_EQ("model", elem->GetName());

    // URI should not exist
    sdf::ElementPtr uriElem = elem->GetElement("uri");
    ASSERT_EQ(nullptr, uriElem);

    sdf::ParamPtr nameAttr = elem->GetAttribute("name");
    ASSERT_NE(nullptr, nameAttr);
    EXPECT_EQ(name, nameAttr->GetAsString());

    sdf::ParamPtr placementFrameAttr = elem->GetAttribute("placement_frame");
    ASSERT_NE(nullptr, placementFrameAttr);
    EXPECT_EQ("link0", placementFrameAttr->GetAsString());

    sdf::ElementPtr poseElem = elem->GetElement("pose");
    ASSERT_NE(nullptr, poseElem);
    EXPECT_EQ(pose, poseElem->Get<ignition::math::Pose3d>());
    EXPECT_EQ("other", poseElem->GetAttribute("relative_to")->GetAsString());

    sdf::ElementPtr staticElem = elem->GetElement("static");
    ASSERT_NE(nullptr, staticElem);
    EXPECT_EQ(true, staticElem->Get<bool>());
  }
}
