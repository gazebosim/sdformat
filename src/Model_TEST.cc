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
#include <gz/math/Pose3.hh>
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "test_config.hh"

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

  EXPECT_EQ(gz::math::Pose3d::Zero, model.RawPose());
  EXPECT_TRUE(model.PoseRelativeTo().empty());
  {
    auto semanticPose = model.SemanticPose();
    EXPECT_EQ(model.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  model.SetRawPose({1, 2, 3, 0, 0, GZ_PI});
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, GZ_PI), model.RawPose());

  model.SetPoseRelativeTo("world");
  EXPECT_EQ("world", model.PoseRelativeTo());
  {
    auto semanticPose = model.SemanticPose();
    EXPECT_EQ(model.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("world", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
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

  // model doesn't have graphs, so no names should exist in graphs
  EXPECT_FALSE(model.NameExistsInFrameAttachedToGraph(""));
  EXPECT_FALSE(model.NameExistsInFrameAttachedToGraph("link"));
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
  // model doesn't have graphs, so no names should exist in graphs
  EXPECT_FALSE(model.NameExistsInFrameAttachedToGraph(""));
  EXPECT_FALSE(model.NameExistsInFrameAttachedToGraph("link1"));

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
TEST(DOMModel, AddModifyFrame)
{
  sdf::Model model;
  EXPECT_EQ(0u, model.FrameCount());

  sdf::Frame frame;
  frame.SetName("frame1");
  EXPECT_TRUE(model.AddFrame(frame));
  EXPECT_EQ(1u, model.FrameCount());
  EXPECT_FALSE(model.AddFrame(frame));
  EXPECT_EQ(1u, model.FrameCount());

  model.ClearFrames();
  EXPECT_EQ(0u, model.FrameCount());

  EXPECT_TRUE(model.AddFrame(frame));
  EXPECT_EQ(1u, model.FrameCount());

  const sdf::Frame *frameFromModel = model.FrameByIndex(0);
  ASSERT_NE(nullptr, frameFromModel);
  EXPECT_EQ(frameFromModel->Name(), frame.Name());

  sdf::Frame *mutableFrame = model.FrameByIndex(0);
  mutableFrame->SetName("newName1");
  EXPECT_EQ(mutableFrame->Name(), model.FrameByIndex(0)->Name());

  sdf::Frame *mutableFrameByName = model.FrameByName("frame1");
  EXPECT_EQ(nullptr, mutableFrameByName);
  mutableFrameByName = model.FrameByName("newName1");
  ASSERT_NE(nullptr, mutableFrameByName);
  EXPECT_EQ("newName1", model.FrameByName("newName1")->Name());
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
  model.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  sdf::Frame frame1, frame2;
  frame1.SetName("my-frame1");
  frame1.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  model.AddFrame(frame1);
  frame2.SetName("my-frame2");
  frame2.SetAttachedTo("my-frame1");
  frame2.SetRawPose(gz::math::Pose3d(-1, 20, 34, 0.1, 0.2, 0.3));
  model.AddFrame(frame2);
  EXPECT_EQ(2u, model.FrameCount());

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

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");
  model.AddPlugin(plugin);

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

  ASSERT_EQ(1u, model2.Plugins().size());
  EXPECT_EQ("name1", model2.Plugins()[0].Name());
  EXPECT_EQ("filename1", model2.Plugins()[0].Filename());

  ASSERT_EQ(2u, model2.FrameCount());
  const sdf::Frame *model2Frame1 = model2.FrameByIndex(0);
  EXPECT_EQ(frame1.Name(), model2Frame1->Name());
  EXPECT_EQ(frame1.AttachedTo(), model2Frame1->AttachedTo());
  EXPECT_EQ(frame1.RawPose(), model2Frame1->RawPose());

  const sdf::Frame *model2Frame2 = model2.FrameByIndex(1);
  EXPECT_EQ(frame2.Name(), model2Frame2->Name());
  EXPECT_EQ(frame2.AttachedTo(), model2Frame2->AttachedTo());
  EXPECT_EQ(frame2.RawPose(), model2Frame2->RawPose());
}

/////////////////////////////////////////////////
TEST(DOMModel, Uri)
{
  sdf::Model model;
  std::string name = "my-model";
  gz::math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  std::string uri =
    "https://fuel.gazebosim.org/1.0/openrobotics/models/my-model";

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

    sdf::ElementPtr uriElem = elem->FindElement("uri");
    ASSERT_NE(nullptr, uriElem);
    EXPECT_EQ(uri, uriElem->Get<std::string>());

    sdf::ElementPtr nameElem = elem->FindElement("name");
    ASSERT_NE(nullptr, nameElem);
    EXPECT_EQ(name, nameElem->Get<std::string>());

    sdf::ElementPtr poseElem = elem->FindElement("pose");
    ASSERT_NE(nullptr, poseElem);
    EXPECT_EQ(pose, poseElem->Get<gz::math::Pose3d>());
    EXPECT_EQ("other", poseElem->GetAttribute("relative_to")->GetAsString());

    EXPECT_EQ("link0",
        elem->FindElement("placement_frame")->Get<std::string>());

    sdf::ElementPtr staticElem = elem->FindElement("static");
    ASSERT_NE(nullptr, staticElem);
    EXPECT_EQ(true, staticElem->Get<bool>());
  }

  // ToElement NOT using the URI, which should result in a <model>
  {
    sdf::OutputConfig config = sdf::OutputConfig::GlobalConfig();
    config.SetToElementUseIncludeTag(false);
    sdf::ElementPtr elem = model.ToElement(config);
    elem->PrintValues("  ");

    // Should be a <model>
    EXPECT_EQ("model", elem->GetName());

    // URI should not exist
    sdf::ElementPtr uriElem = elem->FindElement("uri");
    ASSERT_EQ(nullptr, uriElem);

    sdf::ParamPtr nameAttr = elem->GetAttribute("name");
    ASSERT_NE(nullptr, nameAttr);
    EXPECT_EQ(name, nameAttr->GetAsString());

    sdf::ParamPtr placementFrameAttr = elem->GetAttribute("placement_frame");
    ASSERT_NE(nullptr, placementFrameAttr);
    EXPECT_EQ("link0", placementFrameAttr->GetAsString());

    sdf::ElementPtr poseElem = elem->FindElement("pose");
    ASSERT_NE(nullptr, poseElem);
    EXPECT_EQ(pose, poseElem->Get<gz::math::Pose3d>());
    EXPECT_EQ("other", poseElem->GetAttribute("relative_to")->GetAsString());

    sdf::ElementPtr staticElem = elem->FindElement("static");
    ASSERT_NE(nullptr, staticElem);
    EXPECT_EQ(true, staticElem->Get<bool>());
  }
}

/////////////////////////////////////////////////
TEST(DOMModel, ToElementNestedHasUri)
{
  sdf::Model model;
  model.SetName("parent");
  EXPECT_EQ(0u, model.ModelCount());

  sdf::Model nestedModel;
  nestedModel.SetName("child");
  nestedModel.SetUri("child-uri");
  EXPECT_TRUE(model.AddModel(nestedModel));
  EXPECT_EQ(1u, model.ModelCount());

  sdf::Model nestedModel2;
  nestedModel2.SetName("child2");
  EXPECT_TRUE(model.AddModel(nestedModel2));
  EXPECT_EQ(2u, model.ModelCount());

  sdf::ElementPtr elem = model.ToElement();

  // The parent model does not have a URI, so the element name should be
  // "model".
  ASSERT_NE(nullptr, elem);
  EXPECT_EQ("model", elem->GetName());

  // Get the child <include> element, which should exist because the nested
  // model has a URI.
  sdf::ElementPtr includeElem = elem->FindElement("include");
  ASSERT_NE(nullptr, includeElem);
  ASSERT_NE(nullptr, includeElem->FindElement("uri"));
  EXPECT_EQ("child-uri", includeElem->FindElement("uri")->Get<std::string>());

  // Get the child <model> element, which should exist because one nested
  // model does not have a URI.
  sdf::ElementPtr modelElem = elem->FindElement("model");
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ("child2", modelElem->GetAttribute("name")->GetAsString());
}

/////////////////////////////////////////////////
TEST(DOMModel, MutableByIndex)
{
  sdf::Model model;
  model.SetName("model1");
  EXPECT_EQ(0u, model.ModelCount());

  sdf::Link link;
  link.SetName("link1");
  EXPECT_TRUE(model.AddLink(link));

  sdf::Joint joint;
  joint.SetName("joint1");
  EXPECT_TRUE(model.AddJoint(joint));

  sdf::Model nestedModel;
  nestedModel.SetName("child1");
  EXPECT_TRUE(model.AddModel(nestedModel));

  // Modify the link
  sdf::Link *l = model.LinkByIndex(0);
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("link1", l->Name());
  l->SetName("link2");
  EXPECT_EQ("link2", model.LinkByIndex(0)->Name());

  // Modify the joint
  sdf::Joint *j = model.JointByIndex(0);
  ASSERT_NE(nullptr, j);
  EXPECT_EQ("joint1", j->Name());
  j->SetName("joint2");
  EXPECT_EQ("joint2", model.JointByIndex(0)->Name());

  // Modify the nested model
  sdf::Model *m = model.ModelByIndex(0);
  ASSERT_NE(nullptr, m);
  EXPECT_EQ("child1", m->Name());
  m->SetName("child2");
  EXPECT_EQ("child2", model.ModelByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, MutableByName)
{
  sdf::Model model;
  model.SetName("model1");
  EXPECT_EQ(0u, model.ModelCount());

  sdf::Link link;
  link.SetName("link1");
  EXPECT_TRUE(model.AddLink(link));

  sdf::Joint joint;
  joint.SetName("joint1");
  EXPECT_TRUE(model.AddJoint(joint));

  sdf::Model nestedModel;
  nestedModel.SetName("child1");
  EXPECT_TRUE(model.AddModel(nestedModel));

  sdf::Frame frame;
  frame.SetName("frame1");
  EXPECT_TRUE(model.AddFrame(frame));

  // Modify the link
  sdf::Link *l = model.LinkByName("link1");
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("link1", l->Name());
  l->SetName("link2");
  EXPECT_FALSE(model.LinkNameExists("link1"));
  EXPECT_TRUE(model.LinkNameExists("link2"));

  // Modify the joint
  sdf::Joint *j = model.JointByName("joint1");
  ASSERT_NE(nullptr, j);
  EXPECT_EQ("joint1", j->Name());
  j->SetName("joint2");
  EXPECT_FALSE(model.JointNameExists("joint1"));
  EXPECT_TRUE(model.JointNameExists("joint2"));

  // Modify the nested model
  sdf::Model *m = model.ModelByName("child1");
  ASSERT_NE(nullptr, m);
  EXPECT_EQ("child1", m->Name());
  m->SetName("child2");
  EXPECT_FALSE(model.ModelNameExists("child1"));
  EXPECT_TRUE(model.ModelNameExists("child2"));

  // Modify the frame
  sdf::Frame *f = model.FrameByName("frame1");
  ASSERT_NE(nullptr, f);
  EXPECT_EQ("frame1", f->Name());
  f->SetName("frame2");
  EXPECT_FALSE(model.FrameNameExists("frame1"));
  EXPECT_TRUE(model.FrameNameExists("frame2"));
}

/////////////////////////////////////////////////
TEST(DOMModel, Plugins)
{
  sdf::Model model;
  EXPECT_TRUE(model.Plugins().empty());

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");

  model.AddPlugin(plugin);
  ASSERT_EQ(1u, model.Plugins().size());

  plugin.SetName("name2");
  model.AddPlugin(plugin);
  ASSERT_EQ(2u, model.Plugins().size());

  EXPECT_EQ("name1", model.Plugins()[0].Name());
  EXPECT_EQ("name2", model.Plugins()[1].Name());

  model.ClearPlugins();
  EXPECT_TRUE(model.Plugins().empty());
}
