/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <map>
#include <string>

#include <gtest/gtest.h>

#include "sdf/Model.hh"
#include "sdf/Joint.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"

#include "test_config.hh"

const char SDF_TEST_FILE[] = "fixed_joint_reduction.urdf";
const char SDF_TEST_FILE_COLLISION[] = "fixed_joint_reduction_collision.urdf";
const char SDF_TEST_FILE_SIMPLE[] = "fixed_joint_reduction_simple.urdf";
const char SDF_TEST_FILE_VISUAL[] = "fixed_joint_reduction_visual.urdf";
const char SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION[] =
    "fixed_joint_reduction_collision_visual_extension.urdf";
const char SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION_SDF[] =
    "fixed_joint_reduction_collision_visual_extension.sdf";
const char SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION_EMPTY_ROOT[] =
    "fixed_joint_reduction_collision_visual_empty_root.urdf";
const char SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION_EMPTY_ROOT_SDF[] =
    "fixed_joint_reduction_collision_visual_empty_root.sdf";
const char SDF_TEST_FILE_JOINT_FRAME_EXTENSION[] =
    "fixed_joint_reduction_joint_frame_extension.urdf";
const char SDF_TEST_FILE_PLUGIN_FRAME_EXTENSION[] =
    "fixed_joint_reduction_plugin_frame_extension.urdf";

static std::string GetFullTestFilePath(const char *_input)
{
  return sdf::testing::TestFile("integration", _input);
}

const double gc_tolerance = 1e-6;

void FixedJointReductionEquivalence(const std::string &_file);
void FixedJointReductionCollisionVisualExtension(const std::string &_urdfFile,
                                                 const std::string &_sdfFile);
void FixedJointReductionCollisionVisualExtensionEmptyRoot(
  const std::string &_urdfFile, const std::string &_sdfFile);

/////////////////////////////////////////////////
TEST(SDFParser, FixedJointReductionEquivalenceTest)
{
  FixedJointReductionEquivalence(GetFullTestFilePath(SDF_TEST_FILE));
}

/////////////////////////////////////////////////
// This test uses a urdf that has a fixed joint whose parent
// has no visuals. This can cause a seg-fault with the wrong
// version of urdfdom (see #63).
TEST(SDFParser, FixedJointReductionVisualTest)
{
  FixedJointReductionEquivalence(GetFullTestFilePath(SDF_TEST_FILE_VISUAL));
}

/////////////////////////////////////////////////
// This test uses a urdf that has a fixed joint whose parent
// has no collisions. This is a parallel of the
// FixedJointReductionVisualTest
TEST(SDFParser, FixedJointReductionCollisionTest)
{
  FixedJointReductionEquivalence(GetFullTestFilePath(SDF_TEST_FILE_COLLISION));
}

// this updated test will not work for old parser

/////////////////////////////////////////////////
// This test uses a urdf that has two levels
// of fixed joint reduction.
// Compare parsing results with a pre-built verified sdf
// as well as with hardcoded expected values.
TEST(SDFParser, FixedJointReductionCollisionVisualExtensionTest)
{
  FixedJointReductionCollisionVisualExtension(
    GetFullTestFilePath(SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION),
    GetFullTestFilePath(SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION_SDF));
}

/////////////////////////////////////////////////
// This test uses a urdf that has a fixed joint whose parent link
// (base_link) has no collisions.
// Compare parsing results with a pre-built verified sdf
// as well as with hardcoded expected values.
TEST(SDFParser, FixedJointReductionCollisionVisualExtensionEmptyRootTest)
{
  FixedJointReductionCollisionVisualExtensionEmptyRoot(
      GetFullTestFilePath(SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION_EMPTY_ROOT),
      GetFullTestFilePath(
          SDF_TEST_FILE_COLLISION_VISUAL_EXTENSION_EMPTY_ROOT_SDF));
}

/////////////////////////////////////////////////
void FixedJointReductionCollisionVisualExtension(const std::string &_urdfFile,
                                                 const std::string &_sdfFile)
{
  // load urdf file, load sdf file.
  // check to see that urdf load results are consistent with sdf load.

  // load urdf
  sdf::SDFPtr urdfRobot = sdf::readFile(_urdfFile);
  ASSERT_TRUE(urdfRobot != sdf::SDFPtr());

  // load sdf
  sdf::SDFPtr sdfRobot(new sdf::SDF());
  sdf::init(sdfRobot);
  ASSERT_TRUE(sdf::readFile(_sdfFile, sdfRobot));

  // check two loaded files, make sure they are the same
  sdf::ElementPtr urdfModel = urdfRobot->Root()->GetElement("model");
  sdf::ElementPtr urdf_child_link_1_col;
  sdf::ElementPtr urdf_child_link_1a_col;
  sdf::ElementPtr urdf_child_link_2_col;
  sdf::ElementPtr urdf_child_link_1_vis;
  sdf::ElementPtr urdf_child_link_1a_vis;
  sdf::ElementPtr urdf_child_link_2_vis;
  for (sdf::ElementPtr link = urdfModel->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    EXPECT_FALSE(link->HasElement("gravity"));
    EXPECT_FALSE(link->HasElement("self_collide"));
    EXPECT_FALSE(link->HasElement("velocity_decay"));
    for (sdf::ElementPtr col = link->GetElement("collision"); col;
         col = col->GetNextElement("collision"))
    {
      std::string colName = col->Get<std::string>("name");
      if (colName == "base_link_fixed_joint_lump__child_link_1_collision_1")
      {
        urdf_child_link_1_col = col;
      }
      else if (colName ==
              "base_link_fixed_joint_lump__child_link_1a_collision_3")
      {
        urdf_child_link_1a_col = col;
      }
      else if (colName ==
              "base_link_fixed_joint_lump__child_link_2_collision_2")
      {
        urdf_child_link_2_col = col;
      }
      sdfmsg << "col: " << colName << "\n";
    }
    for (sdf::ElementPtr vis = link->GetElement("visual"); vis;
         vis = vis->GetNextElement("visual"))
    {
      std::string visName = vis->Get<std::string>("name");
      if (visName == "base_link_fixed_joint_lump__child_link_1_visual_1")
      {
        urdf_child_link_1_vis = vis;
      }
      else if (visName == "base_link_fixed_joint_lump__child_link_1a_visual_3")
      {
        urdf_child_link_1a_vis = vis;
      }
      else if (visName == "base_link_fixed_joint_lump__child_link_2_visual_2")
      {
        urdf_child_link_2_vis = vis;
      }
      sdfmsg << "vis: " << visName << "\n";
    }
  }
  sdf::ElementPtr sdfModel = sdfRobot->Root()->GetElement("model");
  sdf::ElementPtr sdf_child_link_1_col;
  sdf::ElementPtr sdf_child_link_1a_col;
  sdf::ElementPtr sdf_child_link_2_col;
  sdf::ElementPtr sdf_child_link_1_vis;
  sdf::ElementPtr sdf_child_link_1a_vis;
  sdf::ElementPtr sdf_child_link_2_vis;
  for (sdf::ElementPtr link = sdfModel->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    EXPECT_FALSE(link->HasElement("gravity"));
    EXPECT_FALSE(link->HasElement("self_collide"));
    EXPECT_FALSE(link->HasElement("velocity_decay"));
    for (sdf::ElementPtr col = link->GetElement("collision"); col;
         col = col->GetNextElement("collision"))
    {
      std::string colName = col->Get<std::string>("name");
      if (colName == "base_link_fixed_joint_lump__child_link_1_collision_1")
      {
        sdf_child_link_1_col = col;
      }
      else if (colName ==
              "base_link_fixed_joint_lump__child_link_1a_collision_3")
      {
        sdf_child_link_1a_col = col;
      }
      else if (colName ==
              "base_link_fixed_joint_lump__child_link_2_collision_2")
      {
        sdf_child_link_2_col = col;
      }
      sdfmsg << "col: " << colName << "\n";
    }
    for (sdf::ElementPtr vis = link->GetElement("visual"); vis;
         vis = vis->GetNextElement("visual"))
    {
      std::string visName = vis->Get<std::string>("name");
      if (visName == "base_link_fixed_joint_lump__child_link_1_visual_1")
      {
        sdf_child_link_1_vis = vis;
      }
      else if (visName == "base_link_fixed_joint_lump__child_link_1a_visual_3")
      {
        sdf_child_link_1a_vis = vis;
      }
      else if (visName == "base_link_fixed_joint_lump__child_link_2_visual_2")
      {
        sdf_child_link_2_vis = vis;
      }
      sdfmsg << "vis: " << visName << "\n";
    }
  }
  // child_link_1
  //   <collision name='base_link_fixed_joint_lump__child_link_1_collision_1'>
  //     <minDepth>0.007</minDepth>
  //     <mu1>0.7</mu1>
  //     <mu2>0.71</mu2>
  //   <visual name='base_link_fixed_joint_lump__child_link_1_visual_1'>
  //     <ambient>0 1 0 1</ambient>
  //     <script><name>script_uri_71_name</name></script>
  EXPECT_EQ(urdf_child_link_1_col->Get<int>("max_contacts"), 177);
  EXPECT_EQ(urdf_child_link_1_col->Get<int>("max_contacts"),
             sdf_child_link_1_col->Get<int>("max_contacts"));

  double urdf_mu1 = urdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu");
  double sdf_mu1 = sdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu");
  sdfmsg << "urdf mu1: " << urdf_mu1 << "\n";
  EXPECT_DOUBLE_EQ(urdf_mu1, 0.7);
  EXPECT_DOUBLE_EQ(urdf_mu1, sdf_mu1);

  double urdf_mu2 = urdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2");
  double sdf_mu2 = sdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2");
  sdfmsg << "urdf mu2: " << urdf_mu2 << "\n";
  EXPECT_DOUBLE_EQ(urdf_mu2, 0.71);
  EXPECT_DOUBLE_EQ(urdf_mu2, sdf_mu2);

  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
            "script_uri_71_name");
  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
             sdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"));

  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("uri"),
            "file://media/materials/scripts/gazebo.material");

  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            gz::math::Color(0, 1, 0, 1));
  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
             sdf_child_link_1_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"));

  // child_link_1a
  //   <collision name='base_link_fixed_joint_lump__child_link_1a_collision_3'>
  //     <maxContacts>166</maxContacts>
  //     <mu1>0.6</mu1>
  //     <mu2>0.61</mu2>
  //  <visual name='base_link_fixed_joint_lump__child_link_1a_visual_3'>
  //    <material>
  //      <script>
  //        <uri>__default__</uri>
  //        <name>__default__</name>
  //      </script>
  //    </material>
  EXPECT_EQ(urdf_child_link_1a_col->Get<int>("max_contacts"), 166);
  EXPECT_EQ(urdf_child_link_1a_col->Get<int>("max_contacts"),
            sdf_child_link_1a_col->Get<int>("max_contacts"));

  EXPECT_DOUBLE_EQ(urdf_child_link_1a_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu"), 0.6);
  EXPECT_DOUBLE_EQ(urdf_child_link_1a_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu"),
    sdf_child_link_1a_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu"));

  EXPECT_DOUBLE_EQ(urdf_child_link_1a_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2"), 0.61);
  EXPECT_DOUBLE_EQ(urdf_child_link_1a_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2"),
    sdf_child_link_1a_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2"));

  EXPECT_EQ(urdf_child_link_1a_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
            "__default__");
  EXPECT_EQ(urdf_child_link_1a_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
             sdf_child_link_1a_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"));

  EXPECT_EQ(urdf_child_link_1a_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("uri"),
            "__default__");

  // ambient unassigned should be 0, 0, 0, 1
  EXPECT_EQ(urdf_child_link_1a_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            gz::math::Color(0, 0, 0, 1));
  EXPECT_EQ(urdf_child_link_1a_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            sdf_child_link_1a_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"));

  // child_link_2
  //   <collision name='base_link_fixed_joint_lump__child_link_2_collision_2'>
  //     <mu1>0.5</mu1>
  //     <mu2>0.51</mu2>
  //   <visual name='base_link_fixed_joint_lump__child_link_2_visual_2'>
  //     <uri>script_uri_51</uri>
  //     <name>script_name_51</name>
  // unassign max_contacts default is 10
  EXPECT_EQ(urdf_child_link_2_col->Get<int>("max_contacts"), 10);
  EXPECT_EQ(urdf_child_link_2_col->Get<int>("max_contacts"),
            sdf_child_link_2_col->Get<int>("max_contacts"));

  EXPECT_DOUBLE_EQ(urdf_child_link_2_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu"), 0.5);
  EXPECT_DOUBLE_EQ(urdf_child_link_2_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu"),
    sdf_child_link_2_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu"));

  EXPECT_DOUBLE_EQ(urdf_child_link_2_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2"), 0.51);
  EXPECT_DOUBLE_EQ(urdf_child_link_2_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2"),
    sdf_child_link_2_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2"));

  EXPECT_EQ(urdf_child_link_2_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
            "script_name_51");
  EXPECT_EQ(urdf_child_link_2_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
            sdf_child_link_2_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"));

  EXPECT_EQ(urdf_child_link_2_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("uri"),
            "script_uri_51");

  // ambient unassigned should be 0, 0, 0, 1
  EXPECT_EQ(urdf_child_link_2_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            gz::math::Color(0, 0, 0, 1));
  EXPECT_EQ(urdf_child_link_2_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            sdf_child_link_2_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"));
}

/////////////////////////////////////////////////
void FixedJointReductionCollisionVisualExtensionEmptyRoot(
  const std::string &_urdfFile, const std::string &_sdfFile)
{
  // load urdf file, load sdf file.
  // check to see that urdf load results are consistent with sdf load.

  // load urdf
  sdf::SDFPtr urdfRobot(new sdf::SDF());
  sdf::init(urdfRobot);
  ASSERT_TRUE(sdf::readFile(_urdfFile, urdfRobot));

  // load sdf
  sdf::SDFPtr sdfRobot(new sdf::SDF());
  sdf::init(sdfRobot);
  ASSERT_TRUE(sdf::readFile(_sdfFile, sdfRobot));

  // check two loaded files, make sure they are the same
  sdf::ElementPtr urdfModel = urdfRobot->Root()->GetElement("model");
  sdf::ElementPtr urdf_child_link_1_col;
  sdf::ElementPtr urdf_child_link_1_vis;
  for (sdf::ElementPtr link = urdfModel->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    for (sdf::ElementPtr col = link->GetElement("collision"); col;
         col = col->GetNextElement("collision"))
    {
      std::string colName = col->Get<std::string>("name");
      // note that if there is only one child visual, no _1 is appended
      // compare this with FixedJointReductionCollisionVisualExtension test
      if (colName == "base_link_fixed_joint_lump__child_link_1_collision")
      {
        urdf_child_link_1_col = col;
      }
      sdfmsg << "col: " << colName << "\n";
    }
    for (sdf::ElementPtr vis = link->GetElement("visual"); vis;
         vis = vis->GetNextElement("visual"))
    {
      std::string visName = vis->Get<std::string>("name");
      // note that if there is only one child visual, no _1 is appended
      // compare this with FixedJointReductionCollisionVisualExtension test
      if (visName == "base_link_fixed_joint_lump__child_link_1_visual")
      {
        urdf_child_link_1_vis = vis;
      }
      sdfmsg << "vis: " << visName << "\n";
    }
  }
  sdf::ElementPtr sdfModel = sdfRobot->Root()->GetElement("model");
  sdf::ElementPtr sdf_child_link_1_col;
  sdf::ElementPtr sdf_child_link_1_vis;
  for (sdf::ElementPtr link = sdfModel->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    for (sdf::ElementPtr col = link->GetElement("collision"); col;
         col = col->GetNextElement("collision"))
    {
      std::string colName = col->Get<std::string>("name");
      // note that if there is only one child visual, no _1 is appended
      // compare this with FixedJointReductionCollisionVisualExtension test
      if (colName == "base_link_fixed_joint_lump__child_link_1_collision")
      {
        sdf_child_link_1_col = col;
      }
      sdfmsg << "col: " << colName << "\n";
    }
    for (sdf::ElementPtr vis = link->GetElement("visual"); vis;
         vis = vis->GetNextElement("visual"))
    {
      std::string visName = vis->Get<std::string>("name");
      // note that if there is only one child visual, no _1 is appended
      // compare this with FixedJointReductionCollisionVisualExtension test
      if (visName == "base_link_fixed_joint_lump__child_link_1_visual")
      {
        sdf_child_link_1_vis = vis;
      }
      sdfmsg << "vis: " << visName << "\n";
    }
  }
  // child_link_1
  //   <collision name='base_link_fixed_joint_lump__child_link_1_collision_1'>
  //     <minDepth>0.007</minDepth>
  //     <mu1>0.7</mu1>
  //     <mu2>0.71</mu2>
  //   <visual name='base_link_fixed_joint_lump__child_link_1_visual_1'>
  //     <ambient>0 1 0 1</ambient>
  //     <script><name>script_uri_71_name</name></script>
  EXPECT_EQ(urdf_child_link_1_col->Get<int>("max_contacts"), 177);
  EXPECT_EQ(urdf_child_link_1_col->Get<int>("max_contacts"),
            sdf_child_link_1_col->Get<int>("max_contacts"));

  double urdf_mu1 = urdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu");
  double sdf_mu1 = sdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu");
  sdfmsg << "urdf mu1: " << urdf_mu1 << "\n";
  EXPECT_DOUBLE_EQ(urdf_mu1, 0.7);
  EXPECT_DOUBLE_EQ(urdf_mu1, sdf_mu1);

  double urdf_mu2 = urdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2");
  double sdf_mu2 = sdf_child_link_1_col->GetElement("surface")
    ->GetElement("friction")->GetElement("ode")->Get<double>("mu2");
  sdfmsg << "urdf mu2: " << urdf_mu2 << "\n";
  EXPECT_DOUBLE_EQ(urdf_mu2, 0.71);
  EXPECT_DOUBLE_EQ(urdf_mu2, sdf_mu2);

  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
            "script_uri_71_name");
  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"),
            sdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("name"));

  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            GetElement("script")->Get<std::string>("uri"),
            "file://media/materials/scripts/gazebo.material");

  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            gz::math::Color(0, 1, 0, 1));
  EXPECT_EQ(urdf_child_link_1_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"),
            sdf_child_link_1_vis->GetElement("material")->
            Get<gz::math::Color>("ambient"));
}

/////////////////////////////////////////////////
void FixedJointReductionEquivalence(const std::string &_file)
{
  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(_file, robot));

  std::map<std::string, double> linkMasses;
  std::map<std::string, gz::math::Pose3d> linkPoses;
  std::map<std::string, gz::math::Vector3d> mapIxxIyyIzz;
  std::map<std::string, gz::math::Vector3d> mapIxyIxzIyz;

  {
    std::string linkName = "link0";
    linkMasses[linkName] = 1000;
    linkPoses[linkName] = gz::math::Pose3d(0, 0, -0.5, 0, -0, 0);
    mapIxxIyyIzz[linkName] = gz::math::Vector3d(1, 1, 1);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link1";
    linkMasses[linkName] = 100;
    linkPoses[linkName] =
      gz::math::Pose3d(0, -1.5, 0, -2.14159, 0.141593, 0.858407);
    mapIxxIyyIzz[linkName] = gz::math::Vector3d(2, 3, 4);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link2";
    linkMasses[linkName] = 200;
    linkPoses[linkName] =
      gz::math::Pose3d(0.2, 0.4, 1, -1.14159, -0.141593, 1.5708);
    mapIxxIyyIzz[linkName] = gz::math::Vector3d(5, 6, 7);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link3";
    linkMasses[linkName] = 400;
    linkPoses[linkName] =
      gz::math::Pose3d(0.1, 0.2, 0.3, -1.14159, 0.141593, 0.858407);
    mapIxxIyyIzz[linkName] = gz::math::Vector3d(8, 9, 10);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link1a";
    linkMasses[linkName] = 700;
    linkPoses[linkName] = gz::math::Pose3d(
      1.6, -2.72857, -0.342857, -2.14159, 0.141593, 0.858407);
    mapIxxIyyIzz[linkName] =
      gz::math::Vector3d(576.477313, 527.680922, 420.127477);
    mapIxyIxzIyz[linkName] =
      gz::math::Vector3d(-65.680839, 134.562430, 264.780538);
  }

  // Count collisions and visuals
  unsigned int countVisuals = 0;
  unsigned int countCollisions = 0;

  sdf::ElementPtr model = robot->Root()->GetElement("model");
  for (sdf::ElementPtr link = model->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    std::string linkName = link->Get<std::string>("name");
    sdf::ElementPtr inertial = link->GetElement("inertial");
    double linkMass = inertial->Get<double>("mass");
    gz::math::Pose3d linkPose =
      inertial->Get<gz::math::Pose3d>("pose");
    sdf::ElementPtr inertia = inertial->GetElement("inertia");
    double ixx = inertia->Get<double>("ixx");
    double iyy = inertia->Get<double>("iyy");
    double izz = inertia->Get<double>("izz");
    double ixy = inertia->Get<double>("ixy");
    double ixz = inertia->Get<double>("ixz");
    double iyz = inertia->Get<double>("iyz");

    sdfmsg << "name [" << linkName
           << "] m[" << linkMass
           << "] p[" << linkPose
           << "] ixx[" << ixx
           << "] iyy[" << iyy
           << "] izz[" << izz
           << "] ixy[" << ixy
           << "] ixz[" << ixz
           << "] iyz[" << iyz
           << "]\n";

    EXPECT_NEAR(linkMass, linkMasses[linkName], gc_tolerance);
    EXPECT_EQ(linkPose, linkPoses[linkName]);
    EXPECT_NEAR(ixx, mapIxxIyyIzz[linkName].X(), gc_tolerance);
    EXPECT_NEAR(iyy, mapIxxIyyIzz[linkName].Y(), gc_tolerance);
    EXPECT_NEAR(izz, mapIxxIyyIzz[linkName].Z(), gc_tolerance);
    EXPECT_NEAR(ixy, mapIxyIxzIyz[linkName].X(), gc_tolerance);
    EXPECT_NEAR(ixz, mapIxyIxzIyz[linkName].Y(), gc_tolerance);
    EXPECT_NEAR(iyz, mapIxyIxzIyz[linkName].Z(), gc_tolerance);

    if (link->HasElement("collision"))
    {
      for (sdf::ElementPtr coll = link->GetElement("collision"); coll;
           coll = coll->GetNextElement("collision"))
      {
        ++countCollisions;
      }
    }
    if (link->HasElement("visual"))
    {
      for (sdf::ElementPtr vis = link->GetElement("visual"); vis;
           vis = vis->GetNextElement("visual"))
      {
        ++countVisuals;
      }
    }
  }

  if (_file.compare(GetFullTestFilePath(SDF_TEST_FILE)) == 0)
  {
    EXPECT_EQ(countCollisions, 7u);
    EXPECT_EQ(countVisuals, 7u);
  }
  else if (_file.compare(GetFullTestFilePath(SDF_TEST_FILE_COLLISION)) == 0)
  {
    EXPECT_EQ(countCollisions, 6u);
    EXPECT_EQ(countVisuals, 0u);
  }
  else if (_file.compare(GetFullTestFilePath(SDF_TEST_FILE_VISUAL)) == 0)
  {
    EXPECT_EQ(countCollisions, 0u);
    EXPECT_EQ(countVisuals, 6u);
  }
}

/////////////////////////////////////////////////
TEST(SDFParser, FixedJointReductionSimple)
{
  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(GetFullTestFilePath(SDF_TEST_FILE_SIMPLE), robot));

  std::map<std::string, double> linkMasses;
  std::map<std::string, gz::math::Pose3d> linkPoses;
  std::map<std::string, gz::math::Vector3d> mapIxxIyyIzz;
  std::map<std::string, gz::math::Vector3d> mapIxyIxzIyz;

  const gz::math::Vector3d defaultIxxIyyIzz(7, 9, 11);
  {
    std::string linkName = "link1";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = gz::math::Pose3d();
    mapIxxIyyIzz[linkName] = defaultIxxIyyIzz;
    mapIxyIxzIyz[linkName] = gz::math::Vector3d();
  }

  {
    std::string linkName = "link1a";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = gz::math::Pose3d();
    mapIxxIyyIzz[linkName] = defaultIxxIyyIzz;
    mapIxyIxzIyz[linkName] = gz::math::Vector3d();
  }

  {
    std::string linkName = "link1b";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = gz::math::Pose3d();
    mapIxxIyyIzz[linkName] = defaultIxxIyyIzz;
    mapIxyIxzIyz[linkName] = gz::math::Vector3d();
  }

  {
    std::string linkName = "link1c";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = gz::math::Pose3d(0.2, 0, 0, 0, 0, 0);
    mapIxxIyyIzz[linkName] =
      defaultIxxIyyIzz + gz::math::Vector3d(0, 6, 6);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d();
  }

  {
    std::string linkName = "link1d";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = gz::math::Pose3d(0.2, 0, 0, 0, 0, 0);
    mapIxxIyyIzz[linkName] =
      defaultIxxIyyIzz + gz::math::Vector3d(0, 6, 6);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d();
  }

  {
    std::string linkName = "link1e";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = gz::math::Pose3d(0.2, 0, 0, 0, 0, 0);
    mapIxxIyyIzz[linkName] =
      defaultIxxIyyIzz + gz::math::Vector3d(0, 6, 6);
    mapIxyIxzIyz[linkName] = gz::math::Vector3d();
  }

  sdf::ElementPtr model = robot->Root()->GetElement("model");
  for (sdf::ElementPtr link = model->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    std::string linkName = link->Get<std::string>("name");
    sdf::ElementPtr inertial = link->GetElement("inertial");
    double linkMass = inertial->Get<double>("mass");
    gz::math::Pose3d linkPose =
      inertial->Get<gz::math::Pose3d>("pose");
    sdf::ElementPtr inertia = inertial->GetElement("inertia");
    double ixx = inertia->Get<double>("ixx");
    double iyy = inertia->Get<double>("iyy");
    double izz = inertia->Get<double>("izz");
    double ixy = inertia->Get<double>("ixy");
    double ixz = inertia->Get<double>("ixz");
    double iyz = inertia->Get<double>("iyz");

    sdfmsg << "name [" << linkName
           << "] m[" << linkMass
           << "] p[" << linkPose
           << "] ixx[" << ixx
           << "] iyy[" << iyy
           << "] izz[" << izz
           << "] ixy[" << ixy
           << "] ixz[" << ixz
           << "] iyz[" << iyz
           << "]\n";

    EXPECT_NEAR(linkMass, linkMasses[linkName], gc_tolerance);
    EXPECT_EQ(linkPose, linkPoses[linkName]);
    EXPECT_NEAR(ixx, mapIxxIyyIzz[linkName].X(), gc_tolerance);
    EXPECT_NEAR(iyy, mapIxxIyyIzz[linkName].Y(), gc_tolerance);
    EXPECT_NEAR(izz, mapIxxIyyIzz[linkName].Z(), gc_tolerance);
    EXPECT_NEAR(ixy, mapIxyIxzIyz[linkName].X(), gc_tolerance);
    EXPECT_NEAR(ixz, mapIxyIxzIyz[linkName].Y(), gc_tolerance);
    EXPECT_NEAR(iyz, mapIxyIxzIyz[linkName].Z(), gc_tolerance);
  }
}

/////////////////////////////////////////////////
// This test uses a urdf that has an SDFormat ball joint embedded in a
// <gazebo> tag, and the parent link part of a chain of fixed joints
// that reduces to the base_link.
// Test to make sure that the parent link name changes appropriately.
TEST(SDFParser, FixedJointReductionJointFrameExtensionTest)
{
  sdf::Root root;
  auto errors =
      root.Load(GetFullTestFilePath(SDF_TEST_FILE_JOINT_FRAME_EXTENSION));
  EXPECT_TRUE(errors.empty()) << errors[0].Message();

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("chained_fixed_joint_links", model->Name());

  EXPECT_EQ(3u, model->JointCount());
  const std::string ball_joint_name = "sdf_ball_joint";
  EXPECT_TRUE(model->JointNameExists(ball_joint_name));
  const sdf::Joint *joint = model->JointByName(ball_joint_name);
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("link3", joint->ChildName());
  EXPECT_EQ("base_link", joint->ParentName());
}

/////////////////////////////////////////////////
// This test uses a urdf that has chained fixed joints with plugin that
// contains bodyName, xyzOffset and rpyOffset.
// Test to make sure that the bodyName is updated to the new base link
// and that the offsets have the correct transform and frame of reference
TEST(SDFParser, FixedJointReductionPluginFrameExtensionTest)
{
  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(
      GetFullTestFilePath(SDF_TEST_FILE_PLUGIN_FRAME_EXTENSION), robot));

  sdf::ElementPtr model = robot->Root()->GetElement("model");
  sdf::ElementPtr plugin = model->GetElement("plugin");

  auto xyzOffset = plugin->Get<gz::math::Vector3d>("xyzOffset");
  auto rpyOffset = plugin->Get<gz::math::Vector3d>("rpyOffset");
  auto bodyName = plugin->Get<std::string>("bodyName");
  EXPECT_EQ("base_link", bodyName);
  EXPECT_EQ(gz::math::Vector3d(-0.707108, 1.70711, 0), xyzOffset);
  EXPECT_EQ(gz::math::Vector3d(0, 0, 1.5708), rpyOffset);

  bool correctedOffset = plugin->Get<bool>("gz::corrected_offsets");
  EXPECT_TRUE(correctedOffset);
}
