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

#include <gtest/gtest.h>
#include "sdf/sdf.hh"

#include "test_config.h"

const std::string SDF_TEST_FILE = std::string(PROJECT_SOURCE_PATH) + "/test/integration/fixed_joint_reduction.sdf";

TEST(SDFParser, FixedJointReductionEquivalentceTest)
{
   sdf::SDFPtr robot(new sdf::SDF());
   sdf::init(robot);
   ASSERT_TRUE(sdf::readFile(SDF_TEST_FILE, robot));

   robot->PrintValues();
   sdf::ElementPtr model = robot->root->GetElement("model");
   for(sdf::ElementPtr link = model->GetElement("link"); link; link = link->GetNextElement("link"))
   {
     std::string linkName = link->Get<std::string>("name");
     sdf::ElementPtr inertial = link->GetElement("inertial");
     double linkMass = inertial->Get<double>("mass");
     sdf::Pose linkPose = inertial->Get<sdf::Pose>("pose");
     sdf::ElementPtr inertia = inertial->GetElement("inertia");
     double ixx = inertia->Get<double>("ixx");
     double iyy = inertia->Get<double>("iyy");
     double izz = inertia->Get<double>("izz");
     double ixy = inertia->Get<double>("ixy");
     double ixz = inertia->Get<double>("ixz");
     double iyz = inertia->Get<double>("iyz");

     std::cout << "name [" << linkName
               << "] m[" << linkMass
               << "] p[" << linkPose
               << "] ixx[" << ixx
               << "] iyy[" << iyy
               << "] izz[" << izz
               << "] ixy[" << ixy
               << "] ixz[" << ixz
               << "] iyz[" << iyz
               << "]\n";


      if (linkName == "link0")
      {
        EXPECT_EQ(linkMass, 1000);
        EXPECT_EQ(linkPose, sdf::Pose(0, 0, -0.5, 0, -0, 0));
        EXPECT_EQ(ixx, 1);
        EXPECT_EQ(iyy, 1);
        EXPECT_EQ(izz, 1);
        EXPECT_EQ(ixy, 0);
        EXPECT_EQ(ixz, 0);
        EXPECT_EQ(iyz, 0);
      }
      else if (linkName == "link1")
      {
        EXPECT_EQ(linkMass, 100);
        EXPECT_EQ(linkPose, sdf::Pose(0, -1.5, 0, -2.14159, 0.141593, 0.858407));
        EXPECT_EQ(ixx, 2);
        EXPECT_EQ(iyy, 3);
        EXPECT_EQ(izz, 4);
        EXPECT_EQ(ixy, 0);
        EXPECT_EQ(ixz, 0);
        EXPECT_EQ(iyz, 0);
      }
      else if (linkName == "link2")
      {
        EXPECT_EQ(linkMass, 200);
        EXPECT_EQ(linkPose, sdf::Pose(0.2, 0.4, 1, -1.14159, -0.141593, 1.5708));
        EXPECT_EQ(ixx, 5);
        EXPECT_EQ(iyy, 6);
        EXPECT_EQ(izz, 7);
        EXPECT_EQ(ixy, 0);
        EXPECT_EQ(ixz, 0);
        EXPECT_EQ(iyz, 0);
      }
      else if (linkName == "link3")
      {
        EXPECT_EQ(linkMass, 400);
        EXPECT_EQ(linkPose, sdf::Pose(0.1, 0.2, 0.3, -1.14159, 0.141593, 0.858407));
        EXPECT_EQ(ixx, 8);
        EXPECT_EQ(iyy, 9);
        EXPECT_EQ(izz, 10);
        EXPECT_EQ(ixy, 0);
        EXPECT_EQ(ixz, 0);
        EXPECT_EQ(iyz, 0);
      }
      else if (linkName == "link1a")
      {
        EXPECT_EQ(linkMass, 700);
        EXPECT_EQ(linkPose, sdf::Pose(1.6, -2.72857, -0.342857, -2.14159, 0.141593, 0.858407));
        EXPECT_EQ(ixx, 576.477);
        EXPECT_EQ(iyy, 527.681);
        EXPECT_EQ(izz, 420.127);
        EXPECT_EQ(ixy, -65.6808);
        EXPECT_EQ(ixz, 134.562);
        EXPECT_EQ(iyz, 264.781);
      }
   }
}
