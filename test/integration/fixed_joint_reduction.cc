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
#include <map>
#include <boost/filesystem.hpp>
#include "sdf/sdf.hh"

#include "test_config.h"

const std::string SDF_TEST_FILE = std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/fixed_joint_reduction.urdf";
const std::string SDF_TEST_FILE_COLLISION =
  std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/fixed_joint_reduction_collision.urdf";
const std::string SDF_TEST_FILE_SIMPLE =
  std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/fixed_joint_reduction_simple.urdf";
const std::string SDF_TEST_FILE_VISUAL =
  std::string(PROJECT_SOURCE_PATH)
  + "/test/integration/fixed_joint_reduction_visual.urdf";

const double gc_tolerance = 1e-6;

void FixedJointReductionEquivalence(const std::string &_file);

/////////////////////////////////////////////////
TEST(SDFParser, FixedJointReductionEquivalenceTest)
{
  FixedJointReductionEquivalence(SDF_TEST_FILE);
}

/////////////////////////////////////////////////
// This test uses a urdf that has a fixed joint whose parent
// has no visuals. This can cause a seg-fault with the wrong
// version of urdfdom (see #63).
TEST(SDFParser, FixedJointReductionVisualTest)
{
  FixedJointReductionEquivalence(SDF_TEST_FILE_VISUAL);
}

/////////////////////////////////////////////////
// This test uses a urdf that has a fixed joint whose parent
// has no collisions. This is a parallel of the
// FixedJointReductionVisualTest
TEST(SDFParser, FixedJointReductionCollisionTest)
{
  FixedJointReductionEquivalence(SDF_TEST_FILE_COLLISION);
}

/////////////////////////////////////////////////
void FixedJointReductionEquivalence(const std::string &_file)
{
  char *pathCStr = getenv("SDF_PATH");
  boost::filesystem::path path = PROJECT_SOURCE_PATH;
  path = path / "sdf" / SDF_VERSION;
  setenv("SDF_PATH", path.string().c_str(), 1);

  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(_file, robot));
  if (pathCStr)
  {
    setenv("SDF_PATH", pathCStr, 1);
  }
  else
  {
    unsetenv("SDF_PATH");
  }

  std::map<std::string, double> linkMasses;
  std::map<std::string, ignition::math::Pose3d> linkPoses;
  std::map<std::string, ignition::math::Vector3d> mapIxxIyyIzz;
  std::map<std::string, ignition::math::Vector3d> mapIxyIxzIyz;

  {
    std::string linkName = "link0";
    linkMasses[linkName] = 1000;
    linkPoses[linkName] = ignition::math::Pose3d(0, 0, -0.5, 0, -0, 0);
    mapIxxIyyIzz[linkName] = ignition::math::Vector3d(1, 1, 1);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link1";
    linkMasses[linkName] = 100;
    linkPoses[linkName] =
      ignition::math::Pose3d(0, -1.5, 0, -2.14159, 0.141593, 0.858407);
    mapIxxIyyIzz[linkName] = ignition::math::Vector3d(2, 3, 4);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link2";
    linkMasses[linkName] = 200;
    linkPoses[linkName] =
      ignition::math::Pose3d(0.2, 0.4, 1, -1.14159, -0.141593, 1.5708);
    mapIxxIyyIzz[linkName] = ignition::math::Vector3d(5, 6, 7);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link3";
    linkMasses[linkName] = 400;
    linkPoses[linkName] =
      ignition::math::Pose3d(0.1, 0.2, 0.3, -1.14159, 0.141593, 0.858407);
    mapIxxIyyIzz[linkName] = ignition::math::Vector3d(8, 9, 10);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d(0, 0, 0);
  }

  {
    std::string linkName = "link1a";
    linkMasses[linkName] = 700;
    linkPoses[linkName] = ignition::math::Pose3d(
      1.6, -2.72857, -0.342857, -2.14159, 0.141593, 0.858407);
    mapIxxIyyIzz[linkName] =
      ignition::math::Vector3d(576.477, 527.681, 420.127);
    mapIxyIxzIyz[linkName] =
      ignition::math::Vector3d(-65.6808, 134.562, 264.781);
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
    ignition::math::Pose3d linkPose =
      inertial->Get<ignition::math::Pose3d>("pose");
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

  if (_file.compare(SDF_TEST_FILE) == 0)
  {
    EXPECT_EQ(countCollisions, 7u);
    EXPECT_EQ(countVisuals, 7u);
  }
  else if (_file.compare(SDF_TEST_FILE_COLLISION) == 0)
  {
    EXPECT_EQ(countCollisions, 6u);
    EXPECT_EQ(countVisuals, 0u);
  }
  else if (_file.compare(SDF_TEST_FILE_VISUAL) == 0)
  {
    EXPECT_EQ(countCollisions, 0u);
    EXPECT_EQ(countVisuals, 6u);
  }
}

/////////////////////////////////////////////////
TEST(SDFParser, FixedJointReductionSimple)
{
  char *pathCStr = getenv("SDF_PATH");
  boost::filesystem::path path = PROJECT_SOURCE_PATH;
  path = path / "sdf" / SDF_VERSION;
  setenv("SDF_PATH", path.string().c_str(), 1);

  sdf::SDFPtr robot(new sdf::SDF());
  sdf::init(robot);
  ASSERT_TRUE(sdf::readFile(SDF_TEST_FILE_SIMPLE, robot));
  if (pathCStr)
  {
    setenv("SDF_PATH", pathCStr, 1);
  }
  else
  {
    unsetenv("SDF_PATH");
  }

  std::map<std::string, double> linkMasses;
  std::map<std::string, ignition::math::Pose3d> linkPoses;
  std::map<std::string, ignition::math::Vector3d> mapIxxIyyIzz;
  std::map<std::string, ignition::math::Vector3d> mapIxyIxzIyz;

  const ignition::math::Vector3d defaultIxxIyyIzz(7, 9, 11);
  {
    std::string linkName = "link1";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = ignition::math::Pose3d();
    mapIxxIyyIzz[linkName] = defaultIxxIyyIzz;
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d();
  }

  {
    std::string linkName = "link1a";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = ignition::math::Pose3d();
    mapIxxIyyIzz[linkName] = defaultIxxIyyIzz;
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d();
  }

  {
    std::string linkName = "link1b";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = ignition::math::Pose3d();
    mapIxxIyyIzz[linkName] = defaultIxxIyyIzz;
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d();
  }

  {
    std::string linkName = "link1c";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = ignition::math::Pose3d(0.2, 0, 0, 0, 0, 0);
    mapIxxIyyIzz[linkName] =
      defaultIxxIyyIzz + ignition::math::Vector3d(0, 6, 6);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d();
  }

  {
    std::string linkName = "link1d";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = ignition::math::Pose3d(0.2, 0, 0, 0, 0, 0);
    mapIxxIyyIzz[linkName] =
      defaultIxxIyyIzz + ignition::math::Vector3d(0, 6, 6);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d();
  }

  {
    std::string linkName = "link1e";
    linkMasses[linkName] = 300;
    linkPoses[linkName] = ignition::math::Pose3d(0.2, 0, 0, 0, 0, 0);
    mapIxxIyyIzz[linkName] =
      defaultIxxIyyIzz + ignition::math::Vector3d(0, 6, 6);
    mapIxyIxzIyz[linkName] = ignition::math::Vector3d();
  }

  sdf::ElementPtr model = robot->Root()->GetElement("model");
  for (sdf::ElementPtr link = model->GetElement("link"); link;
       link = link->GetNextElement("link"))
  {
    std::string linkName = link->Get<std::string>("name");
    sdf::ElementPtr inertial = link->GetElement("inertial");
    double linkMass = inertial->Get<double>("mass");
    ignition::math::Pose3d linkPose =
      inertial->Get<ignition::math::Pose3d>("pose");
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
