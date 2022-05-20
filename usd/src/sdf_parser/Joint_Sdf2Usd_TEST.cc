/*
 * Copyright 2022 Open Source Robotics Foundation
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

#include <unordered_map>
#include <string>

#include <gtest/gtest.h>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "test_config.h"
#include "test_utils.hh"
#include "Model.hh"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
// This fixture also provides methods for performing joint validation checks.
class UsdJointStageFixture : public::testing::Test
{
  public: UsdJointStageFixture() :
          worldPath("/world")
  {
  }

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
    ASSERT_TRUE(this->stage);
  }

  /// \brief Parse the contents of a SDF file and convert to USD.
  /// This method should be called at the beginning of a test case to set up
  /// the USD contents that need to be checked/verified.
  /// \param[in] _sdfFile The full path to the SDF file to parse
  public: void GenerateUSD(const std::string &_sdfFile)
  {
    // load the world in the SDF file
    ASSERT_TRUE(sdf::testing::LoadSdfFile(_sdfFile, this->root));
    this->model = const_cast<sdf::Model *>(this->root.Model());
    ASSERT_NE(nullptr, this->model);

    this->modelPath =
      std::string(this->worldPath.GetString() + "/" + this->model->Name());
    const auto errors = sdf::usd::ParseSdfModel(*(this->model), this->stage,
        this->modelPath, this->worldPath);
    EXPECT_TRUE(errors.empty());

    // save the model's USD joint paths so that they can be verified
    for (uint64_t i = 0; i < this->model->JointCount(); ++i)
    {
      const auto joint = this->model->JointByIndex(i);
      const auto jointPath = this->modelPath + "/" + joint->Name();
      this->jointPathToSdf[jointPath] = joint;
    }
    EXPECT_EQ(this->model->JointCount(), this->jointPathToSdf.size());
  }

  /// \brief Verify that a USD joint is pointing to the correct parent link.
  /// \param[in] _usdJoint The USD joint
  /// \param[in] _parentLinkPath The parent link path that _usdJoint should
  /// point to
  public: void CheckParentLinkPath(const pxr::UsdPhysicsJoint *_usdJoint,
              const std::string &_parentLinkPath) const
  {
    pxr::SdfPathVector jointTarget;
    _usdJoint->GetBody0Rel().GetTargets(&jointTarget);
    ASSERT_EQ(1u, jointTarget.size());
    EXPECT_EQ(_parentLinkPath, jointTarget[0].GetString());
  }

  /// \brief Verify that a USD joint is pointing to the correct child link.
  /// \param[in] _usdJoint The USD joint
  /// \param[in] _childLinkPath The child link path that _usdJoint should
  /// point to
  public: void CheckChildLinkPath(const pxr::UsdPhysicsJoint *_usdJoint,
              const std::string &_childLinkPath) const
  {
    pxr::SdfPathVector jointTarget;
    _usdJoint->GetBody1Rel().GetTargets(&jointTarget);
    ASSERT_EQ(1u, jointTarget.size());
    EXPECT_EQ(_childLinkPath, jointTarget[0].GetString());
  }

  /// \brief Verify that a USD joint has a proper pose w.r.t. its parent and
  /// child link.
  /// \param[in] _usdJoint The USD joint
  /// \param[in] _targetParentPose The pose _usdJoint should have w.r.t. its
  /// parent link
  /// \param[in] _targetChildPose The pose _usdJoint should have w.r.t. its
  /// child link
  public: void CheckRelativeLinkPoses(const pxr::UsdPhysicsJoint *_usdJoint,
              const gz::math::Pose3d &_targetParentPose,
              const gz::math::Pose3d &_targetChildPose) const
  {
    // helper function to compare USD position to gz::math position
    auto validatePos =
      [](const pxr::GfVec3f &_usdPos,
          const gz::math::Vector3d &_targetPos)
      {
        EXPECT_FLOAT_EQ(_usdPos[0], static_cast<float>(_targetPos.X()));
        EXPECT_FLOAT_EQ(_usdPos[1], static_cast<float>(_targetPos.Y()));
        EXPECT_FLOAT_EQ(_usdPos[2], static_cast<float>(_targetPos.Z()));
      };

    // helper function to compare USD rotation to gz::math quaternion
    auto validateRot =
      [](const pxr::GfQuatf &_usdRot,
          const gz::math::Quaterniond &_targetRot)
      {
        EXPECT_FLOAT_EQ(_usdRot.GetReal(),
            static_cast<float>(_targetRot.W()));
        EXPECT_FLOAT_EQ(_usdRot.GetImaginary()[0],
            static_cast<float>(_targetRot.X()));
        EXPECT_FLOAT_EQ(_usdRot.GetImaginary()[1],
            static_cast<float>(_targetRot.Y()));
        EXPECT_FLOAT_EQ(_usdRot.GetImaginary()[2],
            static_cast<float>(_targetRot.Z()));
      };

    pxr::GfVec3f usdPos;
    pxr::GfQuatf usdRot;

    EXPECT_TRUE(_usdJoint->GetLocalPos0Attr().Get(&usdPos));
    validatePos(usdPos, _targetParentPose.Pos());

    EXPECT_TRUE(_usdJoint->GetLocalRot0Attr().Get(&usdRot));
    validateRot(usdRot, _targetParentPose.Rot());

    EXPECT_TRUE(_usdJoint->GetLocalPos1Attr().Get(&usdPos));
    validatePos(usdPos, _targetChildPose.Pos());

    EXPECT_TRUE(_usdJoint->GetLocalRot1Attr().Get(&usdRot));
    validateRot(usdRot, _targetChildPose.Rot());
  }

  /// \brief Verify that a USD joint has a proper axis (this is not required
  /// by all USD joint types, but is required by revolute and prismatic joints,
  /// for example)
  /// \param[in] _usdJoint The USD joint
  /// \param[in] _targetAxis The axis _usdJoint should have. This should be
  /// "X", "Y", or "Z"
  /// \tparam JointTypeT A USD joint type that has a GetAxisAttr() method, which
  /// returns a pxr::UsdAttribute that stores the axis in a pxr::TfToken object
  public: template<typename JointTypeT>
          void VerifyJointAxis(const JointTypeT &_usdJoint,
              const std::string &_targetAxis) const
  {
    pxr::TfToken usdAxis;
    EXPECT_TRUE(_usdJoint.GetAxisAttr().Get(&usdAxis));
    EXPECT_EQ(_targetAxis, usdAxis.GetString());
  }

  /// \brief Verify that a USD joint has the proper limits (this is not required
  /// by all USD joint types, but is required by revolute and prismatic joints,
  /// for example)
  /// \param[in] _usdJoint The USD joint
  /// \param[in] _targetLower The lower limit _usdJoint should have. For
  /// revolute joints, USD interprets this as an angle, but for prismatic
  /// joints, USD interprets this as a distance
  /// \param[in] _targetUpper The upper limit _usdJoint should have. For
  /// revolute joints, USD interprets this as an angle, but for prismatic
  /// joints, USD interprets this as a distance
  /// \param[in] _convertToDeg Whether _targetLower and _targetUpper need to be
  /// converted to degrees (true) or not (false). If _usdJoint is a prismatic
  /// joint, this value should be set to false since prismatic joint limits are
  /// interpreted as a distance instead of an angle
  /// \tparam JointTypeT A USD joint type that has GetLowerLimitAttr() and
  /// GetUpperLimitAttr() methods. Both of these methods should return a
  /// pxr::UsdAttribute that stores the limit as a float
  public: template<typename JointTypeT>
          void VerifyJointLimits(const JointTypeT &_usdJoint,
              float _targetLower, float _targetUpper, bool _convertToDeg) const
  {
    if (_convertToDeg)
    {
      _targetLower = IGN_RTOD(_targetLower);
      _targetUpper = IGN_RTOD(_targetUpper);
    }

    float usdLowerLimit;
    EXPECT_TRUE(_usdJoint.GetLowerLimitAttr().Get(&usdLowerLimit));
    EXPECT_FLOAT_EQ(usdLowerLimit, _targetLower);

    float usdUpperLimit;
    EXPECT_TRUE(_usdJoint.GetUpperLimitAttr().Get(&usdUpperLimit));
    EXPECT_FLOAT_EQ(usdUpperLimit, _targetUpper);
  }

  /// \brief The USD stage
  public: pxr::UsdStageRefPtr stage;

  /// \brief The SDF model with joints to be parsed to USD
  public: sdf::Model *model{nullptr};

  /// \brief The USD path of the SDF world
  public: const pxr::SdfPath worldPath;

  /// \brief The string representation of this->model's USD path
  public: std::string modelPath;

  /// \brief Mapping of a joint's USD path to the corresponding SDF joint
  public: std::unordered_map<std::string, const sdf::Joint *> jointPathToSdf;

  /// \brief The root object of the SDF file that has been loaded. A reference
  /// to the root must be kept so that pointer objects extracted from the root
  /// (sdf::Model and sdf::Joint, for example) remain valid throughout the test
  /// (destroying the sdf::Root object early invalidates referenced pointers)
  private: sdf::Root root;
};

/////////////////////////////////////////////////
TEST_F(UsdJointStageFixture, RevoluteJoints)
{
  this->GenerateUSD(sdf::testing::TestFile("sdf", "double_pendulum.sdf"));

  // validate USD joints
  int checkedJoints = 0;
  for (const auto & prim : this->stage->Traverse())
  {
    if (!prim.IsA<pxr::UsdPhysicsJoint>())
      continue;

    auto iter = this->jointPathToSdf.find(prim.GetPath().GetString());
    ASSERT_NE(this->jointPathToSdf.end(), iter);
    const auto sdfJoint = iter->second;

    // the double pendulum model only has revolute joints
    EXPECT_TRUE(prim.IsA<pxr::UsdPhysicsRevoluteJoint>());
    const auto usdRevoluteJoint =
      pxr::UsdPhysicsRevoluteJoint::Get(this->stage, prim.GetPath());
    ASSERT_TRUE(usdRevoluteJoint);

    // make sure joint is pointing to the proper parent/child links
    this->CheckParentLinkPath(&usdRevoluteJoint,
        this->modelPath + "/" + sdfJoint->ParentLinkName());
    this->CheckChildLinkPath(&usdRevoluteJoint,
        this->modelPath + "/" + sdfJoint->ChildLinkName());

    // check joint's pose w.r.t. parent and child links
    gz::math::Pose3d parentToJointPose;
    auto poseErrors = sdfJoint->SemanticPose().Resolve(parentToJointPose,
          sdfJoint->ParentLinkName());
    EXPECT_TRUE(poseErrors.empty());
    poseErrors.clear();
    gz::math::Pose3d childToJointPose;
    poseErrors = sdfJoint->SemanticPose().Resolve(childToJointPose,
          sdfJoint->ChildLinkName());
    EXPECT_TRUE(poseErrors.empty());
    this->CheckRelativeLinkPoses(&usdRevoluteJoint, parentToJointPose,
        childToJointPose);

    // check the joint's axis
    this->VerifyJointAxis(usdRevoluteJoint, "X");

    // check the joint limits
    this->VerifyJointLimits(usdRevoluteJoint,
        static_cast<float>(sdfJoint->Axis()->Lower()),
        static_cast<float>(sdfJoint->Axis()->Upper()),
        true);

    // revolute joints should have a UsdPhysicsDriveAPI
    EXPECT_TRUE(prim.HasAPI<pxr::UsdPhysicsDriveAPI>(pxr::TfToken("angular")));
    const auto driveApiAttrPrefix = std::string("drive:angular:physics:");

    // check damping
    const auto dampingAttr =
      prim.GetAttribute(pxr::TfToken(driveApiAttrPrefix + "damping"));
    ASSERT_TRUE(dampingAttr);
    float usdDamping;
    EXPECT_TRUE(dampingAttr.Get(&usdDamping));
    EXPECT_FLOAT_EQ(usdDamping,
        static_cast<float>(sdfJoint->Axis()->Damping()));

    // check stiffness
    const auto stiffnessAttr =
      prim.GetAttribute(pxr::TfToken(driveApiAttrPrefix + "stiffness"));
    ASSERT_TRUE(stiffnessAttr);
    float usdStiffness;
    EXPECT_TRUE(stiffnessAttr.Get(&usdStiffness));
    EXPECT_FLOAT_EQ(usdStiffness,
        static_cast<float>(sdfJoint->Axis()->Stiffness()));

    // check max force/effort
    const auto maxForceAttr =
      prim.GetAttribute(pxr::TfToken(driveApiAttrPrefix + "maxForce"));
    ASSERT_TRUE(maxForceAttr);
    float usdForce;
    EXPECT_TRUE(maxForceAttr.Get(&usdForce));
    EXPECT_FLOAT_EQ(usdForce, static_cast<float>(sdfJoint->Axis()->Effort()));

    checkedJoints++;
  }
  EXPECT_EQ(checkedJoints, 2);

  // the model prim that has revolute joints should be marked as a
  // pxr::UsdPhysicsAtriculationRootAPI
  const auto modelPrim =
    this->stage->GetPrimAtPath(pxr::SdfPath(this->modelPath));
  ASSERT_TRUE(modelPrim);
  EXPECT_TRUE(modelPrim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>());
}

/////////////////////////////////////////////////
TEST_F(UsdJointStageFixture, JointParentIsWorld)
{
  this->GenerateUSD(sdf::testing::TestFile("sdf", "joint_parent_world.sdf"));

  // validate USD joints
  int checkedJoints = 0;
  for (const auto & prim : this->stage->Traverse())
  {
    if (!prim.IsA<pxr::UsdPhysicsJoint>())
      continue;

    auto iter = this->jointPathToSdf.find(prim.GetPath().GetString());
    ASSERT_NE(this->jointPathToSdf.end(), iter);
    const auto sdfJoint = iter->second;

    // the only joint type in this test file is a fixed joint
    EXPECT_TRUE(prim.IsA<pxr::UsdPhysicsFixedJoint>());
    const auto usdFixedJoint =
      pxr::UsdPhysicsFixedJoint::Get(this->stage, prim.GetPath());
    ASSERT_TRUE(usdFixedJoint);

    // make sure joint is pointing to the proper parent/child links.
    // The parent in this test should be the world
    this->CheckParentLinkPath(&usdFixedJoint, this->worldPath.GetString());
    this->CheckChildLinkPath(&usdFixedJoint,
        this->modelPath + "/" + sdfJoint->ChildLinkName());

    // check joint's pose w.r.t. parent and child links. For this test case,
    // we need to get the joint pose w.r.t. the world
    gz::math::Pose3d modelToJointPose;
    auto poseErrors = sdfJoint->SemanticPose().Resolve(modelToJointPose);
    EXPECT_TRUE(poseErrors.empty());
    poseErrors.clear();
    gz::math::Pose3d worldToModelPose;
    poseErrors = this->model->SemanticPose().Resolve(worldToModelPose);
    const auto worldToJointPose = worldToModelPose * modelToJointPose;
    EXPECT_TRUE(poseErrors.empty());
    poseErrors.clear();
    gz::math::Pose3d childToJointPose;
    poseErrors = sdfJoint->SemanticPose().Resolve(childToJointPose,
          sdfJoint->ChildLinkName());
    EXPECT_TRUE(poseErrors.empty());
    this->CheckRelativeLinkPoses(&usdFixedJoint, worldToJointPose,
        childToJointPose);

    checkedJoints++;
  }
  EXPECT_EQ(checkedJoints, 1);

  // the model prim doesn't have revolute joints, so it shouldn't be marked as a
  // pxr::UsdPhysicsAtriculationRootAPI
  const auto modelPrim =
    this->stage->GetPrimAtPath(pxr::SdfPath(this->modelPath));
  ASSERT_TRUE(modelPrim);
  EXPECT_FALSE(modelPrim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>());
}

/////////////////////////////////////////////////
TEST_F(UsdJointStageFixture, BallPrismaticJoint)
{
  this->GenerateUSD(sdf::testing::TestFile("sdf", "ball_prismatic_joint.sdf"));

  // validate USD joints
  int checkedBallJoints = 0;
  int checkedPrismaticJoints = 0;
  for (const auto & prim : this->stage->Traverse())
  {
    if (!prim.IsA<pxr::UsdPhysicsJoint>())
      continue;

    auto iter = this->jointPathToSdf.find(prim.GetPath().GetString());
    ASSERT_NE(this->jointPathToSdf.end(), iter);
    const auto sdfJoint = iter->second;

    if (prim.IsA<pxr::UsdPhysicsSphericalJoint>())
    {
      checkedBallJoints++;
    }
    else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
      checkedPrismaticJoints++;

      const auto usdPrismaticJoint =
        pxr::UsdPhysicsPrismaticJoint::Get(this->stage, prim.GetPath());
      ASSERT_TRUE(usdPrismaticJoint);

      // check the joint's axis
      this->VerifyJointAxis(usdPrismaticJoint, "Z");

      // check the joint limits
      this->VerifyJointLimits(usdPrismaticJoint,
          static_cast<float>(sdfJoint->Axis()->Lower()),
          static_cast<float>(sdfJoint->Axis()->Upper()),
          false);
    }
    else
    {
      continue;
    }

    const auto usdJoint =
      pxr::UsdPhysicsJoint::Get(this->stage, prim.GetPath());
    ASSERT_TRUE(usdJoint);

    // make sure joint is pointing to the proper parent/child links
    this->CheckParentLinkPath(&usdJoint,
        this->modelPath + "/" + sdfJoint->ParentLinkName());
    this->CheckChildLinkPath(&usdJoint,
        this->modelPath + "/" + sdfJoint->ChildLinkName());

    // check joint's pose w.r.t. parent and child links
    gz::math::Pose3d parentToJointPose;
    auto poseErrors = sdfJoint->SemanticPose().Resolve(parentToJointPose,
          sdfJoint->ParentLinkName());
    EXPECT_TRUE(poseErrors.empty());
    poseErrors.clear();
    gz::math::Pose3d childToJointPose;
    poseErrors = sdfJoint->SemanticPose().Resolve(childToJointPose,
          sdfJoint->ChildLinkName());
    EXPECT_TRUE(poseErrors.empty());
    this->CheckRelativeLinkPoses(&usdJoint, parentToJointPose,
        childToJointPose);
  }
  EXPECT_EQ(checkedBallJoints, 1);
  EXPECT_EQ(checkedPrismaticJoints, 1);
}

// TODO(adlarkin) Add a test case for a revolute joint with the axis being "y".
// This is a special case; see the sdf::usd::SetUSDJointPose method in
// usd/src/sdf_parser/Joint.cc for how this is handled
