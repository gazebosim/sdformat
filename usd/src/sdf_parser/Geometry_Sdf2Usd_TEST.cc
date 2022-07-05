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

#include <string>

#include <gtest/gtest.h>

#include <gz/common/Util.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/sdf_parser/World.hh"
#include "sdf/Root.hh"
#include "test_config.hh"
#include "test_utils.hh"
#include "../UsdTestUtils.hh"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
class UsdStageFixture : public::testing::Test
{
  public: UsdStageFixture() = default;

  /// \brief Check a USD geometry's extent.
  /// \param[in] _extent The USD geometry extent
  /// \param[in] _targetMin The desired minimum extent
  /// \param[in] _targetMax The desired maximum extent
  public: void CheckExtent(const pxr::VtArray<pxr::GfVec3f> &_extent,
              const pxr::GfVec3f &_targetMin,
              const pxr::GfVec3f &_targetMax) const
  {
    // make sure _extend has 2 points: min and max
    ASSERT_EQ(2u, _extent.size());

    const auto extentMin = _extent[0];
    EXPECT_FLOAT_EQ(extentMin[0], _targetMin[0]);
    EXPECT_FLOAT_EQ(extentMin[1], _targetMin[1]);
    EXPECT_FLOAT_EQ(extentMin[2], _targetMin[2]);

    const auto extentMax = _extent[1];
    EXPECT_FLOAT_EQ(extentMax[0], _targetMax[0]);
    EXPECT_FLOAT_EQ(extentMax[1], _targetMax[1]);
    EXPECT_FLOAT_EQ(extentMax[2], _targetMax[2]);
  }

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
    ASSERT_TRUE(this->stage);
  }

  public: pxr::UsdStageRefPtr stage;
};

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Ellipsoid)
{
  const auto testFile = sdf::testing::TestFile("sdf", "ellipsoid.sdf");

  sdf::Root root;
  ASSERT_TRUE(sdf::testing::LoadSdfFile(testFile, root));
  ASSERT_EQ(1u, root.WorldCount());
  const auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  const auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  const auto usdSphere = pxr::UsdGeomSphere::Define(
    this->stage,
    pxr::SdfPath("/ellipsoid_world/ellipsoid/ellipsoid_link/"
                 "ellipsoid_visual/geometry"));
  ASSERT_TRUE(usdSphere);
  bool resetXformStack;
  const auto xformOps = usdSphere.GetOrderedXformOps(&resetXformStack);
  ASSERT_EQ(xformOps.size(), 1u);
  EXPECT_FALSE(resetXformStack);
  const auto scaleOp = xformOps[0];
  EXPECT_EQ(scaleOp.GetOpType(), pxr::UsdGeomXformOp::TypeScale);
  pxr::GfVec3f scaleValue;
  scaleOp.Get(&scaleValue);
  EXPECT_FLOAT_EQ(scaleValue[0], 0.2);
  EXPECT_FLOAT_EQ(scaleValue[1], 0.3);
  EXPECT_FLOAT_EQ(scaleValue[2], 0.5);

  double radius;
  EXPECT_TRUE(usdSphere.GetRadiusAttr().Get(&radius));
  EXPECT_DOUBLE_EQ(radius, 1);

  pxr::VtArray<pxr::GfVec3f> extent;
  usdSphere.GetExtentAttr().Get(&extent);
  ASSERT_EQ(extent.size(), 2u);
  EXPECT_FLOAT_EQ(extent[0][0], -1.0f);
  EXPECT_FLOAT_EQ(extent[0][1], -1.0f);
  EXPECT_FLOAT_EQ(extent[0][2], -1.0f);
  EXPECT_FLOAT_EQ(extent[1][0], 1.0f);
  EXPECT_FLOAT_EQ(extent[1][1], 1.0f);
  EXPECT_FLOAT_EQ(extent[1][2], 1.0f);
}

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Geometry)
{
  sdf::setFindCallback(sdf::usd::testing::findFileCb);
  gz::common::addFindFileURICallback(
    std::bind(&sdf::usd::testing::FindResourceUri, std::placeholders::_1));

  const auto path = sdf::testing::TestFile("sdf", "basic_shapes.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  const auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  const auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  const auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);

  const std::string groundPlanePath = worldPath + "/ground_plane";
  const std::string groundPlaneLinkPath = groundPlanePath + "/link";
  const std::string groundPlaneVisualPath = groundPlaneLinkPath + "/visual";
  const std::string groundPlaneGeometryPath =
    groundPlaneVisualPath + "/geometry";
  const std::string groundPlaneCollisionPath =
    groundPlaneLinkPath + "/collision";
  const std::string groundPlaneCollisionGeometryPath =
    groundPlaneCollisionPath + "/geometry";

  pxr::GfVec3f scale;
  pxr::VtArray<pxr::GfVec3f> extent;
  double size, height, radius, length;

  const auto groundPlaneCollisionGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(groundPlaneCollisionGeometryPath));
  ASSERT_TRUE(groundPlaneCollisionGeometry);
  EXPECT_TRUE(
    groundPlaneCollisionGeometry.HasAPI<pxr::UsdPhysicsCollisionAPI>());
  const auto groundPlaneGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(groundPlaneGeometryPath));
  ASSERT_TRUE(groundPlaneGeometry);
  const auto usdGroundPlane = pxr::UsdGeomCube(groundPlaneGeometry);
  ASSERT_TRUE(usdGroundPlane);
  usdGroundPlane.GetSizeAttr().Get(&size);
  EXPECT_DOUBLE_EQ(1.0, size);
  auto scaleAttr =
    groundPlaneGeometry.GetAttribute(pxr::TfToken("xformOp:scale"));
  ASSERT_TRUE(scaleAttr);
  scaleAttr.Get(&scale);
  EXPECT_EQ(pxr::GfVec3f(2, 4, 0.25), scale);
  sdf::usd::testing::HasScaleXFormOp(groundPlaneGeometry);
  usdGroundPlane.GetExtentAttr().Get(&extent);
  this->CheckExtent(extent, pxr::GfVec3f(-0.5), pxr::GfVec3f(0.5));
  extent.clear();

  const std::string boxPath = worldPath + "/box";
  const std::string boxLinkPath = boxPath + "/link";
  const std::string boxVisualPath = boxLinkPath + "/box_vis";
  const std::string boxGeometryPath = boxVisualPath + "/geometry";
  const auto boxGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(boxGeometryPath));
  const std::string boxCollisionPath = boxLinkPath + "/collision";
  const std::string boxCollisionGeometryPath = boxCollisionPath + "/geometry";
  const auto boxCollisionGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(boxCollisionGeometryPath));
  ASSERT_TRUE(boxCollisionGeometry);
  EXPECT_TRUE(boxCollisionGeometry.HasAPI<pxr::UsdPhysicsCollisionAPI>());

  ASSERT_TRUE(boxGeometry);
  const auto usdCube = pxr::UsdGeomCube(boxGeometry);
  ASSERT_TRUE(usdCube);
  usdCube.GetSizeAttr().Get(&size);
  EXPECT_DOUBLE_EQ(1.0, size);
  scaleAttr = boxGeometry.GetAttribute(pxr::TfToken("xformOp:scale"));
  ASSERT_TRUE(scaleAttr);
  scaleAttr.Get(&scale);
  EXPECT_EQ(pxr::GfVec3f(1, 2, 3), scale);
  sdf::usd::testing::HasScaleXFormOp(boxGeometry);
  usdCube.GetExtentAttr().Get(&extent);
  this->CheckExtent(extent, pxr::GfVec3f(-0.5), pxr::GfVec3f(0.5));
  extent.clear();

  const std::string cylinderPath = worldPath + "/cylinder";
  const std::string cylinderLinkPath = cylinderPath + "/link";
  const std::string cylinderVisualPath = cylinderLinkPath + "/visual";
  const std::string cylinderGeometryPath = cylinderVisualPath + "/geometry";
  const std::string cylinderCollisionPath = cylinderLinkPath + "/collision";
  const std::string cylinderCollisionGeometryPath =
    cylinderCollisionPath + "/geometry";
  const auto cylinderCollisionGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(cylinderCollisionGeometryPath));
  ASSERT_TRUE(cylinderCollisionGeometry);
  EXPECT_TRUE(cylinderCollisionGeometry.HasAPI<pxr::UsdPhysicsCollisionAPI>());
  const auto cylinderGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(cylinderGeometryPath));
  ASSERT_TRUE(cylinderGeometry);
  const auto usdCylinder = pxr::UsdGeomCylinder(cylinderGeometry);
  ASSERT_TRUE(usdCylinder);
  usdCylinder.GetRadiusAttr().Get(&radius);
  usdCylinder.GetHeightAttr().Get(&height);
  EXPECT_DOUBLE_EQ(0.2, radius);
  EXPECT_DOUBLE_EQ(0.1, height);
  usdCylinder.GetExtentAttr().Get(&extent);
  const pxr::GfVec3f cylinderExtent(0.2, 0.2, 0.05);
  this->CheckExtent(extent, -1.0 * cylinderExtent, cylinderExtent);
  extent.clear();

  const std::string spherePath = worldPath + "/sphere";
  const std::string sphereLinkPath = spherePath + "/link";
  const std::string sphereVisualPath = sphereLinkPath + "/sphere_vis";
  const std::string sphereGeometryPath = sphereVisualPath + "/geometry";
  const std::string sphereCollisionPath = sphereLinkPath + "/collision";
  const std::string sphereCollisionGeometryPath =
    sphereCollisionPath + "/geometry";
  const auto sphereCollisionGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(sphereCollisionGeometryPath));
  ASSERT_TRUE(sphereCollisionGeometry);
  EXPECT_TRUE(sphereCollisionGeometry.HasAPI<pxr::UsdPhysicsCollisionAPI>());
  const auto sphereGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(sphereGeometryPath));
  ASSERT_TRUE(sphereGeometry);
  const auto usdSphere = pxr::UsdGeomSphere(sphereGeometry);
  ASSERT_TRUE(usdSphere);
  usdSphere.GetRadiusAttr().Get(&radius);
  EXPECT_DOUBLE_EQ(1.23, radius);
  usdSphere.GetExtentAttr().Get(&extent);
  this->CheckExtent(extent, pxr::GfVec3f(-1.23), pxr::GfVec3f(1.23));
  extent.clear();

  const std::string capsulePath = worldPath + "/capsule";
  const std::string capsuleLinkPath = capsulePath + "/link";
  const std::string capsuleVisualPath = capsuleLinkPath + "/visual";
  const std::string capsuleGeometryPath = capsuleVisualPath + "/geometry";
  const auto capsuleGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(capsuleGeometryPath));
  ASSERT_TRUE(capsuleGeometry);
  const auto usdCapsule = pxr::UsdGeomCapsule(capsuleGeometry);
  ASSERT_TRUE(usdCapsule);
  usdCapsule.GetRadiusAttr().Get(&radius);
  usdCapsule.GetHeightAttr().Get(&length);
  EXPECT_DOUBLE_EQ(0.2, radius);
  EXPECT_DOUBLE_EQ(0.1, length);
  usdCapsule.GetExtentAttr().Get(&extent);
  const pxr::GfVec3f capsuleExtent(0.2, 0.2, 0.25);
  this->CheckExtent(extent, -1.0 * capsuleExtent, capsuleExtent);
  extent.clear();

  const std::string meshPath = worldPath + "/mesh";
  const std::string meshLinkPath = meshPath + "/link";
  const std::string meshVisualPath = meshLinkPath + "/visual";
  const std::string meshGeometryPath = meshVisualPath + "/geometry";
  const std::string meshGeometryMeshPath = meshGeometryPath + "/Cube";
  const std::string capsuleCollisionPath = capsuleLinkPath + "/collision";
  const std::string capsuleCollisionGeometryPath =
    capsuleCollisionPath + "/geometry";
  const auto capsuleCollisionGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(capsuleCollisionGeometryPath));
  ASSERT_TRUE(capsuleCollisionGeometry);
  EXPECT_TRUE(capsuleCollisionGeometry.HasAPI<pxr::UsdPhysicsCollisionAPI>());
  const auto meshGeometryParentPrim = this->stage->GetPrimAtPath(
      pxr::SdfPath(meshGeometryPath));
  ASSERT_TRUE(meshGeometryParentPrim);
  const auto meshGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(meshGeometryMeshPath));
  ASSERT_TRUE(meshGeometry);
  scaleAttr = meshGeometry.GetAttribute(pxr::TfToken("xformOp:scale"));
  ASSERT_TRUE(scaleAttr);
  scaleAttr.Get(&scale);
  EXPECT_EQ(pxr::GfVec3f(1.2, 2.3, 3.4), scale);
  sdf::usd::testing::HasScaleXFormOp(meshGeometry);
  const auto usdMesh = pxr::UsdGeomMesh(meshGeometry);
  ASSERT_TRUE(usdMesh);
  usdMesh.GetExtentAttr().Get(&extent);
  // there's a bit of floating point round of error in the mesh, so the
  // target extent's minimum y coordinate is manually set to account for this
  this->CheckExtent(extent, pxr::GfVec3f(-1, -1.000001, -1), pxr::GfVec3f(1));
  extent.clear();

  pxr::VtArray<pxr::GfVec3f> normals;
  pxr::VtArray<pxr::GfVec3f> points;
  pxr::VtIntArray faceVertexIndices;
  pxr::VtIntArray faceVertexCounts;
  usdMesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
  usdMesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
  usdMesh.GetNormalsAttr().Get(&normals);
  usdMesh.GetPointsAttr().Get(&points);
  EXPECT_EQ(24u, normals.size());
  EXPECT_EQ(24u, points.size());
  EXPECT_EQ(36u, faceVertexIndices.size());
  EXPECT_EQ(12u, faceVertexCounts.size());

  EXPECT_EQ(pxr::TfToken("vertex"), usdMesh.GetNormalsInterpolation());
  pxr::TfToken subdivisionScheme;
  usdMesh.GetSubdivisionSchemeAttr().Get(&subdivisionScheme);
  EXPECT_EQ(pxr::TfToken("none"), subdivisionScheme);
}
