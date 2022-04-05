/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <string>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "test_config.h"
#include "test_utils.hh"

#include "USDLinks.hh"

/////////////////////////////////////////////////
TEST(USDLinksTest, LinksNameMassAndDiagonalMoments)
{
  const std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  const auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  sdf::usd::USDData usdData(filename);

  const auto boxLink = stage->GetPrimAtPath(pxr::SdfPath("/box/box_link"));
  EXPECT_TRUE(boxLink);

  auto checkLink =
    [](const sdf::Link &_link,
       const std::string &_name,
       double mass,
       const ignition::math::Vector3d &_diagonalMoment)
    {
      EXPECT_EQ(_name, _link.Name());
      EXPECT_NEAR(mass, _link.Inertial().MassMatrix().Mass(), 1e-4);
      EXPECT_EQ(
        _diagonalMoment,
        _link.Inertial().MassMatrix().PrincipalMoments());
    };

  sdf::Link linkSDF;
  sdf::usd::ParseUSDLinks(
    boxLink, "/box/box_link", &linkSDF, usdData);

  checkLink(linkSDF, "box_link", 1.0,
            ignition::math::Vector3d(0.1666, 0.1666, 0.1666));

  const auto cylinderLink = stage->GetPrimAtPath(
    pxr::SdfPath("/cylinder/cylinder_link"));
  EXPECT_TRUE(cylinderLink);

  sdf::Link linkCylinderSDF;
  sdf::usd::ParseUSDLinks(
    cylinderLink, "/cylinder/cylinder_link", &linkCylinderSDF, usdData);

  checkLink(linkCylinderSDF, "cylinder_link", 1.7,
            ignition::math::Vector3d(0.1458, 0.1458, 0.125));

  const auto sphereLink = stage->GetPrimAtPath(
    pxr::SdfPath("/sphere/sphere_link"));
  EXPECT_TRUE(sphereLink);

  sdf::Link linkSphereSDF;
  sdf::usd::ParseUSDLinks(
    sphereLink, "/sphere/sphere_link", &linkSphereSDF, usdData);

  checkLink(linkSphereSDF, "sphere_link", 2,
            ignition::math::Vector3d(0.1, 0.1, 0.1));

  const auto capsuleLink = stage->GetPrimAtPath(
    pxr::SdfPath("/capsule/capsule_link"));
  EXPECT_TRUE(capsuleLink);

  sdf::Link linkCapsuleSDF;
  sdf::usd::ParseUSDLinks(
    capsuleLink, "/capsule/capsule_link", &linkCapsuleSDF, usdData);

  checkLink(linkCapsuleSDF, "capsule_link", 1,
            ignition::math::Vector3d(0.074154, 0.074154, 0.018769));

  const auto ellipsoidLink = stage->GetPrimAtPath(
    pxr::SdfPath("/ellipsoid/ellipsoid_link"));
  EXPECT_TRUE(ellipsoidLink);

  sdf::Link linkEllipsoidSDF;
  sdf::usd::ParseUSDLinks(
    ellipsoidLink, "/ellipsoid/ellipsoid_link", &linkEllipsoidSDF, usdData);

  checkLink(linkEllipsoidSDF, "ellipsoid_link", 1,
            ignition::math::Vector3d(0.068, 0.058, 0.026));
}
