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
#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/Box.hh"
#include "sdf/Model.hh"
#include "sdf/Link.hh"
#include "sdf/Surface.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Types.hh"
#include "sdf/Root.hh"
#include "test_utils.hh"
#include <gz/math/Inertial.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMcollision, Construction)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());
  EXPECT_TRUE(collision.Name().empty());
  EXPECT_EQ(collision.Density(), 1000.0);
  EXPECT_EQ(collision.DensityDefault(), 1000.0);

  collision.SetName("test_collison");
  EXPECT_EQ(collision.Name(), "test_collison");

  EXPECT_EQ(gz::math::Pose3d::Zero, collision.RawPose());
  EXPECT_TRUE(collision.PoseRelativeTo().empty());
  {
    auto semanticPose = collision.SemanticPose();
    EXPECT_EQ(collision.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  EXPECT_EQ(collision.Density(), 1000.0);
  collision.SetDensity(1240.0);
  EXPECT_DOUBLE_EQ(collision.Density(), 1240.0);

  EXPECT_EQ(collision.AutoInertiaParams(), nullptr);
  sdf::ElementPtr autoInertiaParamsElem(new sdf::Element());
  autoInertiaParamsElem->SetName("auto_inertia_params");
  collision.SetAutoInertiaParams(autoInertiaParamsElem);
  EXPECT_EQ(collision.AutoInertiaParams(), autoInertiaParamsElem);
  EXPECT_EQ(collision.AutoInertiaParams()->GetName(),
    autoInertiaParamsElem->GetName());

  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision.RawPose());

  collision.SetPoseRelativeTo("link");
  EXPECT_EQ("link", collision.PoseRelativeTo());
  {
    auto semanticPose = collision.SemanticPose();
    EXPECT_EQ(collision.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  ASSERT_NE(nullptr, collision.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, collision.Geom()->Type());
  EXPECT_EQ(nullptr, collision.Geom()->BoxShape());
  EXPECT_EQ(nullptr, collision.Geom()->ConeShape());
  EXPECT_EQ(nullptr, collision.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, collision.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, collision.Geom()->SphereShape());

  ASSERT_NE(nullptr, collision.Surface());
  ASSERT_NE(nullptr, collision.Surface()->Contact());
}

/////////////////////////////////////////////////
TEST(DOMCollision, MoveConstructor)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2(std::move(collision));
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyConstructor)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2(collision);
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, MoveAssignment)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2;
  collision2 = std::move(collision);
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyAssignment)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2;
  collision2 = collision;
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyAssignmentAfterMove)
{
  sdf::Collision collision1;
  collision1.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2;
  collision2.SetRawPose({-20, -30, -40, GZ_PI, GZ_PI, GZ_PI});

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Collision tmp = std::move(collision1);
  collision1 = collision2;
  collision2 = tmp;

  EXPECT_EQ(gz::math::Pose3d(-20, -30, -40, GZ_PI, GZ_PI, GZ_PI),
            collision1.RawPose());
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMcollision, SetGeometry)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());
  EXPECT_TRUE(collision.Name().empty());

  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);

  collision.SetGeom(geometry);

  ASSERT_NE(nullptr, collision.Geom());
  EXPECT_EQ(sdf::GeometryType::BOX, collision.Geom()->Type());
}

/////////////////////////////////////////////////
TEST(DOMcollision, SetSurface)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());

  sdf::Surface surface;
  ASSERT_NE(nullptr, surface.Contact());
  sdf::Contact contact;
  contact.SetCollideBitmask(0x2);
  surface.SetContact(contact);

  collision.SetSurface(surface);

  ASSERT_NE(nullptr, collision.Surface());
  ASSERT_NE(nullptr, collision.Surface()->Contact());
  EXPECT_EQ(collision.Surface()->Contact()->CollideBitmask(), 0x2);
}

/////////////////////////////////////////////////
TEST(DOMCollision, IncorrectBoxCollisionCalculateInertial)
{
  sdf::Collision collision;
  EXPECT_DOUBLE_EQ(1000.0, collision.Density());

  sdf::ElementPtr sdf(new sdf::Element());
  collision.Load(sdf);

  gz::math::Inertiald collisionInertial;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Geometry geom;
  sdf::Box box;

  // Invalid Inertial test
  box.SetSize(gz::math::Vector3d(-1, 1, 0));
  geom.SetType(sdf::GeometryType::BOX);
  geom.SetBoxShape(box);
  collision.SetGeom(geom);

  sdf::Errors errors;

  collision.CalculateInertial(errors, collisionInertial,
    sdfParserConfig);
  ASSERT_FALSE(errors.empty());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CorrectBoxCollisionCalculateInertial)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
  " <sdf version=\"1.11\">"
  "   <model name='shapes'>"
  "     <link name='link'>"
  "       <inertial auto='true' />"
  "       <collision name='box_col'>"
  "         <density>1240.0</density>"
  "         <geometry>"
  "           <box>"
  "             <size>2 2 2</size>"
  "           </box>"
  "         </geometry>"
  "       </collision>"
  "     </link>"
  "   </model>"
  " </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  const sdf::Collision *collision = link->CollisionByIndex(0);

  sdf::Errors inertialErr;
  root.ResolveAutoInertials(inertialErr, sdfParserConfig);

  const double l = 2;
  const double w = 2;
  const double h = 2;

  double expectedMass = l*w*h * collision->Density();
  double ixx = (1.0/12.0) * expectedMass * (w*w + h*h);
  double iyy = (1.0/12.0) * expectedMass * (l*l + h*h);
  double izz = (1.0/12.0) * expectedMass * (l*l + w*w);

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixx, iyy, izz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  ASSERT_TRUE(inertialErr.empty());
  EXPECT_DOUBLE_EQ(1240.0, collision->Density());
  EXPECT_DOUBLE_EQ(expectedMass, link->Inertial().MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.MassMatrix(), link->Inertial().MassMatrix());
  EXPECT_EQ(expectedInertial.Pose(), link->Inertial().Pose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CalculateInertialWithAutoInertiaParamsElement)
{
  // This example considers a custom voxel-based inertia calculator
  // <auto_inertial_params> element is used to provide properties
  // for the custom calculator like voxel size, grid type, etc.
  std::string sdf = "<?xml version=\"1.0\"?>"
  " <sdf version=\"1.11\">"
  "   <model name='shapes'>"
  "     <link name='link'>"
  "       <inertial auto='true' />"
  "       <collision name='box_col'>"
  "         <auto_inertia_params>"
  "           <gz:voxel_size>0.01</gz:voxel_size>"
  "           <gz:voxel_grid_type>float</gz:voxel_grid_type>"
  "         </auto_inertia_params>"
  "         <density>1240.0</density>"
  "         <geometry>"
  "           <box>"
  "             <size>2 2 2</size>"
  "           </box>"
  "         </geometry>"
  "       </collision>"
  "     </link>"
  "   </model>"
  " </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  const sdf::Collision *collision = link->CollisionByIndex(0);

  root.ResolveAutoInertials(errors, sdfParserConfig);
  EXPECT_TRUE(errors.empty());

  sdf::ElementPtr autoInertiaParamsElem = collision->AutoInertiaParams();

  // <auto_inertial_params> element is used as parent element for custom
  // intertia calculator params. Custom elements have to be defined with a
  // namespace prefix(gz in this case). More about this can be found in the
  // following proposal:
  // http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal&cat=pose_semantics_docs&
  double voxelSize = autoInertiaParamsElem->Get<double>("gz:voxel_size");
  std::string voxelGridType =
    autoInertiaParamsElem->Get<std::string>("gz:voxel_grid_type");
  EXPECT_EQ("float", voxelGridType);
  EXPECT_DOUBLE_EQ(0.01, voxelSize);
}

/////////////////////////////////////////////////
TEST(DOMCollision, CalculateInertialPoseNotRelativeToLink)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
  " <sdf version=\"1.11\">"
  "   <model name='shapes'>"
  "     <frame name='arbitrary_frame'>"
  "       <pose>0 0 1 0 0 0</pose>"
  "     </frame>"
  "     <link name='link'>"
  "       <inertial auto='true' />"
  "       <collision name='box_col'>"
  "         <pose relative_to='arbitrary_frame'>0 0 -1 0 0 0</pose>"
  "         <density>1240.0</density>"
  "         <geometry>"
  "           <box>"
  "             <size>2 2 2</size>"
  "           </box>"
  "         </geometry>"
  "       </collision>"
  "     </link>"
  "   </model>"
  " </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  const sdf::Collision *collision = link->CollisionByIndex(0);

  sdf::Errors inertialErr;
  root.ResolveAutoInertials(inertialErr, sdfParserConfig);

  const double l = 2;
  const double w = 2;
  const double h = 2;

  double expectedMass = l*w*h * collision->Density();
  double ixx = (1.0/12.0) * expectedMass * (w*w + h*h);
  double iyy = (1.0/12.0) * expectedMass * (l*l + h*h);
  double izz = (1.0/12.0) * expectedMass * (l*l + w*w);

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixx, iyy, izz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  ASSERT_TRUE(inertialErr.empty());
  EXPECT_DOUBLE_EQ(1240.0, collision->Density());
  EXPECT_DOUBLE_EQ(expectedMass, link->Inertial().MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.MassMatrix(), link->Inertial().MassMatrix());
  EXPECT_EQ(expectedInertial.Pose(), link->Inertial().Pose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CollisionCalculateInertialFailurePolicy)
{
  sdf::Collision collision;

  sdf::ElementPtr sdf(new sdf::Element());
  collision.Load(sdf);

  const sdf::ParserConfig sdfParserConfig;
  sdf::Geometry geom;
  sdf::Mesh mesh;
  geom.SetType(sdf::GeometryType::MESH);
  geom.SetMeshShape(mesh);
  collision.SetGeom(geom);

  sdf::ParserConfig config;
  sdf::Errors errors;
  sdf::CustomInertiaCalcProperties inertiaCalcProps;

  // Custom inertia calculator that returns null inertial
  auto customMeshInertiaCalculator = [](
    sdf::Errors &,
    const sdf::CustomInertiaCalcProperties &)
      -> std::optional<gz::math::Inertiald>
  {
    return std::nullopt;
  };
  config.RegisterCustomInertiaCalc(customMeshInertiaCalculator);

  // With default inertial failure policy, there should be an error when the
  // mesh inertial calculator returns null inertial values.
  gz::math::Inertiald collisionInertial;
  collision.CalculateInertial(errors, collisionInertial, config);
  ASSERT_EQ(1u, errors.size()) << errors;
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::LINK_INERTIA_INVALID);
  const gz::math::Inertiald empty;
  EXPECT_EQ(empty, collisionInertial);

  // Set inertial failure policy to use default inertial values on failure and
  // verify that there are no more errors.
  errors.clear();
  config.SetCalculateInertialFailurePolicy(
    sdf::CalculateInertialFailurePolicyType::WARN_AND_USE_DEFAULT_INERTIAL);
  collision.CalculateInertial(errors, collisionInertial, config);
  EXPECT_TRUE(errors.empty()) << errors;

  // Verify default inertial values are returned.
  gz::math::Inertiald defaultInertial;
  defaultInertial.SetMassMatrix(
    gz::math::MassMatrix3d(1.0,
      gz::math::Vector3d::One,
      gz::math::Vector3d::Zero));
  EXPECT_EQ(collisionInertial, defaultInertial);
}

/////////////////////////////////////////////////
TEST(DOMCollision, ToElement)
{
  sdf::Collision collision;

  collision.SetName("my-collision");

  sdf::Geometry geom;
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  sdf::Surface surface;
  sdf::Contact contact;
  contact.SetCollideBitmask(123u);
  surface.SetContact(contact);
  sdf::Friction friction;
  sdf::ODE ode;
  ode.SetMu(1.23);
  friction.SetODE(ode);
  surface.SetFriction(friction);
  collision.SetSurface(surface);

  sdf::ElementPtr elem = collision.ToElement();
  ASSERT_NE(nullptr, elem);
  // Expect no density element
  EXPECT_FALSE(elem->HasElement("density"));

  sdf::Collision collision2;
  collision2.Load(elem);
  const sdf::Surface *surface2 = collision2.Surface();

  EXPECT_EQ(collision.Name(), collision2.Name());
  EXPECT_EQ(collision.RawPose(), collision2.RawPose());
  EXPECT_NE(nullptr, collision2.Geom());
  ASSERT_NE(nullptr, surface2);
  ASSERT_NE(nullptr, surface2->Contact());
  EXPECT_EQ(123u, surface2->Contact()->CollideBitmask());
  ASSERT_NE(nullptr, surface2->Friction());
  ASSERT_NE(nullptr, surface2->Friction()->ODE());
  EXPECT_DOUBLE_EQ(1.23, surface2->Friction()->ODE()->Mu());

  // Now set density in collision
  const double kDensity = 1234.5;
  collision.SetDensity(kDensity);
  sdf::ElementPtr elemWithDensity = collision.ToElement();
  ASSERT_NE(nullptr, elemWithDensity);
  // Expect density element
  ASSERT_TRUE(elemWithDensity->HasElement("density"));
  EXPECT_DOUBLE_EQ(kDensity, elemWithDensity->Get<double>("density"));

  sdf::Collision collision3;
  collision3.Load(elem);
  EXPECT_DOUBLE_EQ(kDensity, collision.Density());
}

/////////////////////////////////////////////////
TEST(DOMCollision, ToElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  sdf::Collision collision;
  sdf::Errors errors;

  collision.SetName("my-collision");

  sdf::Geometry geom;
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  sdf::Surface surface;
  sdf::Contact contact;
  contact.SetCollideBitmask(123u);
  surface.SetContact(contact);
  sdf::Friction friction;
  sdf::ODE ode;
  ode.SetMu(1.23);
  friction.SetODE(ode);
  surface.SetFriction(friction);
  collision.SetSurface(surface);

  sdf::ElementPtr elem = collision.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Collision collision2;
  errors = collision2.Load(elem);
  EXPECT_TRUE(errors.empty());
  const sdf::Surface *surface2 = collision2.Surface();

  EXPECT_EQ(collision.Name(), collision2.Name());
  EXPECT_EQ(collision.RawPose(), collision2.RawPose());
  EXPECT_NE(nullptr, collision2.Geom());
  ASSERT_NE(nullptr, surface2);
  ASSERT_NE(nullptr, surface2->Contact());
  EXPECT_EQ(123u, surface2->Contact()->CollideBitmask());
  ASSERT_NE(nullptr, surface2->Friction());
  ASSERT_NE(nullptr, surface2->Friction()->ODE());
  EXPECT_DOUBLE_EQ(1.23, surface2->Friction()->ODE()->Mu());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
