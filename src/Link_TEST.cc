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
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include "sdf/Collision.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Root.hh"
#include "sdf/Model.hh"
#include "sdf/ParticleEmitter.hh"
#include "sdf/Projector.hh"
#include "sdf/Sensor.hh"
#include "sdf/Visual.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Types.hh"

/////////////////////////////////////////////////
TEST(DOMLink, Construction)
{
  sdf::Link link;
  EXPECT_EQ(nullptr, link.Element());
  EXPECT_TRUE(link.Name().empty());

  link.SetName("test_link");
  EXPECT_EQ("test_link", link.Name());

  EXPECT_EQ(0u, link.VisualCount());
  EXPECT_EQ(nullptr, link.VisualByIndex(0));
  EXPECT_EQ(nullptr, link.VisualByIndex(1));
  EXPECT_FALSE(link.VisualNameExists(""));
  EXPECT_FALSE(link.VisualNameExists("default"));

  EXPECT_EQ(0u, link.LightCount());
  EXPECT_EQ(nullptr, link.LightByIndex(0));
  EXPECT_EQ(nullptr, link.LightByIndex(1));
  EXPECT_FALSE(link.LightNameExists(""));
  EXPECT_FALSE(link.LightNameExists("default"));
  EXPECT_EQ(nullptr, link.LightByName("no_such_light"));

  EXPECT_EQ(0u, link.ParticleEmitterCount());
  EXPECT_EQ(nullptr, link.ParticleEmitterByIndex(0));
  EXPECT_EQ(nullptr, link.ParticleEmitterByIndex(1));
  EXPECT_FALSE(link.ParticleEmitterNameExists(""));
  EXPECT_FALSE(link.ParticleEmitterNameExists("default"));
  EXPECT_EQ(nullptr, link.ParticleEmitterByName("no_such_emitter"));

  EXPECT_EQ(0u, link.ProjectorCount());
  EXPECT_EQ(nullptr, link.ProjectorByIndex(0));
  EXPECT_EQ(nullptr, link.ProjectorByIndex(1));
  EXPECT_FALSE(link.ProjectorNameExists(""));
  EXPECT_FALSE(link.ProjectorNameExists("default"));
  EXPECT_EQ(nullptr, link.ProjectorByName("no_such_projector"));

  EXPECT_FALSE(link.EnableWind());
  link.SetEnableWind(true);
  EXPECT_TRUE(link.EnableWind());

  EXPECT_TRUE(link.EnableGravity());
  link.SetEnableGravity(false);
  EXPECT_FALSE(link.EnableGravity());

  EXPECT_FALSE(link.Kinematic());
  link.SetKinematic(true);
  EXPECT_TRUE(link.Kinematic());

  EXPECT_FALSE(link.AutoInertiaSaved());
  link.SetAutoInertiaSaved(true);
  EXPECT_TRUE(link.AutoInertiaSaved());

  EXPECT_FALSE(link.AutoInertia());
  link.SetAutoInertia(true);
  EXPECT_TRUE(link.AutoInertia());

  EXPECT_EQ(0u, link.SensorCount());
  EXPECT_EQ(nullptr, link.SensorByIndex(0));
  EXPECT_EQ(nullptr, link.SensorByIndex(1));
  EXPECT_EQ(nullptr, link.SensorByName("empty"));
  EXPECT_FALSE(link.SensorNameExists(""));
  EXPECT_FALSE(link.SensorNameExists("default"));

  EXPECT_EQ(gz::math::Pose3d::Zero, link.RawPose());
  EXPECT_TRUE(link.PoseRelativeTo().empty());
  {
    auto semanticPose = link.SemanticPose();
    EXPECT_EQ(gz::math::Pose3d::Zero, semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  link.SetRawPose({10, 20, 30, 0, GZ_PI, 0});
  EXPECT_EQ(gz::math::Pose3d(10, 20, 30, 0, GZ_PI, 0), link.RawPose());

  link.SetPoseRelativeTo("model");
  EXPECT_EQ("model", link.PoseRelativeTo());
  {
    auto semanticPose = link.SemanticPose();
    EXPECT_EQ(link.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("model", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  // Get the default inertial
  const gz::math::Inertiald inertial = link.Inertial();
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_FALSE(inertial.FluidAddedMass().has_value());
  EXPECT_EQ(inertial.BodyMatrix(), inertial.SpatialMatrix());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  EXPECT_FALSE(link.Density().has_value());
  const double density = 123.0;
  link.SetDensity(density);
  ASSERT_TRUE(link.Density().has_value());
  EXPECT_DOUBLE_EQ(density, *link.Density());

  EXPECT_EQ(link.AutoInertiaParams(), nullptr);
  sdf::ElementPtr autoInertiaParamsElem(new sdf::Element());
  autoInertiaParamsElem->SetName("auto_inertia_params");
  link.SetAutoInertiaParams(autoInertiaParamsElem);
  EXPECT_EQ(link.AutoInertiaParams(), autoInertiaParamsElem);
  EXPECT_EQ(link.AutoInertiaParams()->GetName(),
    autoInertiaParamsElem->GetName());

  EXPECT_EQ(0u, link.CollisionCount());
  EXPECT_EQ(nullptr, link.CollisionByIndex(0));
  EXPECT_EQ(nullptr, link.CollisionByIndex(1));
  EXPECT_FALSE(link.CollisionNameExists(""));
  EXPECT_FALSE(link.CollisionNameExists("default"));

  gz::math::Inertiald inertial2 {
    {2.3,
      gz::math::Vector3d(1.4, 2.3, 3.2),
      gz::math::Vector3d(0.1, 0.2, 0.3)},
      gz::math::Pose3d(1, 2, 3, 0, 0, 0),
    {1, 2, 3, 4, 5, 6,
     2, 7, 8, 9, 10, 11,
     3, 8, 12, 13, 14, 15,
     4, 9, 13, 16, 17, 18,
     5, 10, 14, 17, 19, 20,
     6, 11, 15, 18, 20, 21}};

  EXPECT_TRUE(link.SetInertial(inertial2));

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.4, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.2, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
  EXPECT_TRUE(link.Inertial().MassMatrix().IsValid());

  ASSERT_TRUE(link.Inertial().FluidAddedMass().has_value());
  EXPECT_DOUBLE_EQ(1.0, link.Inertial().FluidAddedMass().value()(0, 0));
  EXPECT_DOUBLE_EQ(2.0, link.Inertial().FluidAddedMass().value()(0, 1));
  EXPECT_DOUBLE_EQ(3.0, link.Inertial().FluidAddedMass().value()(0, 2));
  EXPECT_DOUBLE_EQ(4.0, link.Inertial().FluidAddedMass().value()(0, 3));
  EXPECT_DOUBLE_EQ(5.0, link.Inertial().FluidAddedMass().value()(0, 4));
  EXPECT_DOUBLE_EQ(6.0, link.Inertial().FluidAddedMass().value()(0, 5));
  EXPECT_DOUBLE_EQ(2.0, link.Inertial().FluidAddedMass().value()(1, 0));
  EXPECT_DOUBLE_EQ(7.0, link.Inertial().FluidAddedMass().value()(1, 1));
  EXPECT_DOUBLE_EQ(8.0, link.Inertial().FluidAddedMass().value()(1, 2));
  EXPECT_DOUBLE_EQ(9.0, link.Inertial().FluidAddedMass().value()(1, 3));
  EXPECT_DOUBLE_EQ(10.0, link.Inertial().FluidAddedMass().value()(1, 4));
  EXPECT_DOUBLE_EQ(11.0, link.Inertial().FluidAddedMass().value()(1, 5));
  EXPECT_DOUBLE_EQ(3.0, link.Inertial().FluidAddedMass().value()(2, 0));
  EXPECT_DOUBLE_EQ(8.0, link.Inertial().FluidAddedMass().value()(2, 1));
  EXPECT_DOUBLE_EQ(12.0, link.Inertial().FluidAddedMass().value()(2, 2));
  EXPECT_DOUBLE_EQ(13.0, link.Inertial().FluidAddedMass().value()(2, 3));
  EXPECT_DOUBLE_EQ(14.0, link.Inertial().FluidAddedMass().value()(2, 4));
  EXPECT_DOUBLE_EQ(15.0, link.Inertial().FluidAddedMass().value()(2, 5));
  EXPECT_DOUBLE_EQ(4.0, link.Inertial().FluidAddedMass().value()(3, 0));
  EXPECT_DOUBLE_EQ(9.0, link.Inertial().FluidAddedMass().value()(3, 1));
  EXPECT_DOUBLE_EQ(13.0, link.Inertial().FluidAddedMass().value()(3, 2));
  EXPECT_DOUBLE_EQ(16.0, link.Inertial().FluidAddedMass().value()(3, 3));
  EXPECT_DOUBLE_EQ(17.0, link.Inertial().FluidAddedMass().value()(3, 4));
  EXPECT_DOUBLE_EQ(18.0, link.Inertial().FluidAddedMass().value()(3, 5));
  EXPECT_DOUBLE_EQ(5.0, link.Inertial().FluidAddedMass().value()(4, 0));
  EXPECT_DOUBLE_EQ(10.0, link.Inertial().FluidAddedMass().value()(4, 1));
  EXPECT_DOUBLE_EQ(14.0, link.Inertial().FluidAddedMass().value()(4, 2));
  EXPECT_DOUBLE_EQ(17.0, link.Inertial().FluidAddedMass().value()(4, 3));
  EXPECT_DOUBLE_EQ(19.0, link.Inertial().FluidAddedMass().value()(4, 4));
  EXPECT_DOUBLE_EQ(20.0, link.Inertial().FluidAddedMass().value()(4, 5));
  EXPECT_DOUBLE_EQ(6.0, link.Inertial().FluidAddedMass().value()(5, 0));
  EXPECT_DOUBLE_EQ(11.0, link.Inertial().FluidAddedMass().value()(5, 1));
  EXPECT_DOUBLE_EQ(15.0, link.Inertial().FluidAddedMass().value()(5, 2));
  EXPECT_DOUBLE_EQ(18.0, link.Inertial().FluidAddedMass().value()(5, 3));
  EXPECT_DOUBLE_EQ(20.0, link.Inertial().FluidAddedMass().value()(5, 4));
  EXPECT_DOUBLE_EQ(21.0, link.Inertial().FluidAddedMass().value()(5, 5));
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyConstructor)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyAssignmentOperator)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2;
  link2 = link;
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MoveConstructor)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MoveAssignmentOperator)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2;
  link2 = std::move(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyAssignmentAfterMove)
{
  sdf::Link link1;
  link1.SetName("link1");

  sdf::Link link2;
  link2.SetName("link2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Link tmp = std::move(link1);
  link1 = link2;
  link2 = tmp;

  EXPECT_EQ("link2", link1.Name());
  EXPECT_EQ("link1", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, InvalidInertia)
{
  sdf::Link link;
  EXPECT_EQ(nullptr, link.Element());
  EXPECT_TRUE(link.Name().empty());

  gz::math::Inertiald invalidInertial {
    {2.3, gz::math::Vector3d(0.1, 0.2, 0.3),
      gz::math::Vector3d(1.2, 2.3, 3.4)},
      gz::math::Pose3d(1, 2, 3, 0, 0, 0)};

  EXPECT_FALSE(link.SetInertial(invalidInertial));

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(1.2, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.4, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
  EXPECT_FALSE(link.Inertial().MassMatrix().IsValid());
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsWithNoCollisionsInLink)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
  " <sdf version=\"1.11\">"
  "   <model name='shapes'>"
  "     <link name='link'>"
  "       <inertial auto='true' />"
  "     </link>"
  "   </model>"
  " </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  ASSERT_EQ(1u, errors.size()) << errors;
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos,
            errors[0].Message().find(
                "Inertial is set to auto but there are no <collision> elements "
                "for the link named link."));
  EXPECT_NE(nullptr, root.Element());
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsWithDifferentDensity)
{
  const std::string sdfString = R"(
  <?xml version="1.0"?>
  <sdf version="1.11">
    <model name='multilink_model'>
      <pose>0 0 1.0 0 0 0</pose>
      <link name='link_density'>
        <pose>0 1.0 0 0 0 0</pose>
        <inertial auto='true'>
          <density>12.0</density>
        </inertial>
        <collision name='box_collision'>
          <pose>1.0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
      <link name='collision_density'>
        <pose>0 2.0 0 0 0 0</pose>
        <inertial auto='true'/>
        <collision name='box_collision'>
          <pose>1.0 0 0 0 0 0</pose>
          <density>24.0</density>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
      <link name='collision_density_overrides_link_density'>
        <pose>0 3.0 0 0 0 0</pose>
        <inertial auto='true'>
          <density>12.0</density>
        </inertial>
        <collision name='box_collision'>
          <pose>1.0 0 0 0 0 0</pose>
          <density>24.0</density>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
      <link name='default_density'>
        <pose>0 4.0 0 0 0 0</pose>
        <inertial auto='true'/>
        <collision name='box_collision'>
          <pose>1.0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdfString, sdfParserConfig);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(4u, model->LinkCount());

  // ResolveAutoInertials should run by default during Root::Load.

  {
    const sdf::Link *link = model->LinkByName("link_density");
    ASSERT_NE(nullptr, link);
    auto linkDensity = link->Density();
    ASSERT_TRUE(linkDensity.has_value());
    EXPECT_DOUBLE_EQ(12.0, *linkDensity);
    // Expected inertial
    auto inertial = link->Inertial();
    EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0), inertial.Pose());
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(12.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(2, 2, 2),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
  }

  {
    const sdf::Link *link = model->LinkByName("collision_density");
    ASSERT_NE(nullptr, link);
    auto linkDensity = link->Density();
    EXPECT_FALSE(linkDensity.has_value());
    // assume Collision density is properly set to 24
    // Expected inertial
    auto inertial = link->Inertial();
    EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0), inertial.Pose());
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(24.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(4, 4, 4),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
  }

  {
    const sdf::Link *link =
      model->LinkByName("collision_density_overrides_link_density");
    ASSERT_NE(nullptr, link);
    auto linkDensity = link->Density();
    ASSERT_TRUE(linkDensity.has_value());
    EXPECT_DOUBLE_EQ(12.0, *linkDensity);
    // assume Collision density is properly set to 24 and overrides the link
    // density
    // Expected inertial
    auto inertial = link->Inertial();
    EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0), inertial.Pose());
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(24.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(4, 4, 4),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
  }

  {
    const sdf::Link *link =
      model->LinkByName("default_density");
    ASSERT_NE(nullptr, link);
    auto linkDensity = link->Density();
    EXPECT_FALSE(linkDensity.has_value());
    // assume density is the default value of 1000.0
    // Expected inertial
    auto inertial = link->Inertial();
    EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0), inertial.Pose());
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(1000.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d::One * 500.0 / 3.0,
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
  }
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsWithDifferentAutoInertiaParams)
{
  const std::string sdfString = R"(
  <?xml version="1.0"?>
  <sdf version="1.11">
    <model name='multilink_model'>
      <pose>0 0 1.0 0 0 0</pose>
      <link name='link_auto_inertia_params'>
        <pose>0 1.0 0 0 0 0</pose>
        <inertial auto='true'>
          <auto_inertia_params>
            <gz:density>12</gz:density>
            <gz:box_size>1 1 1</gz:box_size>
          </auto_inertia_params>
        </inertial>
        <collision name='box_collision'>
          <pose>1.0 0 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>uri</uri>
            </mesh>
          </geometry>
        </collision>
      </link>
      <link name='collision_auto_inertia_params'>
        <pose>0 2.0 0 0 0 0</pose>
        <inertial auto='true'/>
        <collision name='box_collision'>
          <pose>2.0 0 0 0 0 0</pose>
          <auto_inertia_params>
            <gz:density>24</gz:density>
            <gz:box_size>1 1 1</gz:box_size>
          </auto_inertia_params>
          <geometry>
            <mesh>
              <uri>uri</uri>
            </mesh>
          </geometry>
        </collision>
      </link>
      <link name='collision_auto_inertia_params_overrides'>
        <pose>0 3.0 0 0 0 0</pose>
        <inertial auto='true'>
          <auto_inertia_params>
            <gz:density>12</gz:density>
            <gz:box_size>1 1 1</gz:box_size>
          </auto_inertia_params>
        </inertial>
        <collision name='box_collision'>
          <pose>3.0 0 0 0 0 0</pose>
          <auto_inertia_params>
            <gz:density>24</gz:density>
            <gz:box_size>1 1 1</gz:box_size>
          </auto_inertia_params>
          <geometry>
            <mesh>
              <uri>uri</uri>
            </mesh>
          </geometry>
        </collision>
      </link>
      <link name='default_auto_inertia_params'>
        <pose>0 4.0 0 0 0 0</pose>
        <inertial auto='true'/>
        <collision name='box_collision'>
          <pose>4.0 0 0 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>uri</uri>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  // Lambda function for registering as custom inertia calculator
  auto customMeshInertiaCalculator = [](
    sdf::Errors &,
    const sdf::CustomInertiaCalcProperties &_inertiaProps
  ) -> std::optional<gz::math::Inertiald>
  {
    auto autoInertiaParams = _inertiaProps.AutoInertiaParams();
    if (!autoInertiaParams ||
        !autoInertiaParams->HasElement("gz:density") ||
        !autoInertiaParams->HasElement("gz:box_size"))
    {
      // return default inertial values
      gz::math::Inertiald meshInertial;

      meshInertial.SetMassMatrix(
        gz::math::MassMatrix3d(
          1.0,
          gz::math::Vector3d::One,
          gz::math::Vector3d::Zero
        )
      );

      return meshInertial;
    }

    gz::math::Inertiald meshInerial;

    double gzDensity = autoInertiaParams->Get<double>("gz:density");
    gz::math::Vector3d gzBoxSize =
        autoInertiaParams->Get<gz::math::Vector3d>("gz:box_size");
    gz::math::Material material = gz::math::Material(gzDensity);

    gz::math::MassMatrix3d massMatrix;
    massMatrix.SetFromBox(gz::math::Material(gzDensity), gzBoxSize);

    meshInerial.SetMassMatrix(massMatrix);

    return meshInerial;
  };


  sdf::Root root;
  sdf::ParserConfig sdfParserConfig;
  sdfParserConfig.RegisterCustomInertiaCalc(customMeshInertiaCalculator);
  sdf::Errors errors = root.LoadSdfString(sdfString, sdfParserConfig);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(4u, model->LinkCount());

  // ResolveAutoInertials should run by default during Root::Load.

  {
    const sdf::Link *link = model->LinkByName("link_auto_inertia_params");
    ASSERT_NE(nullptr, link);

    // Verify inertial values computed from //inertial/auto_inertia_params
    // * gz:density == 12
    // * gz:box_size == (1 1 1)
    auto inertial = link->Inertial();
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(12.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(2, 2, 2),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
    EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0), inertial.Pose());
  }

  {
    const sdf::Link *link = model->LinkByName("collision_auto_inertia_params");
    ASSERT_NE(nullptr, link);

    // Verify inertial values computed from //collision/auto_inertia_params
    // * gz:density == 24
    // * gz:box_size == (1 1 1)
    auto inertial = link->Inertial();
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(24.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(4, 4, 4),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
    EXPECT_EQ(gz::math::Pose3d(2, 0, 0, 0, 0, 0), inertial.Pose());
  }

  {
    const sdf::Link *link =
      model->LinkByName("collision_auto_inertia_params_overrides");
    ASSERT_NE(nullptr, link);

    // Verify inertial values computed from //collision/auto_inertia_params
    // * gz:density == 24
    // * gz:box_size == (1 1 1)
    auto inertial = link->Inertial();
    // box is 1 m^3, so expected mass == density
    EXPECT_DOUBLE_EQ(24.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(4, 4, 4),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
    EXPECT_EQ(gz::math::Pose3d(3, 0, 0, 0, 0, 0), inertial.Pose());
  }

  {
    const sdf::Link *link =
      model->LinkByName("default_auto_inertia_params");
    ASSERT_NE(nullptr, link);

    // Verify default inertial values
    auto inertial = link->Inertial();
    EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().Mass());
    EXPECT_EQ(gz::math::Vector3d(1, 1, 1),
              inertial.MassMatrix().DiagonalMoments());
    EXPECT_EQ(gz::math::Vector3d::Zero,
              inertial.MassMatrix().OffDiagonalMoments());
    EXPECT_EQ(gz::math::Pose3d(4, 0, 0, 0, 0, 0), inertial.Pose());
  }
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsWithMultipleCollisions)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
  "<sdf version=\"1.11\">"
  "  <model name='compound_model'>"
  "   <pose>0 0 1.0 0 0 0</pose>"
  "   <link name='compound_link'>"
  "     <inertial auto='true' />"
  "     <collision name='box_collision'>"
  "      <pose>0 0 -0.5 0 0 0</pose>"
  "      <density>2.0</density>"
  "      <geometry>"
  "        <box>"
  "          <size>1 1 1</size>"
  "        </box>"
  "        </geometry>"
  "      </collision>"
  "      <collision name='cylinder_compound_collision'>"
  "        <pose>0 0 0.5 0 0 0</pose>"
  "        <density>4</density>"
  "        <geometry>"
  "          <cylinder>"
  "            <radius>0.5</radius>"
  "            <length>1.0</length>"
  "          </cylinder>"
  "        </geometry>"
  "      </collision>"
  "    </link>"
  "   </model>"
  "  </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  root.ResolveAutoInertials(errors, sdfParserConfig);
  EXPECT_TRUE(errors.empty());

  // Mass of cube(volume * density) + mass of cylinder(volume * density)
  double expectedMass = 1.0 * 2.0 + GZ_PI * 0.5 * 0.5 * 1 * 4.0;

  EXPECT_DOUBLE_EQ(expectedMass, link->Inertial().MassMatrix().Mass());
  EXPECT_EQ(gz::math::Vector3d(2.013513, 2.013513, 0.72603),
    link->Inertial().MassMatrix().DiagonalMoments());
}

/////////////////////////////////////////////////
TEST(DOMLink, InertialValuesGivenWithAutoSetToTrue)
{
  // A model with link inertial auto set to true.
  // The inertia matrix is specified but should be ignored.
  // <mass> is not speicifed so the inertial values should be computed
  // based on the collision density value.
  std::string sdf = "<?xml version=\"1.0\"?>"
  "<sdf version=\"1.11\">"
  "  <model name='compound_model'>"
  "   <link name='compound_link'>"
  "     <inertial auto='true'>"
  "       <pose>1 1 1 2 2 2</pose>"
  "       <inertia>"
  "         <ixx>1</ixx>"
  "         <iyy>1</iyy>"
  "         <izz>1</izz>"
  "       </inertia>"
  "     </inertial>"
  "     <collision name='box_collision'>"
  "      <density>2.0</density>"
  "      <geometry>"
  "        <box>"
  "          <size>1 1 1</size>"
  "        </box>"
  "        </geometry>"
  "      </collision>"
  "    </link>"
  "   </model>"
  "  </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  root.ResolveAutoInertials(errors, sdfParserConfig);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(2.0, link->Inertial().MassMatrix().Mass());
  EXPECT_EQ(gz::math::Pose3d::Zero, link->Inertial().Pose());
  EXPECT_EQ(gz::math::Vector3d(0.33333, 0.33333, 0.33333),
    link->Inertial().MassMatrix().DiagonalMoments());
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsWithMass)
{
  // A model with link inertial auto set to true.
  // The inertia matrix is specified but should be ignored.
  // <mass> is specified - the auto computed inertial values should
  // be scaled based on the desired mass.
  std::string sdf = "<?xml version=\"1.0\"?>"
  "<sdf version=\"1.11\">"
  "  <model name='compound_model'>"
  "   <link name='compound_link'>"
  "     <inertial auto='true'>"
  "       <mass>4.0</mass>"
  "       <pose>1 1 1 2 2 2</pose>"
  "       <inertia>"
  "         <ixx>1</ixx>"
  "         <iyy>1</iyy>"
  "         <izz>1</izz>"
  "       </inertia>"
  "     </inertial>"
  "     <collision name='box_collision'>"
  "      <density>2.0</density>"
  "      <geometry>"
  "        <box>"
  "          <size>1 1 1</size>"
  "        </box>"
  "        </geometry>"
  "      </collision>"
  "    </link>"
  "   </model>"
  "  </sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  root.ResolveAutoInertials(errors, sdfParserConfig);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(4.0, link->Inertial().MassMatrix().Mass());
  EXPECT_EQ(gz::math::Pose3d::Zero, link->Inertial().Pose());
  EXPECT_EQ(gz::math::Vector3d(0.66666, 0.66666, 0.66666),
    link->Inertial().MassMatrix().DiagonalMoments());
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsWithMassAndMultipleCollisions)
{
  // A model with link inertial auto set to true.
  // The inertia matrix is specified but should be ignored.
  // <mass> is specified - the auto computed inertial values should
  // be scaled based on the desired mass.
  // The model contains two collisions with different sizes and densities
  // and a lumped center of mass at the link origin.
  std::string sdf = "<?xml version=\"1.0\"?>"
  "<sdf version=\"1.11\">"
  "  <model name='compound_model'>"
  "   <link name='compound_link'>"
  "     <inertial auto='true'>"
  "       <mass>12.0</mass>"
  "       <pose>1 1 1 2 2 2</pose>"
  "       <inertia>"
  "         <ixx>1</ixx>"
  "         <iyy>1</iyy>"
  "         <izz>1</izz>"
  "       </inertia>"
  "     </inertial>"
  "     <collision name='cube_collision'>"
  "       <pose>0.0 0.0 0.5 0 0 0</pose>"
  "       <density>4.0</density>"
  "       <geometry>"
  "         <box>"
  "           <size>1 1 1</size>"
  "         </box>"
  "       </geometry>"
  "     </collision>"
  "     <collision name='box_collision'>"
  "       <pose>0.0 0.0 -1.0 0 0 0</pose>"
  "       <density>1.0</density>"
  "       <geometry>"
  "         <box>"
  "           <size>1 1 2</size>"
  "         </box>"
  "       </geometry>"
  "     </collision>"
  "    </link>"
  "  </model>"
  "</sdf>";

  sdf::Root root;
  const sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);
  root.ResolveAutoInertials(errors, sdfParserConfig);
  EXPECT_TRUE(errors.empty());

  // Expect mass value from //inertial/mass
  EXPECT_DOUBLE_EQ(12.0, link->Inertial().MassMatrix().Mass());

  // Inertial values based on density before scaling to match specified mass
  //
  // Collision geometries:
  //
  // --------- z = 1
  // |       |
  // |   c   | cube on top, side length 1.0, density 4.0
  // |       |
  // |-------| z = 0
  // |       |
  // |       |
  // |       |
  // |   c   | box on bottom, size 1x1x2, density 1.0
  // |       |
  // |       |
  // |       |
  // --------- z = -2
  //
  //   Top cube: volume = 1.0, mass = 4.0, center of mass z = 0.5
  // Bottom box: volume = 2.0, mass = 2.0, center of mass z = -1.0
  //
  // Total mass from density: 6.0
  // Lumped center of mass z = (4.0 * 0.5 + 2.0 * (-1.0)) / 6.0 = 0.0
  EXPECT_EQ(gz::math::Pose3d::Zero, link->Inertial().Pose());

  // Moment of inertia at center of each shape
  //   Top cube: Ixx = Iyy = Izz = 4.0 / 12 * (1*1 + 1*1) =  8/12 = 2/3
  // Bottom box:       Ixx = Iyy = 2.0 / 12 * (1*1 + 2*2) = 10/12 = 5/6
  //                         Izz = 2.0 / 12 * (1*1 + 1*1) =  4/12 = 1/3
  //
  // Lumped moment of inertia at lumped center of mass
  // Ixx = sum(Ixx_i + mass_i * center_of_mass_z_i^2) for i in [top, bottom]
  // Iyy = Ixx
  // Izz = Izz_top + Izz_bottom
  //
  // Ixx = Iyy = (2/3 + 4.0 * 0.5 * 0.5) + (5/6 + 2.0 * (-1.0) * (-1.0))
  //           = (2/3 + 1)               + (5/6 + 2.0)
  //           = 5/3                     + 17/6
  //           = 27/6 = 9/2 = 4.5
  //
  // Izz = 2/3 + 1/3 = 1.0

  // Then scale the inertias according to the specified mass of 12.0
  // mass_ratio = 12.0 / 6.0 = 2.0
  // Ixx = Iyy = mass_ratio * Ixx_unscaled = 2.0 * 4.5 = 9.0
  //       Izz = mass_ratio * Izz_unscaled = 2.0 * 1.0 = 2.0
  EXPECT_EQ(gz::math::Vector3d(9.0, 9.0, 2.0),
    link->Inertial().MassMatrix().DiagonalMoments());
}

/////////////////////////////////////////////////
TEST(DOMLink, ResolveAutoInertialsCalledWithAutoFalse)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
  " <sdf version=\"1.11\">"
  "   <model name='shapes'>"
  "     <link name='link'>"
  "       <inertial auto='false' />"
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
  root.ResolveAutoInertials(errors, sdfParserConfig);
  EXPECT_TRUE(errors.empty());

  // Default Inertial values set during load
  EXPECT_EQ(1.0, link->Inertial().MassMatrix().Mass());
  EXPECT_EQ(gz::math::Vector3d::One,
    link->Inertial().MassMatrix().DiagonalMoments());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddCollision)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.CollisionCount());

  sdf::Collision collision;
  collision.SetName("collision1");
  EXPECT_TRUE(link.AddCollision(collision));
  EXPECT_EQ(1u, link.CollisionCount());
  EXPECT_FALSE(link.AddCollision(collision));
  EXPECT_EQ(1u, link.CollisionCount());

  link.ClearCollisions();
  EXPECT_EQ(0u, link.CollisionCount());

  EXPECT_TRUE(link.AddCollision(collision));
  EXPECT_EQ(1u, link.CollisionCount());
  const sdf::Collision *collisionFromLink = link.CollisionByIndex(0);
  ASSERT_NE(nullptr, collisionFromLink);
  EXPECT_EQ(collisionFromLink->Name(), collision.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddVisual)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.VisualCount());

  sdf::Visual visual;
  visual.SetName("visual1");
  EXPECT_TRUE(link.AddVisual(visual));
  EXPECT_EQ(1u, link.VisualCount());
  EXPECT_FALSE(link.AddVisual(visual));
  EXPECT_EQ(1u, link.VisualCount());

  link.ClearVisuals();
  EXPECT_EQ(0u, link.VisualCount());

  EXPECT_TRUE(link.AddVisual(visual));
  EXPECT_EQ(1u, link.VisualCount());
  const sdf::Visual *visualFromLink = link.VisualByIndex(0);
  ASSERT_NE(nullptr, visualFromLink);
  EXPECT_EQ(visualFromLink->Name(), visual.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddLight)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.LightCount());

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(link.AddLight(light));
  EXPECT_EQ(1u, link.LightCount());
  EXPECT_FALSE(link.AddLight(light));
  EXPECT_EQ(1u, link.LightCount());

  link.ClearLights();
  EXPECT_EQ(0u, link.LightCount());

  EXPECT_TRUE(link.AddLight(light));
  EXPECT_EQ(1u, link.LightCount());
  const sdf::Light *lightFromLink = link.LightByIndex(0);
  ASSERT_NE(nullptr, lightFromLink);
  EXPECT_EQ(lightFromLink->Name(), light.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddSensor)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.SensorCount());

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(link.AddSensor(sensor));
  EXPECT_EQ(1u, link.SensorCount());
  EXPECT_FALSE(link.AddSensor(sensor));
  EXPECT_EQ(1u, link.SensorCount());

  link.ClearSensors();
  EXPECT_EQ(0u, link.SensorCount());

  EXPECT_TRUE(link.AddSensor(sensor));
  EXPECT_EQ(1u, link.SensorCount());
  const sdf::Sensor *sensorFromLink = link.SensorByIndex(0);
  ASSERT_NE(nullptr, sensorFromLink);
  EXPECT_EQ(sensorFromLink->Name(), sensor.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, ToElement)
{
  sdf::Link link;
  link.SetName("my-name");

  gz::math::Inertiald inertial {
    {2.3,
      gz::math::Vector3d(1.4, 2.3, 3.2),
      gz::math::Vector3d(0.1, 0.2, 0.3)},
      gz::math::Pose3d(1, 2, 3, 0, 0, 0),
    {1, 2, 3, 4, 5, 6,
     2, 7, 8, 9, 10, 11,
     3, 8, 12, 13, 14, 15,
     4, 9, 13, 16, 17, 18,
     5, 10, 14, 17, 19, 20,
     6, 11, 15, 18, 20, 21}};
  EXPECT_TRUE(link.SetInertial(inertial));
  link.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  link.SetEnableWind(true);
  link.SetKinematic(true);

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 1; ++i)
    {
      sdf::Collision collision;
      collision.SetName("collision" + std::to_string(i));
      EXPECT_TRUE(link.AddCollision(collision));
      EXPECT_FALSE(link.AddCollision(collision));
    }
    if (j == 0)
      link.ClearCollisions();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 2; i++)
    {
      sdf::Visual visual;
      visual.SetName("visual" + std::to_string(i));
      EXPECT_TRUE(link.AddVisual(visual));
      EXPECT_FALSE(link.AddVisual(visual));
    }
    if (j == 0)
     link.ClearVisuals();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; i++)
    {
      sdf::Light light;
      light.SetName("light" + std::to_string(i));
      EXPECT_TRUE(link.AddLight(light));
      EXPECT_FALSE(link.AddLight(light));
    }
    if (j == 0)
      link.ClearLights();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 4; i++)
    {
      sdf::Sensor sensor;
      sensor.SetName("sensor" + std::to_string(i));
      EXPECT_TRUE(link.AddSensor(sensor));
      EXPECT_FALSE(link.AddSensor(sensor));
    }
    if (j == 0)
      link.ClearSensors();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 5; i++)
    {
      sdf::ParticleEmitter emitter;
      emitter.SetName("emitter" + std::to_string(i));
      EXPECT_TRUE(link.AddParticleEmitter(emitter));
      EXPECT_FALSE(link.AddParticleEmitter(emitter));
    }
    if (j == 0)
      link.ClearParticleEmitters();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; i++)
    {
      sdf::Projector projector;
      projector.SetName("projector" + std::to_string(i));
      projector.SetTexture("projector.png");
      EXPECT_TRUE(link.AddProjector(projector));
      EXPECT_FALSE(link.AddProjector(projector));
    }
    if (j == 0)
      link.ClearProjectors();
  }

  sdf::ElementPtr elem = link.ToElement();
  ASSERT_NE(nullptr, elem);
  // Expect no density element
  {
    auto inertialElem = elem->FindElement("inertial");
    EXPECT_FALSE(inertialElem->HasElement("density"));
  }

  sdf::Link link2;
  link2.Load(elem);

  EXPECT_EQ(link.Name(), link2.Name());
  EXPECT_EQ(link.Inertial(), link2.Inertial());
  EXPECT_EQ(link.Density(), link2.Density());
  EXPECT_EQ(link.RawPose(), link2.RawPose());
  EXPECT_EQ(link.EnableWind(), link2.EnableWind());
  EXPECT_EQ(link.Kinematic(), link2.Kinematic());
  EXPECT_EQ(link.CollisionCount(), link2.CollisionCount());
  for (uint64_t i = 0; i < link2.CollisionCount(); ++i)
    EXPECT_NE(nullptr, link2.CollisionByIndex(i));

  EXPECT_EQ(link.VisualCount(), link2.VisualCount());
  for (uint64_t i = 0; i < link2.VisualCount(); ++i)
    EXPECT_NE(nullptr, link2.VisualByIndex(i));

  EXPECT_EQ(link.LightCount(), link2.LightCount());
  for (uint64_t i = 0; i < link2.LightCount(); ++i)
    EXPECT_NE(nullptr, link2.LightByIndex(i));

  EXPECT_EQ(link.SensorCount(), link2.SensorCount());
  for (uint64_t i = 0; i < link2.SensorCount(); ++i)
    EXPECT_NE(nullptr, link2.SensorByIndex(i));

  EXPECT_EQ(link.ParticleEmitterCount(), link2.ParticleEmitterCount());
  for (uint64_t i = 0; i < link2.ParticleEmitterCount(); ++i)
    EXPECT_NE(nullptr, link2.ParticleEmitterByIndex(i));

  EXPECT_EQ(link.ProjectorCount(), link2.ProjectorCount());
  for (uint64_t i = 0; i < link2.ProjectorCount(); ++i)
    EXPECT_NE(nullptr, link2.ProjectorByIndex(i));

  // Now set density in link
  const double kDensity = 1234.5;
  link.SetDensity(kDensity);
  sdf::ElementPtr elemWithDensity = link.ToElement();
  ASSERT_NE(nullptr, elemWithDensity);
  // Expect density element
  {
    auto inertialElem = elemWithDensity->FindElement("inertial");
    ASSERT_TRUE(inertialElem->HasElement("density"));
    EXPECT_DOUBLE_EQ(kDensity, inertialElem->Get<double>("density"));
  }

  sdf::Link link3;
  link3.Load(elemWithDensity);
  ASSERT_TRUE(link3.Density().has_value());
  EXPECT_DOUBLE_EQ(kDensity, *link3.Density());
}

/////////////////////////////////////////////////
TEST(DOMLink, MutableByIndex)
{
  sdf::Link link;
  link.SetName("my-name");

  sdf::Visual visual;
  visual.SetName("visual1");
  EXPECT_TRUE(link.AddVisual(visual));

  sdf::Collision collision;
  collision.SetName("collision1");
  EXPECT_TRUE(link.AddCollision(collision));

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(link.AddLight(light));

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(link.AddSensor(sensor));

  sdf::ParticleEmitter pe;
  pe.SetName("pe1");
  EXPECT_TRUE(link.AddParticleEmitter(pe));

  sdf::Projector projector;
  projector.SetName("projector1");
  EXPECT_TRUE(link.AddProjector(projector));

  // Modify the visual
  sdf::Visual *v = link.VisualByIndex(0);
  ASSERT_NE(nullptr, v);
  EXPECT_EQ("visual1", v->Name());
  v->SetName("visual2");
  EXPECT_EQ("visual2", link.VisualByIndex(0)->Name());

  // Modify the collision
  sdf::Collision *c = link.CollisionByIndex(0);
  ASSERT_NE(nullptr, c);
  EXPECT_EQ("collision1", c->Name());
  c->SetName("collision2");
  EXPECT_EQ("collision2", link.CollisionByIndex(0)->Name());

  // Modify the light
  sdf::Light *l = link.LightByIndex(0);
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("light1", l->Name());
  l->SetName("light2");
  EXPECT_EQ("light2", link.LightByIndex(0)->Name());

  // Modify the sensor
  sdf::Sensor *s = link.SensorByIndex(0);
  ASSERT_NE(nullptr, s);
  EXPECT_EQ("sensor1", s->Name());
  s->SetName("sensor2");
  EXPECT_EQ("sensor2", link.SensorByIndex(0)->Name());

  // Modify the particle emitter
  sdf::ParticleEmitter *p = link.ParticleEmitterByIndex(0);
  ASSERT_NE(nullptr, p);
  EXPECT_EQ("pe1", p->Name());
  p->SetName("pe2");
  EXPECT_EQ("pe2", link.ParticleEmitterByIndex(0)->Name());

  // Modify the projector
  sdf::Projector *pr = link.ProjectorByIndex(0);
  ASSERT_NE(nullptr, pr);
  EXPECT_EQ("projector1", pr->Name());
  pr->SetName("projector2");
  EXPECT_EQ("projector2", link.ProjectorByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MutableByName)
{
  sdf::Link link;
  link.SetName("my-name");

  sdf::Visual visual;
  visual.SetName("visual1");
  EXPECT_TRUE(link.AddVisual(visual));

  sdf::Collision collision;
  collision.SetName("collision1");
  EXPECT_TRUE(link.AddCollision(collision));

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(link.AddLight(light));

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(link.AddSensor(sensor));

  sdf::ParticleEmitter pe;
  pe.SetName("pe1");
  EXPECT_TRUE(link.AddParticleEmitter(pe));

  sdf::Projector projector;
  projector.SetName("projector1");
  EXPECT_TRUE(link.AddProjector(projector));

  // Modify the visual
  sdf::Visual *v = link.VisualByName("visual1");
  ASSERT_NE(nullptr, v);
  EXPECT_EQ("visual1", v->Name());
  v->SetName("visual2");
  EXPECT_FALSE(link.VisualNameExists("visual1"));
  EXPECT_TRUE(link.VisualNameExists("visual2"));

  // Modify the collision
  sdf::Collision *c = link.CollisionByName("collision1");
  ASSERT_NE(nullptr, c);
  EXPECT_EQ("collision1", c->Name());
  c->SetName("collision2");
  EXPECT_FALSE(link.CollisionNameExists("collision1"));
  EXPECT_TRUE(link.CollisionNameExists("collision2"));

  // Modify the light
  sdf::Light *l = link.LightByName("light1");
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("light1", l->Name());
  l->SetName("light2");
  EXPECT_FALSE(link.LightNameExists("light1"));
  EXPECT_TRUE(link.LightNameExists("light2"));

  // Modify the sensor
  sdf::Sensor *s = link.SensorByName("sensor1");
  ASSERT_NE(nullptr, s);
  EXPECT_EQ("sensor1", s->Name());
  s->SetName("sensor2");
  EXPECT_FALSE(link.SensorNameExists("sensor1"));
  EXPECT_TRUE(link.SensorNameExists("sensor2"));

  // Modify the particle emitter
  sdf::ParticleEmitter *p = link.ParticleEmitterByName("pe1");
  ASSERT_NE(nullptr, p);
  EXPECT_EQ("pe1", p->Name());
  p->SetName("pe2");
  EXPECT_FALSE(link.ParticleEmitterNameExists("pe1"));
  EXPECT_TRUE(link.ParticleEmitterNameExists("pe2"));

  // Modify the projector
  sdf::Projector *pr = link.ProjectorByName("projector1");
  ASSERT_NE(nullptr, pr);
  EXPECT_EQ("projector1", pr->Name());
  pr->SetName("projector2");
  EXPECT_FALSE(link.ProjectorNameExists("projector1"));
  EXPECT_TRUE(link.ProjectorNameExists("projector2"));
}
