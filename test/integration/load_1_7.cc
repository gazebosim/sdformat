#include <gtest/gtest.h>
#include "sdf/sdf.hh"

#include "test_config.h"

/////////////////////////////////////////////////
/// Test that a world with SDF version 1.7 can be loaded
TEST(SDF1_7, Load1_7World)
{
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="1_7_world_loaded">
  </world>
</sdf>)";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf->SetFromString(xmlString);

  ASSERT_NE(nullptr, sdf->Root());
  EXPECT_EQ(sdf->Root()->GetName(), "sdf");

  sdf::ElementPtr worldElem = sdf->Root()->GetElement("world");
  ASSERT_NE(nullptr, worldElem);
  EXPECT_EQ(worldElem->Get<std::string>("name"), "1_7_world_loaded");
}

/////////////////////////////////////////////////
/// Test that  <name> in <names_to_ignore> are retrievable
TEST(SDF1_7, RayNamesToIgnore)
{
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="testmodel">
    <link name="testlink">
      <sensor name="world_ray" type="ray">
        <ray>
          <names_to_ignore>
            <name>asdf</name>
          </names_to_ignore>
        </ray>
      </sensor>
    </link>
  </model>
</sdf>)";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf->SetFromString(xmlString);
  ASSERT_NE(nullptr, sdf->Root());

  sdf::ElementPtr modelElem = sdf->Root()->GetElement("model");
  ASSERT_NE(nullptr, modelElem);

  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);

  sdf::ElementPtr sensorElem = linkElem->GetElement("sensor");
  ASSERT_NE(nullptr, sensorElem);

  sdf::ElementPtr rayElem = sensorElem->GetElement("ray");
  ASSERT_NE(nullptr, rayElem);

  sdf::ElementPtr namesElem =rayElem->GetElement("names_to_ignore");
  ASSERT_NE(nullptr, namesElem);

  sdf::ElementPtr nameElem = namesElem->GetElement("name");
  EXPECT_EQ("asdf", nameElem->GetValue()->GetAsString());
}
