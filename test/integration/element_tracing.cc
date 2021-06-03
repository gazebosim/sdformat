/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Model.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Actor.hh"
#include "sdf/Light.hh"
#include "test_config.h"

//////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(ElementTracing, NestedModels)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "nested_model.sdf");

  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  const std::string xmlPath = "/sdf/model[@name=\"top_level_model\"]";

  sdf::ElementPtr modelElem = model->Element();
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ(testFile, modelElem->FilePath());
  ASSERT_TRUE(modelElem->LineNumber().has_value());
  EXPECT_EQ(2, modelElem->LineNumber().value());
  EXPECT_EQ(xmlPath, modelElem->XmlPath());
  EXPECT_EQ(2, static_cast<int>(model->LinkCount()));
  EXPECT_EQ(1, static_cast<int>(model->JointCount()));
  EXPECT_EQ(1, static_cast<int>(model->ModelCount()));

  const std::string parentLinkXmlPath = xmlPath + "/link[@name=\"parent\"]";

  const sdf::Link *parentLink = model->LinkByName("parent");
  ASSERT_NE(nullptr, parentLink);
  sdf::ElementPtr parentLinkElem = parentLink->Element();
  ASSERT_NE(nullptr, parentLinkElem);
  EXPECT_EQ(testFile, parentLinkElem->FilePath());
  ASSERT_TRUE(parentLinkElem->LineNumber().has_value());
  EXPECT_EQ(3, parentLinkElem->LineNumber().value());
  EXPECT_EQ(parentLinkXmlPath, parentLinkElem->XmlPath());

  const std::string childLinkXmlPath = xmlPath + "/link[@name=\"child\"]";

  const sdf::Link *childLink = model->LinkByName("child");
  ASSERT_NE(nullptr, childLink);
  sdf::ElementPtr childLinkElem = childLink->Element();
  ASSERT_NE(nullptr, childLinkElem);
  EXPECT_EQ(testFile, childLinkElem->FilePath());
  ASSERT_TRUE(childLinkElem->LineNumber().has_value());
  EXPECT_EQ(4, childLinkElem->LineNumber().value());
  EXPECT_EQ(childLinkXmlPath, childLinkElem->XmlPath());

  const std::string jointXmlPath =
    xmlPath + "/joint[@name=\"top_level_joint\"]";

  const sdf::Joint *joint = model->JointByIndex(0);
  ASSERT_NE(nullptr, joint);
  sdf::ElementPtr jointElem = joint->Element();
  ASSERT_NE(nullptr, jointElem);
  EXPECT_EQ(testFile, jointElem->FilePath());
  ASSERT_TRUE(jointElem->LineNumber().has_value());
  EXPECT_EQ(11, jointElem->LineNumber().value());
  EXPECT_EQ(jointXmlPath, jointElem->XmlPath());

  const std::string nestedModelXmlPath =
    xmlPath + "/model[@name=\"nested_model\"]";

  const sdf::Model *nestedModel = model->ModelByName("nested_model");
  ASSERT_NE(nullptr, nestedModel);
  sdf::ElementPtr nestedModelElem = nestedModel->Element();
  ASSERT_NE(nullptr, nestedModelElem);
  EXPECT_EQ(testFile, nestedModelElem->FilePath());
  ASSERT_TRUE(nestedModelElem->LineNumber().has_value());
  EXPECT_EQ(5, nestedModelElem->LineNumber().value());
  EXPECT_EQ(nestedModelXmlPath, nestedModelElem->XmlPath());

  const std::string nestedLinkXmlPath =
    nestedModelXmlPath + "/link[@name=\"nested_link01\"]";

  const sdf::Link *nestedLink = nestedModel->LinkByName("nested_link01");
  ASSERT_NE(nullptr, nestedLink);
  sdf::ElementPtr nestedLinkElem = nestedLink->Element();
  ASSERT_NE(nullptr, nestedLinkElem);
  EXPECT_EQ(testFile, nestedLinkElem->FilePath());
  ASSERT_TRUE(nestedLinkElem->LineNumber().has_value());
  EXPECT_EQ(6, nestedLinkElem->LineNumber().value());
  EXPECT_EQ(nestedLinkXmlPath, nestedLinkElem->XmlPath());

  const std::string nestedNestedModelXmlPath =
    nestedModelXmlPath + "/model[@name=\"nested_nested_model\"]";

  const sdf::Model *nestedNestedModel =
    nestedModel->ModelByName("nested_nested_model");
  ASSERT_NE(nullptr, nestedNestedModel);
  sdf::ElementPtr nestedNestedModelElem = nestedNestedModel->Element();
  ASSERT_NE(nullptr, nestedNestedModelElem);
  EXPECT_EQ(testFile, nestedNestedModelElem->FilePath());
  ASSERT_TRUE(nestedNestedModelElem->LineNumber().has_value());
  EXPECT_EQ(7, nestedNestedModelElem->LineNumber().value());
  EXPECT_EQ(nestedNestedModelXmlPath, nestedNestedModelElem->XmlPath());

  const std::string nestedNestedLinkXmlPath =
    nestedNestedModelXmlPath + "/link[@name=\"nested_nested_link01\"]";

  const sdf::Link *nestedNestedLink =
    nestedNestedModel->LinkByName("nested_nested_link01");
  ASSERT_NE(nullptr, nestedNestedLink);
  sdf::ElementPtr nestedNestedLinkElem = nestedNestedLink->Element();
  ASSERT_NE(nullptr, nestedNestedLinkElem);
  EXPECT_EQ(testFile, nestedNestedLinkElem->FilePath());
  ASSERT_TRUE(nestedNestedLinkElem->LineNumber().has_value());
  EXPECT_EQ(8, nestedNestedLinkElem->LineNumber().value());
  EXPECT_EQ(nestedNestedLinkXmlPath, nestedNestedLinkElem->XmlPath());
}

//////////////////////////////////////////////////
TEST(ElementTracing, includes)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes.sdf");
  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  EXPECT_TRUE(errors.empty());

  const std::string xmlPath = "/sdf";

  sdf::ElementPtr rootElem = root.Element();
  ASSERT_NE(nullptr, rootElem);
  EXPECT_EQ(worldFile, rootElem->FilePath());
  ASSERT_TRUE(rootElem->LineNumber().has_value());
  EXPECT_EQ(2, rootElem->LineNumber().value());
  EXPECT_EQ(xmlPath, rootElem->XmlPath());

  const std::string worldXmlPath = xmlPath + "/world[@name=\"default\"]";
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  sdf::ElementPtr worldElem = world->Element();
  ASSERT_NE(nullptr, worldElem);
  EXPECT_EQ(worldFile, worldElem->FilePath());
  ASSERT_TRUE(worldElem->LineNumber().has_value());
  EXPECT_EQ(3, worldElem->LineNumber().value());
  EXPECT_EQ(worldXmlPath, worldElem->XmlPath());

  // Actors
  const std::string actorFilePath = sdf::testing::TestFile(
      "integration", "model", "test_actor", "model.sdf");
  const std::string actorXmlPath = "/sdf/actor[@name=\"actor\"]";

  const sdf::Actor *actor = world->ActorByIndex(0);
  ASSERT_NE(nullptr, actor);
  sdf::ElementPtr actorElem = actor->Element();
  ASSERT_NE(nullptr, actorElem);
  EXPECT_EQ(actorFilePath, actorElem->FilePath());
  ASSERT_TRUE(actorElem->LineNumber().has_value());
  EXPECT_EQ(4, actorElem->LineNumber().value());
  EXPECT_EQ(actorXmlPath, actorElem->XmlPath());

  const std::string overrideActorXmlPath =
      "/sdf/actor[@name=\"override_actor_name\"]";
  const sdf::Actor *overrideActor = world->ActorByIndex(1);
  ASSERT_NE(nullptr, overrideActor);
  sdf::ElementPtr overrideActorElem = overrideActor->Element();
  ASSERT_NE(nullptr, overrideActorElem);
  EXPECT_EQ(actorFilePath, overrideActorElem->FilePath());
  ASSERT_TRUE(overrideActorElem->LineNumber().has_value());
  EXPECT_EQ(4, overrideActorElem->LineNumber().value());
  EXPECT_EQ(overrideActorXmlPath, overrideActorElem->XmlPath());

  const std::string overrideActorPluginXmlPath =
      worldXmlPath + "/include[6]/plugin[0]";

  sdf::ElementPtr overrideActorPluginElem =
      overrideActorElem->GetElement("plugin");
  ASSERT_NE(nullptr, overrideActorPluginElem);
  EXPECT_EQ(actorFilePath, overrideActorPluginElem->FilePath());
  ASSERT_TRUE(overrideActorPluginElem->LineNumber().has_value());
  EXPECT_EQ(40, overrideActorPluginElem->LineNumber().value());
  EXPECT_EQ(overrideActorPluginXmlPath, overrideActorPluginElem->XmlPath());

  // Lights
  const std::string lightFilePath = sdf::testing::TestFile(
      "integration", "model", "test_light", "model.sdf");
  const std::string lightXmlPath = "/sdf/light[@name=\"point_light\"]";

  const sdf::Light *light = world->LightByIndex(0);
  ASSERT_NE(nullptr, light);
  sdf::ElementPtr lightElem = light->Element();
  ASSERT_NE(nullptr, lightElem);
  EXPECT_EQ(lightFilePath, lightElem->FilePath());
  ASSERT_TRUE(lightElem->LineNumber().has_value());
  EXPECT_EQ(3, lightElem->LineNumber().value());
  EXPECT_EQ(lightXmlPath, lightElem->XmlPath());

  const std::string overrideLightXmlPath =
      "/sdf/light[@name=\"override_light_name\"]";
  const sdf::Light *overrideLight = world->LightByIndex(1);
  ASSERT_NE(nullptr, overrideLight);
  sdf::ElementPtr overrideLightElem = overrideLight->Element();
  ASSERT_NE(nullptr, overrideLightElem);
  EXPECT_EQ(lightFilePath, overrideLightElem->FilePath());
  ASSERT_TRUE(overrideLightElem->LineNumber().has_value());
  EXPECT_EQ(3, overrideLightElem->LineNumber().value());
  EXPECT_EQ(overrideLightXmlPath, overrideLightElem->XmlPath());

  // Models
  std::string modelFilePath = sdf::testing::TestFile(
      "integration", "model", "test_model", "model.sdf");
  const std::string modelXmlPath = "/sdf/model[@name=\"test_model\"]";

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  sdf::ElementPtr modelElem = model->Element();
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ(modelFilePath, modelElem->FilePath());
  ASSERT_TRUE(modelElem->LineNumber().has_value());
  EXPECT_EQ(4, modelElem->LineNumber().value());
  EXPECT_EQ(modelXmlPath, modelElem->XmlPath());

  const std::string overrideModelXmlPath =
      "/sdf/model[@name=\"override_model_name\"]";
  const sdf::Model *overrideModel = world->ModelByIndex(1);
  ASSERT_NE(nullptr, overrideModel);
  sdf::ElementPtr overrideModelElem = overrideModel->Element();
  ASSERT_NE(nullptr, overrideModelElem);
  EXPECT_EQ(modelFilePath, overrideModelElem->FilePath());
  ASSERT_TRUE(overrideModelElem->LineNumber().has_value());
  EXPECT_EQ(4, overrideModelElem->LineNumber().value());
  EXPECT_EQ(overrideModelXmlPath, overrideModelElem->XmlPath());

  const std::string overrideModelPluginXmlPath =
      worldXmlPath + "/include[1]/plugin[0]";

  sdf::ElementPtr overrideModelPluginElem =
      overrideModelElem->GetElement("plugin");
  ASSERT_NE(nullptr, overrideModelPluginElem);
  EXPECT_EQ(modelFilePath, overrideModelPluginElem->FilePath());
  ASSERT_TRUE(overrideModelPluginElem->LineNumber().has_value());
  EXPECT_EQ(14, overrideModelPluginElem->LineNumber().value());
  EXPECT_EQ(overrideModelPluginXmlPath, overrideModelPluginElem->XmlPath());

#ifdef _WIN32
  // There is a / in the last argument of TestFile here, as sdf::findFile does
  // not sanitize the file paths provided, which in this case is
  // 'test_model/model.sdf', therefore Windows machines will assign the path
  // '\\path\\to\\integration\\model\\test_model/model.sdf', instead of
  // '\\path\\to\\integration\\model\\test_model\\model.sdf'.
  // Reference issue #572.
  modelFilePath = sdf::testing::TestFile(
      "integration", "model", "test_model/model.sdf");
#endif
  const std::string overrideModelWithFileXmlPath =
      "/sdf/model[@name=\"test_model_with_file\"]";

  const sdf::Model *overrideModelWithFile = world->ModelByIndex(2);
  ASSERT_NE(nullptr, overrideModelWithFile);
  sdf::ElementPtr overrideModelWithFileElem = overrideModelWithFile->Element();
  ASSERT_NE(nullptr, overrideModelWithFileElem);
  EXPECT_EQ(modelFilePath, overrideModelWithFileElem->FilePath());
  ASSERT_TRUE(overrideModelWithFileElem->LineNumber().has_value());
  EXPECT_EQ(4, overrideModelWithFileElem->LineNumber().value());
  EXPECT_EQ(overrideModelWithFileXmlPath,
      overrideModelWithFileElem->XmlPath());
}

