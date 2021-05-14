/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include "sdf/Frame.hh"
#include "sdf/Model.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Actor.hh"
#include "sdf/Collision.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Light.hh"
#include "sdf/Mesh.hh"
#include "sdf/Visual.hh"
#include "test_config.h"

//////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("integration", "model", _input);
}

//////////////////////////////////////////////////
void checkElementPtr(const sdf::ElementPtr &_elem,
    const std::string &_filePath, int _lineNumber, const std::string &_xmlPath)
{
  ASSERT_NE(nullptr, _elem);
  EXPECT_EQ(_filePath, _elem->FilePath());
  ASSERT_TRUE(_elem->LineNumber().has_value());
  EXPECT_EQ(_lineNumber, _elem->LineNumber().value());
  EXPECT_EQ(_xmlPath, _elem->XmlPath());
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
  checkElementPtr(modelElem, testFile, 2, xmlPath);
  EXPECT_EQ(2, static_cast<int>(model->LinkCount()));
  EXPECT_EQ(1, static_cast<int>(model->JointCount()));
  EXPECT_EQ(1, static_cast<int>(model->ModelCount()));

  const std::string parentLinkXmlPath = xmlPath + "/link[@name=\"parent\"]";

  const sdf::Link *parentLink = model->LinkByName("parent");
  ASSERT_NE(nullptr, parentLink);
  sdf::ElementPtr parentLinkElem = parentLink->Element();
  checkElementPtr(parentLinkElem, testFile, 3, parentLinkXmlPath);

  const std::string childLinkXmlPath = xmlPath + "/link[@name=\"child\"]";

  const sdf::Link *childLink = model->LinkByName("child");
  ASSERT_NE(nullptr, childLink);
  sdf::ElementPtr childLinkElem = childLink->Element();
  checkElementPtr(childLinkElem, testFile, 4, childLinkXmlPath);

  const std::string jointXmlPath =
    xmlPath + "/joint[@name=\"top_level_joint\"]";

  const sdf::Joint *joint = model->JointByIndex(0);
  ASSERT_NE(nullptr, joint);
  sdf::ElementPtr jointElem = joint->Element();
  checkElementPtr(jointElem, testFile, 11, jointXmlPath);

  const std::string nestedModelXmlPath =
    xmlPath + "/model[@name=\"nested_model\"]";

  const sdf::Model *nestedModel = model->ModelByName("nested_model");
  ASSERT_NE(nullptr, nestedModel);
  sdf::ElementPtr nestedModelElem = nestedModel->Element();
  checkElementPtr(nestedModelElem, testFile, 5, nestedModelXmlPath);

  const std::string nestedLinkXmlPath =
    nestedModelXmlPath + "/link[@name=\"nested_link01\"]";

  const sdf::Link *nestedLink = nestedModel->LinkByName("nested_link01");
  ASSERT_NE(nullptr, nestedLink);
  sdf::ElementPtr nestedLinkElem = nestedLink->Element();
  checkElementPtr(nestedLinkElem, testFile, 6, nestedLinkXmlPath);

  const std::string nestedNestedModelXmlPath =
    nestedModelXmlPath + "/model[@name=\"nested_nested_model\"]";

  const sdf::Model *nestedNestedModel =
    nestedModel->ModelByName("nested_nested_model");
  ASSERT_NE(nullptr, nestedNestedModel);
  sdf::ElementPtr nestedNestedModelElem = nestedNestedModel->Element();
  checkElementPtr(nestedNestedModelElem, testFile, 7,
      nestedNestedModelXmlPath);

  const std::string nestedNestedLinkXmlPath =
    nestedNestedModelXmlPath + "/link[@name=\"nested_nested_link01\"]";

  const sdf::Link *nestedNestedLink =
    nestedNestedModel->LinkByName("nested_nested_link01");
  ASSERT_NE(nullptr, nestedNestedLink);
  sdf::ElementPtr nestedNestedLinkElem = nestedNestedLink->Element();
  checkElementPtr(nestedNestedLinkElem, testFile, 8, nestedNestedLinkXmlPath);
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
  checkElementPtr(rootElem, worldFile, 2, xmlPath);

  const std::string worldXmlPath = xmlPath + "/world[@name=\"default\"]";
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  sdf::ElementPtr worldElem = world->Element();
  checkElementPtr(worldElem, worldFile, 3, worldXmlPath);

  // Actors
  const std::string actorFilePath =
      sdf::testing::TestFile("integration", "model", "test_actor") +
      "/model.sdf";
  const std::string actorXmlPath = "/sdf/actor[@name=\"actor\"]";

  const sdf::Actor *actor = world->ActorByIndex(0);
  ASSERT_NE(nullptr, actor);
  sdf::ElementPtr actorElem = actor->Element();
  checkElementPtr(actorElem, actorFilePath, 4, actorXmlPath);

  const std::string overrideActorXmlPath =
      "/sdf/actor[@name=\"override_actor_name\"]";
  const sdf::Actor *overrideActor = world->ActorByIndex(1);
  ASSERT_NE(nullptr, overrideActor);
  sdf::ElementPtr overrideActorElem = overrideActor->Element();
  checkElementPtr(overrideActorElem, actorFilePath, 4, overrideActorXmlPath);

  const std::string overrideActorPluginXmlPath =
      worldXmlPath + "/include[6]/plugin[0]";

  sdf::ElementPtr overrideActorPluginElem =
      overrideActorElem->GetElement("plugin");
  checkElementPtr(
      overrideActorPluginElem, worldFile, 40, overrideActorPluginXmlPath);

  // Lights
  const std::string lightFilePath =
      sdf::testing::TestFile("integration", "model", "test_light") +
      "/model.sdf";
  const std::string lightXmlPath = "/sdf/light[@name=\"point_light\"]";

  const sdf::Light *light = world->LightByIndex(0);
  ASSERT_NE(nullptr, light);
  sdf::ElementPtr lightElem = light->Element();
  checkElementPtr(lightElem, lightFilePath, 3, lightXmlPath);

  const std::string overrideLightXmlPath =
      "/sdf/light[@name=\"override_light_name\"]";
  const sdf::Light *overrideLight = world->LightByIndex(1);
  ASSERT_NE(nullptr, overrideLight);
  sdf::ElementPtr overrideLightElem = overrideLight->Element();
  checkElementPtr(overrideLightElem, lightFilePath, 3, overrideLightXmlPath);

  // Models
  const std::string modelFilePath =
      sdf::testing::TestFile("integration", "model", "test_model") +
      "/model.sdf";
  const std::string modelXmlPath = "/sdf/model[@name=\"test_model\"]";

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  sdf::ElementPtr modelElem = model->Element();
  checkElementPtr(modelElem, modelFilePath, 4, modelXmlPath);

  const std::string overrideModelXmlPath =
      "/sdf/model[@name=\"override_model_name\"]";
  const sdf::Model *overrideModel = world->ModelByIndex(1);
  ASSERT_NE(nullptr, overrideModel);
  sdf::ElementPtr overrideModelElem = overrideModel->Element();
  checkElementPtr(overrideModelElem, modelFilePath, 4, overrideModelXmlPath);

  const std::string overrideModelPluginXmlPath =
      worldXmlPath + "/include[1]/plugin[0]";

  sdf::ElementPtr overrideModelPluginElem =
      overrideModelElem->GetElement("plugin");
  checkElementPtr(
      overrideModelPluginElem, worldFile, 14, overrideModelPluginXmlPath);

  const std::string overrideModelWithFileXmlPath =
      "/sdf/model[@name=\"test_model_with_file\"]";

  const sdf::Model *overrideModelWithFile = world->ModelByIndex(2);
  ASSERT_NE(nullptr, overrideModelWithFile);
  sdf::ElementPtr overrideModelWithFileElem = overrideModelWithFile->Element();
  checkElementPtr(
      overrideModelWithFileElem, modelFilePath, 4,
      overrideModelWithFileXmlPath);
}

