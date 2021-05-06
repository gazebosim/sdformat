/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/parser.hh"
#include "test_config.h"
#include "test_utils.hh"

////////////////////////////////////////////////////
TEST(DeprecatedSpecs, Spec1_0)
{
  const std::string filename =
    sdf::testing::TestFile("integration", "deprecated_sdf_1-0.sdf");
  sdf::SDFPtr sdf(new sdf::SDF());
  EXPECT_FALSE(sdf::initFile(filename, sdf));
}

////////////////////////////////////////////////////
TEST(DeprecatedSpecs, Spec1_2)
{
  const std::string filename =
    sdf::testing::TestFile("integration", "deprecated_sdf_1-2.sdf");
  sdf::SDFPtr sdf(new sdf::SDF());
  EXPECT_FALSE(sdf::initFile(filename, sdf));
}

////////////////////////////////////////////////////
TEST(DeprecatedElements, CanEmitErrors)
{
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  auto elem = std::make_shared<sdf::Element>();
  elem->SetRequired("-1");
  elem->SetName("testElem");
  // Change the root of sdf so we can test "testElem" in isolation
  sdf->Root(elem);
  auto config = sdf::ParserConfig::GlobalConfig();
  config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  sdf::Errors errors;
  EXPECT_TRUE(sdf::readString(
      "<sdf version='1.8'><testElem/></sdf>", config, sdf, errors));
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_DEPRECATED, errors[0].Code());
}

bool contains(const std::string &_a, const std::string &_b)
{
  return _a.find(_b) != std::string::npos;
}

////////////////////////////////////////////////////
TEST(DeprecatedElements, CanEmitWarningWithErrorEnforcmentPolicy)
{
  // Redirect sdfwarn output
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

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  auto elem = std::make_shared<sdf::Element>();
  elem->SetRequired("-1");
  elem->SetName("testElem");
  // Change the root of sdf so we can test "testElem" in isolation
  sdf->Root(elem);
  {
    auto config = sdf::ParserConfig::GlobalConfig();
    config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
    config.SetDeprecatedElementsPolicy(sdf::EnforcementPolicy::WARN);
    sdf::Errors errors;
    EXPECT_TRUE(sdf::readString(
          "<sdf version='1.8'><testElem/></sdf>", config, sdf, errors));
    EXPECT_TRUE(errors.empty());
    EXPECT_PRED2(contains, buffer.str(), "SDF Element[testElem] is deprecated");
  }
  // Flip the order of calling SetDeprecatedElementsPolicy and
  // SetWarningsPolicy.
  {
    // clear the contents of the buffer
    buffer.str("");
    auto config = sdf::ParserConfig::GlobalConfig();
    config.SetDeprecatedElementsPolicy(sdf::EnforcementPolicy::WARN);
    config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
    sdf::Errors errors;
    EXPECT_TRUE(sdf::readString(
          "<sdf version='1.8'><testElem/></sdf>", config, sdf, errors));
    EXPECT_TRUE(errors.empty());
    EXPECT_PRED2(contains, buffer.str(), "SDF Element[testElem] is deprecated");
  }
  // Test ResetDeprecatedElementsPolicy
  {
    auto config = sdf::ParserConfig::GlobalConfig();
    config.SetDeprecatedElementsPolicy(sdf::EnforcementPolicy::WARN);
    config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
    config.ResetDeprecatedElementsPolicy();
    sdf::Errors errors;
    EXPECT_TRUE(sdf::readString(
          "<sdf version='1.8'><testElem/></sdf>", config, sdf, errors));
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_DEPRECATED, errors[0].Code());
  }
}
