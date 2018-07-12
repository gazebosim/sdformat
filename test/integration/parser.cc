/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include <boost/uuid/sha1.hpp>
#include <boost/filesystem.hpp>
#include <string>

#include "test_config.h"

#include "sdf/sdf.hh"
#include "sdf/parser.hh"

const std::string RSDF_TEST_FILE =
  std::string(PROJECT_SOURCE_PATH) + "/test/integration/erb_test.sdf.erb";

/////////////////////////////////////////////////
// Move this function to a more public place if more tests need it.
template<typename T>
std::string get_sha1(const T &_buffer)
{
  boost::uuids::detail::sha1 sha1;
  unsigned int hash[5];
  std::stringstream stream;

  if (_buffer.size() == 0)
  {
    sha1.process_bytes(NULL, 0);
  }
  else
  {
    sha1.process_bytes(&(_buffer[0]), _buffer.size() * sizeof(_buffer[0]));
  }

  sha1.get_digest(hash);

  for (std::size_t i = 0; i < sizeof(hash) / sizeof(hash[0]); ++i)
  {
    stream << std::setfill('0')
      << std::setw(sizeof(hash[0]) * 2)
      << std::hex
      << hash[i];
  }

  return stream.str();
}

/////////////////////////////////////////////////
TEST(Parser, ParseERB)
{
  char *pathCStr = getenv("SDF_PATH");
  boost::filesystem::path path = PROJECT_SOURCE_PATH;
  path = path / "sdf" / SDF_VERSION;
  setenv("SDF_PATH", path.string().c_str(), 1);

  // Parse the ERB file
  std::string parsed;
  EXPECT_TRUE(sdf::erbFile(RSDF_TEST_FILE, parsed));

  // Make sure the parsed string is correct.
  EXPECT_EQ(get_sha1(parsed), "6daaea47155b80ae9ff20e9b41948f455dd6f39b");

  // Read the parsed string into an SDF object
  sdf::SDFPtr p(new sdf::SDF());
  sdf::init(p);
  EXPECT_TRUE(sdf::readString(parsed, p));

  // There should be a model
  EXPECT_TRUE(p->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = p->Root()->GetElement("model");
  ASSERT_TRUE(modelElem != NULL);

  // The model should have a link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_TRUE(linkElem != NULL);

  // The link should have a pose
  EXPECT_TRUE(linkElem->HasElement("pose"));
  sdf::ElementPtr poseElem = linkElem->GetElement("pose");
  ASSERT_TRUE(poseElem != NULL);

  // The pose.pos.z should equal 0.005
  auto pose = linkElem->Get<ignition::math::Pose3d>("pose");
  EXPECT_DOUBLE_EQ(pose.Pos().Z(), 0.005);

  if (pathCStr)
  {
    setenv("SDF_PATH", pathCStr, 1);
  }
  else
  {
    unsetenv("SDF_PATH");
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
