/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "sdf/sdf.hh"

#include "test_config.h"

const std::string SDF_SIMPLE_TEST_FILE =
  std::string(PROJECT_SOURCE_PATH) + "/test/integration/simple.sdf";

///////////////////////////////////////////////
// Implementation of get_sha1
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
TEST(ConvertToURDF, ConvertSimple)
{
  std::string result;
  std::cout << SDF_SIMPLE_TEST_FILE << "\n";
  EXPECT_TRUE(sdf::ConvertToURDF::ConvertFile(SDF_SIMPLE_TEST_FILE, result));

  std::string sha1 = get_sha1(result);
  EXPECT_EQ(sha1, "653a3ac33583db12d27f36dd53027e827e808941");
}
