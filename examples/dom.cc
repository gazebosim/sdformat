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
#include <iostream>
#include <sdf/Root.hh>

int main(int argc, char **argv)
{
  // Check for a second argument, which should be the sdf file to parse
  if (argc != 2)
  {
    std::cerr << "Usage: dom <sdf_file>\n";
    return -1;
  }

  /// [rootUsage]
  sdf::Root root;
  if (root.Load(argv[1]))
  {
    root.Print();
    std::cerr << "Valid SDF file. Parsed values shown above.\n";
    return 0;
  }
  /// [rootUsage]

  std::cerr << "Invalid SDF file.\n";
  return -1;
}
