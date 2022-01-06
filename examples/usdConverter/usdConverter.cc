/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <pxr/usd/usd/stage.h>

#include "sdf/sdf.hh"
#include "sdf/world.hh"

int main(int argc, const char* argv[])
{
  if (argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " <sdf-path> <usd-path>\n";
    return -1;
  }

  sdf::Root root;
  auto errors = root.Load(argv[1]);
  if (!errors.empty())
  {
    std::cerr << "Errors encountered:\n";
    for (const auto &e : errors)
      std::cout << e << "\n";
    return -2;
  }

  // only support SDF files with exactly 1 world for now
  if (root.WorldCount() != 1u)
  {
    std::cerr << argv[1] << " does not have exactly 1 world\n";
    return -3;
  }

  auto world = root.WorldByIndex(0u);
  if (!world)
  {
    std::cerr << "Error retrieving the world from " << argv[1] << "\n";
    return -4;
  }

  auto stage = pxr::UsdStage::CreateInMemory();

  const auto worldPath = std::string("/" + world->Name());
  if (!usd::ParseSdfWorld(*world, stage, worldPath))
  {
    std::cerr << "Error parsing world [" << world->Name() << "]\n";
    //return -5;
  }

  if (!stage->GetRootLayer()->Export(argv[2]))
  {
    std::cerr << "Issue saving USD to " << argv[2] << "\n";
    return -6;
  }
}
