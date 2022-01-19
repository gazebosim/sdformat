/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <string.h>

#include <ignition/utils/cli/CLI.hpp>
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/sdf.hh"
#include "sdf/usd/World.hh"

//////////////////////////////////////////////////
/// \brief Enumeration of available commands
enum class Command
{
  kNone,
};

//////////////////////////////////////////////////
/// \brief Structure to hold all available topic options
struct Options
{
  /// \brief Command to execute
  Command command{Command::kNone};

  /// \brief input filename
  std::string inputFilename{"input.sdf"};

  /// \brief output filename
  std::string outputFilename{"output.sdf"};
};

void runCommand(const Options &_opt)
{
  sdf::Root root;
  auto errors = root.Load(_opt.inputFilename);
  if (!errors.empty())
  {
    std::cerr << "Errors encountered:\n";
    for (const auto &e : errors)
      std::cout << e << "\n";
    exit(-2);
  }

  // only support SDF files with exactly 1 world for now
  if (root.WorldCount() != 1u)
  {
    std::cerr << _opt.inputFilename << " does not have exactly 1 world\n";
    exit(-3);
  }

  auto world = root.WorldByIndex(0u);
  if (!world)
  {
    std::cerr << "Error retrieving the world from "
              << _opt.inputFilename << "\n";
    exit(-4);
  }

  auto stage = pxr::UsdStage::CreateInMemory();

  const auto worldPath = std::string("/" + world->Name());
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  if (usdErrors.empty())
  {
    std::cerr << "Error parsing world [" << world->Name() << "]\n";
    exit(-5);
  }

  if (!stage->GetRootLayer()->Export(_opt.outputFilename))
  {
    std::cerr << "Issue saving USD to " << _opt.outputFilename << "\n";
    exit(-6);
  }
}

void addFlags(CLI::App &_app)
{
  auto opt = std::make_shared<Options>();

  _app.add_option("-i,--input",
    opt->inputFilename,
    "Input filename");

  _app.add_option("-o,--output",
    opt->outputFilename,
    "Output filename");

  _app.callback([&_app, opt](){
    runCommand(*opt);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Sdf format converter"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("--version", [](){
    std::cout << SDF_VERSION_FULL << std::endl;
    throw CLI::Success();
  });

  addFlags(app);
  CLI11_PARSE(app, argc, argv);

  return 0;
}
