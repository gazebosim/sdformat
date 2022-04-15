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

#include <iostream>
#include <string>

#include <ignition/utils/cli/CLI.hpp>

#include "sdf/usd/usd_parser/Parser.hh"
#include "sdf/usd/UsdError.hh"
#include "sdf/config.hh"

//////////////////////////////////////////////////
/// \brief Enumeration of available commands
enum class Command
{
  kNone,
};

//////////////////////////////////////////////////
/// \brief Structure to hold all filenames
struct Options
{
  /// \brief Command to execute
  Command command{Command::kNone};

  /// \brief input filename
  std::string inputFilename{"input.usd"};

  /// \brief output filename
  std::string outputFilename{"output.sdf"};
};

void runCommand(const Options &_opt)
{
  const auto errors =
    sdf::usd::parseUSDFile(_opt.inputFilename, _opt.outputFilename);
  if (!errors.empty())
  {
    std::cerr << "Errors occurred when generating [" << _opt.outputFilename
              << "] from [" << _opt.inputFilename << "]:\n";
    for (const auto &e : errors)
      std::cerr << "\t" << e << "\n";
  }
}

void addFlags(CLI::App &_app)
{
  auto opt = std::make_shared<Options>();

  _app.add_option("input",
    opt->inputFilename,
    "Input filename. Defaults to input.usd unless otherwise specified.");

  _app.add_option("output",
    opt->outputFilename,
    "Output filename. Defaults to output.sdf unless otherwise specified.");

  _app.callback([&_app, opt](){
    runCommand(*opt);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"USD to SDF converter"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("--version", [](){
    std::cout << SDF_VERSION_FULL << std::endl;
    throw CLI::Success();
  });

  addFlags(app);
  CLI11_PARSE(app, argc, argv);

  return 0;
}
