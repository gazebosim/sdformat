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

#include <string.h>

#include <ignition/utils/cli/CLI.hpp>

#include "sdf/sdf_config.h"
#include "sdf/sdf.hh"

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
  // Read an SDF file, and store the result in sdf.
  auto sdf = sdf::readFile(_opt.inputFilename);

  if (sdf)
  {
    sdf->Write(_opt.outputFilename);
    return;
  }
  std::cerr << "Error sdf was not able to open the file" << '\n';
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
      std::cout << strdup(SDF_VERSION_FULL) << std::endl;
      throw CLI::Success();
  });

  addFlags(app);
  CLI11_PARSE(app, argc, argv);

  return 0;
}
