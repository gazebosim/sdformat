/*
 * Copyright 2014 Open Source Robotics Foundation
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
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <sdf/ConvertToURDF.hh>

namespace po = boost::program_options;

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  po::variables_map vm;
  po::options_description visibleOptions;
  visibleOptions.add_options()
    ("help,h", "Print this help message")
    ("file,f", po::value<std::string>(), "SDF file to convert to URDF")
    ("out,o", po::value<std::string>(), "Output filename");

  // Parse the command line arguments
  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(
          visibleOptions).run(), vm);
    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n";
    return -1;
  }

  // Convert the given file.
  if (vm.count("file"))
  {
    std::string result;
    if (!sdf::ConvertToURDF::ConvertFile(vm["file"].as<std::string>(), result))
    {
      std::cerr << "Unable to convert file[" << vm["file"].as<std::string>()
        << std::endl;
      return -1;
    }

    // Output to file, if specified. Otherwise output to screen
    if (vm.count("out"))
    {
      std::ofstream out(vm["out"].as<std::string>().c_str(), std::ios::out);
      out << result;
      out.close();
    }
    else
    {
      std::cout << result;
    }
  }
  else
  {
    std::cerr << "Specify a file to convert with the -f option\n";
    return -1;
  }

  return 0;
}
