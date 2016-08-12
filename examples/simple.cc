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
#include <sdf/sdf.hh>

int main(int argc, char **argv)
{
  // Check for a second argument, which should be the sdf file to parse
  if (argc != 2)
  {
    std::cerr << "Usage: simple <sdf_file>\n";
    return -1;
  }

  // Create and initialize the data structure that will hold the parsed SDF data
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  // Read an SDF file, and store the result in sdfParsed.
  if (sdf::readFile(argv[1], sdfParsed))
  {
    // Get a pointer to the model element
    sdf::ElementPtr model = sdfParsed->Root()->GetElement("model");

    // Get the "name" attribute from the model
    std::string modelName = model->Get<std::string>("name");

    std::cout << "Model name = " << modelName << "\n";

    // Get the child "link" element, and its name
    auto link = model->GetElement("link");
    auto linkName = link->Get<std::string>("name");

    std::cout << "Link name = " << linkName << "\n";
    return 0;
  }

  return -1;
}
