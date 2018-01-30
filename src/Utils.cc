/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include "Utils.hh"

/////////////////////////////////////////////////
bool sdf::loadName(sdf::ElementPtr _sdf, std::string &_name)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");

  _name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
Errors sdf::loadModels(sdf::ElementPtr _sdf,
                     std::map<std::string, Model> &_models)
{
  Errors errors;

  // Read all the models
  if (_sdf->HasElement("model"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("model");
    while (elem)
    {
      Model model;

      // Attempt to load the model
      if (model.Load(elem))
      {
        // Check that the model's name does not exist.
        if (_models.find(model.Name()) != _models.end())
        {
          std::cerr << "Model with name[" << model.Name() << "] already exists."
            << " Each model must have a unique name.\n";
          result = false;
        }
        _models.insert(std::make_pair(model.Name(), std::move(model)));
      }
      elem = elem->GetNextElement("model");
    }
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING, "No model element present"});
  }

  return errors;
}
