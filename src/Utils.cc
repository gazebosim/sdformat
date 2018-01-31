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
#include <algorithm>
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
sdf::Errors sdf::loadModels(sdf::ElementPtr _sdf, std::vector<Model> &_models)
{
  Errors errors;

  // Check that a model element exists.
  if (_sdf->HasElement("model"))
  {
    // Read all the models.
    sdf::ElementPtr elem = _sdf->GetElement("model");
    while (elem)
    {
      Model model;

      // Load the model and capture the errors.
      Errors modelLoadErrors = model.Load(elem);

      // If there are no errors...
      if (modelLoadErrors.empty())
      {
        auto it = std::find_if(_models.begin(), _models.end(),
            [&model](const Model &_m) {return _m.Name() == model.Name();});

        // Check that the model's name does not exist.
        if (it != _models.end())
        {
          errors.push_back({ErrorCode::DUPLICATE_NAME,
              "Model with name[" + model.Name() + "] already exists."});
        }
        else
        {
          // Add the model to the result if no errors have been encountered.
          _models.push_back(std::move(model));
        }
      }
      else
      {
        // Add the load errors to the master error list.
        errors.insert(errors.end(), modelLoadErrors.begin(),
                      modelLoadErrors.end());
      }

      elem = elem->GetNextElement("model");
    }
  }
  // Do not add an error if the model tag is missing. This is an internal
  // function that is used by Root and World. Both of these classes call
  // this function without checking if a model element actually exists. This
  // is a bit of safe code reduction.

  return errors;
}
