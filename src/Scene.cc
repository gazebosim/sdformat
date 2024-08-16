/*
 * Copyright 2019 Open Source Robotics Foundation
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
#include "sdf/parser.hh"
#include "sdf/Scene.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Scene private data.
class sdf::Scene::Implementation
{
  /// \brief True if grid should be enabled
  public: bool grid = true;

  /// \brief True if shadows should be enabled
  public: bool shadows = true;

  /// \brief True if originVisual should be enabled
  public: bool originVisual = true;

  /// \brief Ambient light color of the scene.
  public: gz::math::Color ambient =
      gz::math::Color(0.4f, 0.4f, 0.4f);

  /// \brief Background color of the scene.
  public: gz::math::Color background =
      gz::math::Color(0.7f, 0.7f, .7f);

  /// \brief Pointer to the sky properties.
  public: std::optional<sdf::Sky> sky;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Scene::Scene()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Scene::Load(ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Scene::Load(ElementPtr _sdf, const ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <scene> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "scene")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Scene, but the provided SDF element is not a "
        "<scene>."});
    return errors;
  }

  // Get the ambient property
  this->dataPtr->ambient = _sdf->Get<gz::math::Color>(errors, "ambient",
      this->dataPtr->ambient).first;

  // Get the background color property
  this->dataPtr->background = _sdf->Get<gz::math::Color>(errors, "background",
      this->dataPtr->background).first;

  // Get the grid property
  this->dataPtr->grid = _sdf->Get<bool>(errors, "grid",
      this->dataPtr->grid).first;

  // Get the shadows property
  this->dataPtr->shadows = _sdf->Get<bool>(errors, "shadows",
      this->dataPtr->shadows).first;

  // Get the origin_visual property
  this->dataPtr->originVisual = _sdf->Get<bool>(errors, "origin_visual",
      this->dataPtr->originVisual).first;

  // load sky
  if (_sdf->HasElement("sky"))
  {
    this->dataPtr->sky.emplace();
    Errors err = this->dataPtr->sky->Load(_sdf->GetElement(
                                            "sky", errors), _config);
    errors.insert(errors.end(), err.begin(), err.end());
  }

  return errors;
}

/////////////////////////////////////////////////
gz::math::Color Scene::Ambient() const
{
  return this->dataPtr->ambient;
}

/////////////////////////////////////////////////
void Scene::SetAmbient(const gz::math::Color &_ambient)
{
  this->dataPtr->ambient = _ambient;
}

/////////////////////////////////////////////////
gz::math::Color Scene::Background() const
{
  return this->dataPtr->background;
}

/////////////////////////////////////////////////
void Scene::SetBackground(const gz::math::Color &_background)
{
  this->dataPtr->background = _background;
}
/////////////////////////////////////////////////
bool Scene::Grid() const
{
  return this->dataPtr->grid;
}

/////////////////////////////////////////////////
void Scene::SetGrid(const bool _enabled)
{
  this->dataPtr->grid = _enabled;
}

/////////////////////////////////////////////////
bool Scene::Shadows() const
{
  return this->dataPtr->shadows;
}

/////////////////////////////////////////////////
void Scene::SetShadows(const bool _enabled)
{
  this->dataPtr->shadows = _enabled;
}

/////////////////////////////////////////////////
bool Scene::OriginVisual() const
{
  return this->dataPtr->originVisual;
}

/////////////////////////////////////////////////
void Scene::SetOriginVisual(const bool _enabled)
{
  this->dataPtr->originVisual = _enabled;
}

/////////////////////////////////////////////////
void Scene::SetSky(const sdf::Sky &_sky)
{
  this->dataPtr->sky = _sky;
}

/////////////////////////////////////////////////
const sdf::Sky *Scene::Sky() const
{
  return optionalToPointer(this->dataPtr->sky);
}

/////////////////////////////////////////////////
sdf::ElementPtr Scene::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
sdf::ElementPtr Scene::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Scene::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile(std::string(this->SchemaFile()), elem);

  elem->GetElement("ambient", _errors)->Set(_errors, this->Ambient());
  elem->GetElement("background", _errors)->Set(_errors, this->Background());
  elem->GetElement("grid", _errors)->Set(_errors, this->Grid());
  elem->GetElement("origin_visual", _errors)->Set(
      _errors, this->OriginVisual());
  elem->GetElement("shadows", _errors)->Set(_errors, this->Shadows());

  if (this->dataPtr->sky)
    elem->InsertElement(this->dataPtr->sky->ToElement(_errors), true);

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view Scene::SchemaFile() 
{
    static char kSchemaFile[] = "scene.sdf";
    return kSchemaFile;
}

