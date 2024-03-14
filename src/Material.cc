/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <filesystem>
#include <optional>
#include <string>
#include <vector>
#include <gz/math/Vector3.hh>

#include "sdf/Material.hh"
#include "sdf/parser.hh"
#include "sdf/Pbr.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Material::Implementation
{
  /// \brief Script URI
  public: std::string scriptUri = "";

  /// \brief Script name
  public: std::string scriptName = "";

  /// \brief Shader type
  public: ShaderType shader = ShaderType::PIXEL;

  /// \brief Normal map
  public: std::string normalMap = "";

  /// \brief Lighting enabled?
  public: bool lighting = true;

  /// \brief Double sided material
  public: bool doubleSided = false;

  /// \brief Ambient color
  public: gz::math::Color ambient {0, 0, 0, 1};

  /// \brief Diffuse color
  public: gz::math::Color diffuse {0, 0, 0, 1};

  /// \brief Specular color
  public: gz::math::Color specular {0, 0, 0, 1};

  /// \brief Specular exponent
  public: double shininess {0};

  /// \brief Emissive color
  public: gz::math::Color emissive {0, 0, 0, 1};

  /// \brief Render order
  public: float renderOrder = 0;

  /// \brief Physically Based Rendering (PBR) properties
  public: std::optional<Pbr> pbr;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief The path to the file where this material was defined.
  public: std::string filePath = "";
};

/////////////////////////////////////////////////
Material::Material()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Material::Load(sdf::ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Material::Load(sdf::ElementPtr _sdf, const sdf::ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  this->dataPtr->filePath = _sdf->FilePath();

  // Check that the provided SDF element is a <material>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "material")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Material, but the provided SDF element is not a "
        "<material>."});
    return errors;
  }

  // Load the script information
  if (_sdf->HasElement("script"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("script", errors);
    std::pair<std::string, bool> uriPair =
        elem->Get<std::string>(errors, "uri", "");
    if (uriPair.first == "__default__")
      uriPair.first = "";

    if (!uriPair.second || uriPair.first.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "A <script> element is missing a child <uri> element, or the "
          "<uri> element is empty."});
    }

    std::unordered_set<std::string> paths;
    if (!this->dataPtr->filePath.empty())
    {
      paths.insert(std::filesystem::path(
          this->dataPtr->filePath).parent_path().string());
    }
    this->dataPtr->scriptUri = resolveURI(uriPair.first, _config, errors,
        paths);

    std::pair<std::string, bool> namePair =
        elem->Get<std::string>(errors, "name", "");
    if (namePair.first == "__default__")
      namePair.first = "";

    if (!namePair.second || namePair.first.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <script> element is missing a child <name> element, or the "
          "<name> element is empty."});
    }
    this->dataPtr->scriptName = namePair.first;
  }

  // Load the shader information
  if (_sdf->HasElement("shader"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("shader", errors);

    std::pair<std::string, bool> typePair =
      elem->Get<std::string>(errors, "type", "pixel");
    if (typePair.first == "pixel")
      this->dataPtr->shader = ShaderType::PIXEL;
    else if (typePair.first == "vertex")
      this->dataPtr->shader = ShaderType::VERTEX;
    else if (typePair.first == "normal_map_objectspace")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_OBJECTSPACE;
    else if (typePair.first == "normal_map_object_space")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_OBJECTSPACE;
    else if (typePair.first == "normal_map_tangentspace")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_TANGENTSPACE;
    else if (typePair.first == "normal_map_tangent_space")
      this->dataPtr->shader = ShaderType::NORMAL_MAP_TANGENTSPACE;
    else
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "The value[" + typePair.first + "] for a <shader><type> element is "
          "not supported"});
    }

    this->dataPtr->normalMap =
        elem->Get<std::string>(errors, "normal_map", "").first;
    if (this->dataPtr->normalMap == "__default__")
      this->dataPtr->normalMap = "";

    if ((this->dataPtr->shader == ShaderType::NORMAL_MAP_OBJECTSPACE ||
         this->dataPtr->shader == ShaderType::NORMAL_MAP_TANGENTSPACE) &&
        this->dataPtr->normalMap.empty())
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A normal map shader type has been specified, but a normal_map has "
          "not."});
    }
  }

  this->dataPtr->renderOrder = _sdf->Get<float>(errors, "render_order",
      this->dataPtr->renderOrder).first;

  this->dataPtr->ambient = _sdf->Get<gz::math::Color>(errors, "ambient",
      this->dataPtr->ambient).first;

  this->dataPtr->diffuse = _sdf->Get<gz::math::Color>(errors, "diffuse",
      this->dataPtr->diffuse).first;

  this->dataPtr->specular = _sdf->Get<gz::math::Color>(errors, "specular",
      this->dataPtr->specular).first;

  this->dataPtr->shininess = _sdf->Get<double>(errors, "shininess",
      this->dataPtr->shininess).first;

  this->dataPtr->emissive = _sdf->Get<gz::math::Color>(errors, "emissive",
      this->dataPtr->emissive).first;

  this->dataPtr->lighting = _sdf->Get<bool>(errors, "lighting",
      this->dataPtr->lighting).first;

  this->dataPtr->doubleSided = _sdf->Get<bool>(errors, "double_sided",
      this->dataPtr->doubleSided).first;

  // load pbr param
  if (_sdf->HasElement("pbr"))
  {
    this->dataPtr->pbr.emplace();
    Errors pbrErrors = this->dataPtr->pbr->Load(
        _sdf->GetElement("pbr", errors));
    errors.insert(errors.end(), pbrErrors.begin(), pbrErrors.end());
  }

  return errors;
}

//////////////////////////////////////////////////
gz::math::Color Material::Ambient() const
{
  return this->dataPtr->ambient;
}

//////////////////////////////////////////////////
void Material::SetAmbient(const gz::math::Color &_color)
{
  this->dataPtr->ambient = _color;
}

//////////////////////////////////////////////////
gz::math::Color Material::Diffuse() const
{
  return this->dataPtr->diffuse;
}

//////////////////////////////////////////////////
void Material::SetDiffuse(const gz::math::Color &_color)
{
  this->dataPtr->diffuse = _color;
}

//////////////////////////////////////////////////
gz::math::Color Material::Specular() const
{
  return this->dataPtr->specular;
}

//////////////////////////////////////////////////
void Material::SetSpecular(const gz::math::Color &_color)
{
  this->dataPtr->specular = _color;
}

//////////////////////////////////////////////////
double Material::Shininess() const
{
  return this->dataPtr->shininess;
}

//////////////////////////////////////////////////
void Material::SetShininess(const double _shininess)
{
  this->dataPtr->shininess = _shininess;
}

//////////////////////////////////////////////////
gz::math::Color Material::Emissive() const
{
  return this->dataPtr->emissive;
}

//////////////////////////////////////////////////
void Material::SetEmissive(const gz::math::Color &_color)
{
  this->dataPtr->emissive = _color;
}

//////////////////////////////////////////////////
float Material::RenderOrder() const
{
  return this->dataPtr->renderOrder;
}

//////////////////////////////////////////////////
void Material::SetRenderOrder(const float _renderOrder)
{
  this->dataPtr->renderOrder = _renderOrder;
}

//////////////////////////////////////////////////
bool Material::Lighting() const
{
  return this->dataPtr->lighting;
}

//////////////////////////////////////////////////
void Material::SetLighting(const bool _lighting)
{
  this->dataPtr->lighting = _lighting;
}

//////////////////////////////////////////////////
bool Material::DoubleSided() const
{
  return this->dataPtr->doubleSided;
}

//////////////////////////////////////////////////
void Material::SetDoubleSided(const bool _doubleSided)
{
  this->dataPtr->doubleSided = _doubleSided;
}

//////////////////////////////////////////////////
sdf::ElementPtr Material::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
std::string Material::ScriptUri() const
{
  return this->dataPtr->scriptUri;
}

//////////////////////////////////////////////////
void Material::SetScriptUri(const std::string &_uri)
{
  this->dataPtr->scriptUri = _uri;
}

//////////////////////////////////////////////////
std::string Material::ScriptName() const
{
  return this->dataPtr->scriptName;
}

//////////////////////////////////////////////////
void Material::SetScriptName(const std::string &_name)
{
  this->dataPtr->scriptName = _name;
}

//////////////////////////////////////////////////
ShaderType Material::Shader() const
{
  return this->dataPtr->shader;
}

//////////////////////////////////////////////////
void Material::SetShader(const ShaderType &_type)
{
  this->dataPtr->shader = _type;
}

//////////////////////////////////////////////////
std::string Material::NormalMap() const
{
  return this->dataPtr->normalMap;
}

//////////////////////////////////////////////////
void Material::SetNormalMap(const std::string &_map)
{
  this->dataPtr->normalMap = _map;
}

//////////////////////////////////////////////////
void Material::SetPbrMaterial(const Pbr &_pbr)
{
  this->dataPtr->pbr = _pbr;
}

//////////////////////////////////////////////////
const Pbr *Material::PbrMaterial() const
{
  return optionalToPointer(this->dataPtr->pbr);
}

//////////////////////////////////////////////////
const std::string &Material::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Material::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
sdf::ElementPtr Material::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Material::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("material.sdf", elem);

  elem->GetElement("ambient", _errors)->Set(_errors, this->Ambient());
  elem->GetElement("diffuse", _errors)->Set(_errors, this->Diffuse());
  elem->GetElement("specular", _errors)->Set(_errors, this->Specular());
  elem->GetElement("emissive", _errors)->Set(_errors, this->Emissive());
  elem->GetElement("render_order", _errors)->Set(_errors, this->RenderOrder());
  elem->GetElement("lighting", _errors)->Set(_errors, this->Lighting());
  elem->GetElement("double_sided", _errors)->Set(_errors, this->DoubleSided());

  // Script, if set
  if (!this->ScriptName().empty() && !this->ScriptUri().empty())
  {
    sdf::ElementPtr scriptElem = elem->GetElement("script", _errors);
    scriptElem->GetElement("uri", _errors)->Set(_errors, this->ScriptUri());
    scriptElem->GetElement("name", _errors)->Set(_errors, this->ScriptName());
  }

  // Shader properties
  sdf::ElementPtr shaderElem = elem->GetElement("shader", _errors);
  switch (this->dataPtr->shader)
  {
    default:
    case ShaderType::PIXEL:
      shaderElem->GetAttribute("type")->Set("pixel", _errors);
      break;
    case ShaderType::VERTEX:
      shaderElem->GetAttribute("type")->Set("vertex", _errors);
      break;
    case ShaderType::NORMAL_MAP_OBJECTSPACE:
      shaderElem->GetAttribute("type")->Set(
          "normal_map_object_space", _errors);
      break;
    case ShaderType::NORMAL_MAP_TANGENTSPACE:
      shaderElem->GetAttribute("type")->Set(
          "normal_map_tangent_space", _errors);
      break;
  }
  if (!this->NormalMap().empty())
    shaderElem->GetElement("normal_map", _errors)->Set(
        _errors, this->NormalMap());

  // PBR material
  if (this->dataPtr->pbr)
  {
    const PbrWorkflow *workflow = this->dataPtr->pbr->Workflow(
        PbrWorkflowType::METAL);
    sdf::ElementPtr pbrElem = elem->GetElement("pbr", _errors);
    if (workflow && workflow->Type() == PbrWorkflowType::METAL)
    {
      sdf::ElementPtr metalElem = pbrElem->GetElement("metal", _errors);
      metalElem->GetElement("albedo_map", _errors)->Set(
          _errors, workflow->AlbedoMap());
      metalElem->GetElement("roughness_map", _errors)->Set(
          _errors, workflow->RoughnessMap());
      metalElem->GetElement("roughness", _errors)->Set(
          _errors, workflow->Roughness());
      metalElem->GetElement("metalness_map", _errors)->Set(
          _errors, workflow->MetalnessMap());
      metalElem->GetElement("metalness", _errors)->Set(
          _errors, workflow->Metalness());
      metalElem->GetElement("ambient_occlusion_map", _errors)->Set(
          _errors, workflow->AmbientOcclusionMap());
      sdf::ElementPtr normalElem = metalElem->GetElement(
          "normal_map", _errors);
      if (workflow->NormalMapType() == NormalMapSpace::TANGENT)
        normalElem->GetAttribute("type")->Set("tangent", _errors);
      else
        normalElem->GetAttribute("type")->Set("object", _errors);
      normalElem->Set(_errors, workflow->NormalMap());

      metalElem->GetElement("emissive_map", _errors)->Set(
          _errors, workflow->EmissiveMap());

      sdf::ElementPtr lightElem = metalElem->GetElement("light_map", _errors);
      lightElem->GetAttribute("uv_set")->Set(
          workflow->LightMapTexCoordSet(), _errors);
      lightElem->Set(_errors, workflow->LightMap());
    }

    workflow = this->dataPtr->pbr->Workflow(PbrWorkflowType::SPECULAR);
    if (workflow && workflow->Type() == PbrWorkflowType::SPECULAR)
    {
      sdf::ElementPtr specularElem = pbrElem->GetElement("specular", _errors);
      specularElem->GetElement("albedo_map", _errors)->Set(
          _errors, workflow->AlbedoMap());
      specularElem->GetElement("specular_map", _errors)->Set(
          _errors, workflow->SpecularMap());
      specularElem->GetElement("environment_map", _errors)->Set(
          _errors, workflow->EnvironmentMap());
      specularElem->GetElement("ambient_occlusion_map", _errors)->Set(
          _errors, workflow->AmbientOcclusionMap());
      specularElem->GetElement("emissive_map", _errors)->Set(
          _errors, workflow->EmissiveMap());
      specularElem->GetElement("glossiness_map", _errors)->Set(
          _errors, workflow->GlossinessMap());
      specularElem->GetElement("glossiness", _errors)->Set(
          _errors, workflow->Glossiness());

      sdf::ElementPtr normalElem = specularElem->GetElement(
          "normal_map", _errors);
      if (workflow->NormalMapType() == NormalMapSpace::TANGENT)
        normalElem->GetAttribute("type")->Set("tangent", _errors);
      else
        normalElem->GetAttribute("type")->Set("object", _errors);
      normalElem->Set(_errors, workflow->NormalMap());

      sdf::ElementPtr lightElem = specularElem->GetElement(
          "light_map", _errors);
      lightElem->GetAttribute("uv_set")->Set(
          workflow->LightMapTexCoordSet(), _errors);
      lightElem->Set(_errors, workflow->LightMap());
    }
  }

  return elem;
}
