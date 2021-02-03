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
#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>

#include "sdf/Model.hh"
#include "sdf/Types.hh"
#include "sdf/Pbr.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Private data for PbrWorkflow class
class sdf::PbrWorkflow::Implementation
{
  /// \brief Workflow type
  public: PbrWorkflowType type = PbrWorkflowType::NONE;

  /// \brief Albedo map
  public: std::string albedoMap = "";

  /// \brief Normal map
  public: std::string normalMap = "";

  /// \brief Normal map space
  public: NormalMapSpace normalMapSpace = NormalMapSpace::TANGENT;

  /// \brief Environment map
  public: std::string environmentMap = "";

  /// \brief Ambient occlusion map
  public: std::string ambientOcclusionMap = "";

  /// \brief Roughness map (metal workflow only)
  public: std::string roughnessMap = "";

  /// \brief Metalness map (metal workflow only)
  public: std::string metalnessMap = "";

  /// \brief Emissive map
  public: std::string emissiveMap = "";

  /// \brief Light map
  public: std::string lightMapFilename;

  /// \brief Light map texture coordinate set
  public: unsigned int lightMapUvSet = 0u;

  /// \brief Roughness value (metal workflow only)
  public: double roughness = 0.5;

  /// \brief Metalness value (metal workflow only)
  public: double metalness = 0.5;

  /// \brief Specular map (specular workflow only)
  public: std::string specularMap = "";

  /// \brief Glossiness map (specular workflow only)
  public: std::string glossinessMap = "";

  /// \brief Glossiness value (specular workflow only)
  public: double glossiness = 0.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};


/// \brief Private data for Pbr class
class sdf::Pbr::Implementation
{
  /// \brief PBR workflows
  public: std::map<PbrWorkflowType, PbrWorkflow> workflows;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
PbrWorkflow::PbrWorkflow()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
bool PbrWorkflow::operator!=(const PbrWorkflow &_pbr) const
{
  return !(*this == _pbr);
}

/////////////////////////////////////////////////
bool PbrWorkflow::operator==(const PbrWorkflow &_workflow) const
{
  return (this->dataPtr->albedoMap == _workflow.dataPtr->albedoMap)
    && (this->dataPtr->normalMap == _workflow.dataPtr->normalMap)
    && (this->dataPtr->metalnessMap == _workflow.dataPtr->metalnessMap)
    && (this->dataPtr->roughnessMap == _workflow.dataPtr->roughnessMap)
    && (this->dataPtr->glossinessMap == _workflow.dataPtr->glossinessMap)
    && (this->dataPtr->environmentMap == _workflow.dataPtr->environmentMap)
    && (this->dataPtr->emissiveMap == _workflow.dataPtr->emissiveMap)
    && (this->dataPtr->lightMapFilename == _workflow.dataPtr->lightMapFilename)
    && (this->dataPtr->ambientOcclusionMap ==
        _workflow.dataPtr->ambientOcclusionMap)
    && (ignition::math::equal(
        this->dataPtr->metalness, _workflow.dataPtr->metalness))
    && (ignition::math::equal(
        this->dataPtr->roughness, _workflow.dataPtr->roughness))
    && (ignition::math::equal(
        this->dataPtr->glossiness, _workflow.dataPtr->glossiness));
}

/////////////////////////////////////////////////
Errors PbrWorkflow::Load(sdf::ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Load the workflow element
  sdf::ElementPtr workflowElem;
  if (_sdf->GetName() == "metal")
  {
    this->dataPtr->type = PbrWorkflowType::METAL;

    this->dataPtr->roughnessMap = _sdf->Get<std::string>("roughness_map",
      this->dataPtr->roughnessMap).first;

    this->dataPtr->metalnessMap = _sdf->Get<std::string>("metalness_map",
      this->dataPtr->metalnessMap).first;

    this->dataPtr->roughness = _sdf->Get<double>("roughness",
      this->dataPtr->roughness).first;

    this->dataPtr->metalness = _sdf->Get<double>("metalness",
      this->dataPtr->metalness).first;
  }
  else if (_sdf->GetName() == "specular")
  {
    this->dataPtr->type = PbrWorkflowType::SPECULAR;

    this->dataPtr->specularMap = _sdf->Get<std::string>("specular_map",
      this->dataPtr->specularMap).first;

    this->dataPtr->glossinessMap = _sdf->Get<std::string>("glossiness_map",
      this->dataPtr->glossinessMap).first;

    this->dataPtr->glossiness = _sdf->Get<double>("glossiness",
      this->dataPtr->glossiness).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a PbrWorkflow material, but the provided SDF "
        " element is not <metal> or <specular>."});
    return errors;
  }

  this->dataPtr->albedoMap = _sdf->Get<std::string>("albedo_map",
      this->dataPtr->albedoMap).first;

  this->dataPtr->normalMap= _sdf->Get<std::string>("normal_map",
    this->dataPtr->normalMap).first;

  this->dataPtr->environmentMap = _sdf->Get<std::string>("environment_map",
      this->dataPtr->environmentMap).first;

  this->dataPtr->ambientOcclusionMap =
      _sdf->Get<std::string>("ambient_occlusion_map",
      this->dataPtr->ambientOcclusionMap).first;

  this->dataPtr->emissiveMap = _sdf->Get<std::string>("emissive_map",
      this->dataPtr->emissiveMap).first;

  if (_sdf->HasElement("light_map"))
  {
    sdf::ElementPtr lightMapElem = _sdf->GetElement("light_map");
    this->dataPtr->lightMapFilename = lightMapElem->Get<std::string>();
    this->dataPtr->lightMapUvSet = lightMapElem->Get<unsigned int>("uv_set",
        this->dataPtr->lightMapUvSet).first;
  }

  return errors;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::AlbedoMap() const
{
  return this->dataPtr->albedoMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetAlbedoMap(const std::string &_map)
{
  this->dataPtr->albedoMap = _map;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::NormalMap() const
{
  return this->dataPtr->normalMap;
}

//////////////////////////////////////////////////
NormalMapSpace PbrWorkflow::NormalMapType() const
{
  return this->dataPtr->normalMapSpace;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetNormalMap(const std::string &_map, NormalMapSpace _space)
{
  this->dataPtr->normalMap = _map;
  this->dataPtr->normalMapSpace = _space;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::EnvironmentMap() const
{
  return this->dataPtr->environmentMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetEnvironmentMap(const std::string &_map)
{
  this->dataPtr->environmentMap = _map;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::AmbientOcclusionMap() const
{
  return this->dataPtr->ambientOcclusionMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetAmbientOcclusionMap(const std::string &_map)
{
  this->dataPtr->ambientOcclusionMap = _map;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::RoughnessMap() const
{
  return this->dataPtr->roughnessMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetRoughnessMap(const std::string &_map)
{
  this->dataPtr->roughnessMap = _map;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::MetalnessMap() const
{
  return this->dataPtr->metalnessMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetMetalnessMap(const std::string &_map)
{
  this->dataPtr->metalnessMap = _map;
}

//////////////////////////////////////////////////
double PbrWorkflow::Metalness() const
{
  return this->dataPtr->metalness;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetMetalness(const double _metalness)
{
  this->dataPtr->metalness = _metalness;
}

//////////////////////////////////////////////////
double PbrWorkflow::Roughness() const
{
  return this->dataPtr->roughness;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetRoughness(const double _roughness)
{
  this->dataPtr->roughness = _roughness;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::SpecularMap() const
{
  return this->dataPtr->specularMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetSpecularMap(const std::string &_map)
{
  this->dataPtr->specularMap = _map;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::GlossinessMap() const
{
  return this->dataPtr->glossinessMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetGlossinessMap(const std::string &_map)
{
  this->dataPtr->glossinessMap = _map;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetGlossiness(const double _glossiness)
{
  this->dataPtr->glossiness = _glossiness;
}

//////////////////////////////////////////////////
double PbrWorkflow::Glossiness() const
{
  return this->dataPtr->glossiness;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::EmissiveMap() const
{
  return this->dataPtr->emissiveMap;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetEmissiveMap(const std::string &_map)
{
  this->dataPtr->emissiveMap = _map;
}

//////////////////////////////////////////////////
std::string PbrWorkflow::LightMap() const
{
  return this->dataPtr->lightMapFilename;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetLightMap(const std::string &_map, unsigned int _uvSet)
{
  this->dataPtr->lightMapFilename = _map;
  this->dataPtr->lightMapUvSet = _uvSet;
}

//////////////////////////////////////////////////
unsigned int PbrWorkflow::LightMapTexCoordSet() const
{
  return this->dataPtr->lightMapUvSet;
}

//////////////////////////////////////////////////
sdf::ElementPtr PbrWorkflow::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
PbrWorkflowType PbrWorkflow::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void PbrWorkflow::SetType(PbrWorkflowType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
Pbr::Pbr()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Pbr::Load(sdf::ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <pbr>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "pbr")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Pbr material, but the provided SDF element is "
        "not a <pbr>."});
    return errors;
  }

  // load all workflows
  sdf::ElementPtr workflowElem = _sdf->GetFirstElement();
  while (workflowElem)
  {
    PbrWorkflow workflow;
    Errors workflowErrors = workflow.Load(workflowElem);
    if (workflowErrors.empty())
      this->dataPtr->workflows[workflow.Type()] = workflow;
    else
      errors.insert(errors.end(), workflowErrors.begin(), workflowErrors.end());
    workflowElem = workflowElem->GetNextElement();
  }

  return errors;
}

/////////////////////////////////////////////////
const PbrWorkflow *Pbr::Workflow(PbrWorkflowType _type) const
{
  auto it = this->dataPtr->workflows.find(_type);
  if (it != this->dataPtr->workflows.end())
    return &it->second;
  return nullptr;
}

/////////////////////////////////////////////////
void Pbr::SetWorkflow(PbrWorkflowType _type,
    const PbrWorkflow &_workflow)
{
  this->dataPtr->workflows[_type] = _workflow;
}
