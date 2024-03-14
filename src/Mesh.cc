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

#include <gz/math/Inertial.hh>
#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/parser.hh"
#include "sdf/Mesh.hh"
#include "sdf/Element.hh"
#include "sdf/ParserConfig.hh"
#include "Utils.hh"

using namespace sdf;

/// Mesh Simplification method strings. These should match the data in
/// `enum class MeshSimplification` located in Mesh.hh, and the size
/// template parameter should match the number of elements as well.
constexpr std::array<const std::string_view, 3> kMeshSimplificationStrs =
{
  "",
  "convex_hull",
  "convex_decomposition"
};

// Private data class
class sdf::Mesh::Implementation
{
  /// \brief Mesh simplification method
  public: MeshSimplification simplification = MeshSimplification::NONE;

  /// \brief The mesh's URI.
  public: std::string uri = "";

  /// \brief The path to the file where this mesh was defined.
  public: std::string filePath = "";

  /// \brief The mesh's scale.
  public: gz::math::Vector3d scale {1, 1, 1};

  /// \brief The name of the submesh.
  public: std::string submesh = "";

  /// \brief True to center the submesh.
  public: bool centerSubmesh = false;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf = nullptr;
};

/////////////////////////////////////////////////
Mesh::Mesh()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Mesh::Load(ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Mesh::Load(ElementPtr _sdf, const ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a mesh, but the provided SDF element is null."});
    return errors;
  }

  this->dataPtr->filePath = _sdf->FilePath();

  // We need a mesh element
  if (_sdf->GetName() != "mesh")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a mesh geometry, but the provided SDF "
        "element is not a <mesh>."});
    return errors;
  }

  // Simplify
  if (_sdf->HasAttribute("simplification"))
  {
    this->SetSimplification(_sdf->Get<std::string>("simplification", "").first);
  }

  if (_sdf->HasElement("uri"))
  {
    std::unordered_set<std::string> paths;
    if (!this->dataPtr->filePath.empty())
    {
      paths.insert(std::filesystem::path(
          this->dataPtr->filePath).parent_path().string());
    }
    this->dataPtr->uri = resolveURI(
      _sdf->Get<std::string>(errors, "uri", "").first,
      _config, errors, paths);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Mesh geometry is missing a <uri> child element."});
  }

  if (_sdf->HasElement("submesh"))
  {
    sdf::ElementPtr subMesh = _sdf->GetElement("submesh", errors);

    std::pair<std::string, bool> subMeshNamePair =
      subMesh->Get<std::string>(errors, "name", this->dataPtr->submesh);

    if (subMeshNamePair.first == "__default__" ||
        subMeshNamePair.first.empty() || !subMeshNamePair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <submesh> element is missing a child <name> element, or the "
          "<name> element is empty."});
    }
    else
    {
      this->dataPtr->submesh = subMeshNamePair.first;
    }

    this->dataPtr->centerSubmesh = subMesh->Get<bool>(errors, "center",
        this->dataPtr->centerSubmesh).first;
  }

  this->dataPtr->scale = _sdf->Get<gz::math::Vector3d>(errors, "scale",
      this->dataPtr->scale).first;

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Mesh::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
MeshSimplification Mesh::Simplification() const
{
  return this->dataPtr->simplification;
}

//////////////////////////////////////////////////
std::string Mesh::SimplificationStr() const
{
  size_t index = static_cast<int>(this->dataPtr->simplification);
  if (index < kMeshSimplificationStrs.size())
    return std::string(kMeshSimplificationStrs[index]);
  return "";
}

//////////////////////////////////////////////////
bool Mesh::SetSimplification(const std::string &_simplificationStr)
{
  for (size_t i = 0; i < kMeshSimplificationStrs.size(); ++i)
  {
    if (_simplificationStr == kMeshSimplificationStrs[i])
    {
      this->dataPtr->simplification = static_cast<MeshSimplification>(i);
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
void Mesh::SetSimplification(MeshSimplification _simplification)
{
  this->dataPtr->simplification = _simplification;
}

//////////////////////////////////////////////////
std::string Mesh::Uri() const
{
  return this->dataPtr->uri;
}

//////////////////////////////////////////////////
void Mesh::SetUri(const std::string &_uri)
{
  this->dataPtr->uri = _uri;
}

//////////////////////////////////////////////////
const std::string &Mesh::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Mesh::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

//////////////////////////////////////////////////
gz::math::Vector3d Mesh::Scale() const
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
void Mesh::SetScale(const gz::math::Vector3d &_scale)
{
  this->dataPtr->scale = _scale;
}

//////////////////////////////////////////////////
std::string Mesh::Submesh() const
{
  return this->dataPtr->submesh;
}

//////////////////////////////////////////////////
void Mesh::SetSubmesh(const std::string &_submesh)
{
  this->dataPtr->submesh = _submesh;
}

//////////////////////////////////////////////////
bool Mesh::CenterSubmesh() const
{
  return this->dataPtr->centerSubmesh;
}

//////////////////////////////////////////////////
void Mesh::SetCenterSubmesh(const bool _center)
{
  this->dataPtr->centerSubmesh = _center;
}

//////////////////////////////////////////////////
std::optional<gz::math::Inertiald> Mesh::CalculateInertial(sdf::Errors &_errors,
  double _density, const sdf::ElementPtr _autoInertiaParams,
  const ParserConfig &_config)
{
  const auto &customCalculator = _config.CustomInertiaCalc();

  if (!customCalculator)
  {
    Error err(
        sdf::ErrorCode::WARNING,
        "Custom moment of inertia calculator for meshes not set via "
        "sdf::ParserConfig::RegisterCustomInertiaCalc, using default "
        "inertial values.");
    enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, _errors);

    using namespace gz::math;
    return Inertiald(
        MassMatrix3d(1, Vector3d::One, Vector3d::Zero),
        Pose3d::Zero);
  }

  sdf::CustomInertiaCalcProperties calcInterface = CustomInertiaCalcProperties(
    _density, *this, _autoInertiaParams);

  return customCalculator(_errors, calcInterface);
}

/////////////////////////////////////////////////
sdf::ElementPtr Mesh::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Mesh::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("mesh_shape.sdf", elem);

  // Simplification
  elem->GetAttribute("simplification")->Set<std::string>(
      this->SimplificationStr());

  // Uri
  sdf::ElementPtr uriElem = elem->GetElement("uri", _errors);
  uriElem->Set(_errors, this->Uri());

  // Submesh
  if (!this->dataPtr->submesh.empty())
  {
    sdf::ElementPtr subMeshElem = elem->GetElement("submesh", _errors);

    sdf::ElementPtr subMeshNameElem = subMeshElem->GetElement("name", _errors);
    subMeshNameElem->Set(_errors, this->dataPtr->submesh);

    sdf::ElementPtr subMeshCenterElem = subMeshElem->GetElement(
        "center", _errors);
    subMeshCenterElem->Set(_errors, this->dataPtr->centerSubmesh);
  }

  // Scale
  sdf::ElementPtr scaleElem = elem->GetElement("scale", _errors);
  scaleElem->Set(_errors, this->Scale());

  return elem;
}
