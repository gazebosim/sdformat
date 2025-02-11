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
#include <array>
#include <filesystem>
#include <optional>
#include <string_view>

#include <gz/math/Inertial.hh>
#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/parser.hh"
#include "sdf/Mesh.hh"
#include "sdf/Element.hh"
#include "sdf/ParserConfig.hh"
#include "Utils.hh"

using namespace sdf;

/// Mesh Optimization method strings. These should match the data in
/// `enum class MeshOptimization` located in Mesh.hh, and the size
/// template parameter should match the number of elements as well.
constexpr std::array<const std::string_view, 3> kMeshOptimizationStrs =
{
  "",
  "convex_hull",
  "convex_decomposition"
};

// Private data class for ConvexDecomposition
class sdf::ConvexDecomposition::Implementation
{
  /// \brief Maximum number of convex hulls to generate.
  public: unsigned int maxConvexHulls{16u};

  /// \brief Voxel resolution. Applicable only to voxel based methods for
  /// representing the mesh before decomposition
  public: unsigned int voxelResolution{200000u};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf = nullptr;
};

// Private data class for Mesh
class sdf::Mesh::Implementation
{
  /// \brief Mesh optimization method
  public: MeshOptimization optimization = MeshOptimization::NONE;

  /// \brief Optional convex decomposition.
  public: std::optional<sdf::ConvexDecomposition> convexDecomposition;

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
ConvexDecomposition::ConvexDecomposition()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors ConvexDecomposition::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load convex decomposition, "
        "but the provided SDF element is null."});
    return errors;
  }

  // We need a convex_decomposition element
  if (_sdf->GetName() != "convex_decomposition")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load convex decomposition, but the provided SDF "
        "element is not <convex_decomposition>."});
    return errors;
  }

  this->dataPtr->maxConvexHulls = _sdf->Get<unsigned int>(
      errors, "max_convex_hulls",
      this->dataPtr->maxConvexHulls).first;

  this->dataPtr->voxelResolution = _sdf->Get<unsigned int>(
      errors, "voxel_resolution",
      this->dataPtr->voxelResolution).first;

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr ConvexDecomposition::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
unsigned int ConvexDecomposition::MaxConvexHulls() const
{
  return this->dataPtr->maxConvexHulls;
}

/////////////////////////////////////////////////
void ConvexDecomposition::SetMaxConvexHulls(unsigned int _maxConvexHulls)
{
  this->dataPtr->maxConvexHulls = _maxConvexHulls;
}

/////////////////////////////////////////////////
unsigned int ConvexDecomposition::VoxelResolution() const
{
  return this->dataPtr->voxelResolution;
}

/////////////////////////////////////////////////
void ConvexDecomposition::SetVoxelResolution(unsigned int _voxelResolution)
{
  this->dataPtr->voxelResolution = _voxelResolution;
}

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

  // Optimization
  if (_sdf->HasAttribute("optimization"))
  {
    this->SetOptimization(_sdf->Get<std::string>("optimization", "").first);
  }

  if (_sdf->HasElement("convex_decomposition"))
  {
    this->dataPtr->convexDecomposition.emplace();
    Errors err = this->dataPtr->convexDecomposition->Load(
        _sdf->GetElement("convex_decomposition", errors));
    errors.insert(errors.end(), err.begin(), err.end());
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
MeshOptimization Mesh::Optimization() const
{
  return this->dataPtr->optimization;
}

//////////////////////////////////////////////////
std::string Mesh::OptimizationStr() const
{
  size_t index = static_cast<int>(this->dataPtr->optimization);
  if (index < kMeshOptimizationStrs.size())
    return std::string(kMeshOptimizationStrs[index]);
  return "";
}

//////////////////////////////////////////////////
bool Mesh::SetOptimization(const std::string &_optimizationStr)
{
  for (size_t i = 0; i < kMeshOptimizationStrs.size(); ++i)
  {
    if (_optimizationStr == kMeshOptimizationStrs[i])
    {
      this->dataPtr->optimization = static_cast<MeshOptimization>(i);
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
void Mesh::SetOptimization(MeshOptimization _optimization)
{
  this->dataPtr->optimization = _optimization;
}

//////////////////////////////////////////////////
const sdf::ConvexDecomposition *Mesh::ConvexDecomposition() const
{
  if (this->dataPtr->convexDecomposition.has_value())
    return &this->dataPtr->convexDecomposition.value();
  return nullptr;
}

//////////////////////////////////////////////////
 void Mesh::SetConvexDecomposition(
    const sdf::ConvexDecomposition &_convexDecomposition)
{
  this->dataPtr->convexDecomposition = _convexDecomposition;
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

  auto defaultInertial = gz::math::Inertiald(
      gz::math::MassMatrix3d(1, gz::math::Vector3d::One,
                             gz::math::Vector3d::Zero),
                             gz::math::Pose3d::Zero);
  if (!customCalculator)
  {
    Error err(
        sdf::ErrorCode::WARNING,
        "Custom moment of inertia calculator for meshes not set via "
        "sdf::ParserConfig::RegisterCustomInertiaCalc, using default "
        "inertial values.");
    enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, _errors);
    return defaultInertial;
  }

  sdf::CustomInertiaCalcProperties calcInterface = CustomInertiaCalcProperties(
    _density, *this, _autoInertiaParams);

  std::optional<gz::math::Inertiald> inertial =
      customCalculator(_errors, calcInterface);
  if (!inertial)
  {
    Error err(
        sdf::ErrorCode::WARNING,
        "Custom moment of inertia calculator for meshes produced invalid inertia, "
        "using default inertial values.");
    enforceConfigurablePolicyCondition(
          _config.WarningsPolicy(), err, _errors);
    return defaultInertial;
  }
  return inertial;
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

  // Optimization
  elem->GetAttribute("optimization")->Set<std::string>(
      this->OptimizationStr());

  if (this->dataPtr->convexDecomposition.has_value())
  {
    sdf::ElementPtr convexDecomp = elem->GetElement("convex_decomposition",
        _errors);
    convexDecomp->GetElement("max_convex_hulls")->Set(
        this->dataPtr->convexDecomposition->MaxConvexHulls());
    convexDecomp->GetElement("voxel_resolution")->Set(
        this->dataPtr->convexDecomposition->VoxelResolution());
  }

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
