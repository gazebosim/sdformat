/*
 * Copyright 2021 Open Source Robotics Foundation
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
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sdf/Error.hh"
#include "sdf/parser.hh"
#include "sdf/ParticleEmitter.hh"
#include "sdf/Types.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

/// Particle emitter type strings. These should match the data in
/// `enum class ParticleEmitterType` located in ParticleEmitter.hh.
const std::vector<std::string> emitterTypeStrs =
{
  "point",
  "box",
  "cylinder",
  "ellipsoid",
};

class sdf::ParticleEmitter::Implementation
{
  /// \brief Name of the particle emitter.
  public: std::string name = "";

  /// \brief Type of the particle emitter.
  public: ParticleEmitterType type = ParticleEmitterType::POINT;

  /// \brief True if the emitting should produce particles.
  public: bool emitting = true;

  /// \brief The number of seconds the emitter is active.
  /// A value of 0 means infinite duration.
  public: double duration = 0;

  /// \brief The number of seconds each particle will 'live' for.
  public: double lifetime = 5;

  /// \brief The number of particles per second that should be emitted.
  public: double rate = 10;

  /// \brief The amount by which to scale the particles in both x and y
  /// direction per second.
  public: double scaleRate = 0;

  /// \brief The minimum velocity for each particle (m/s).
  public: double minVelocity = 1;

  /// \brief The maximum velocity for each particle (m/s).
  public: double maxVelocity = 1;

  /// \brief The size of the emitter where the particles are sampled.
  public: ignition::math::Vector3d size = ignition::math::Vector3d::One;

  /// \brief The size of a particle.
  public: ignition::math::Vector3d particleSize = ignition::math::Vector3d::One;

  /// \brief The starting color for all particle emitted.
  public: ignition::math::Color colorStart = ignition::math::Color::White;

  /// \brief The ending color for all particle emitted.
  public: ignition::math::Color colorEnd = ignition::math::Color::White;

  /// \brief The color range image
  public: std::string colorRangeImage = "";

  /// \brief Topic used to update particle emitter properties at runtime.
  public: std::string topic = "";

  /// \brief Particle scatter ratio. This is used to determine the ratio of
  /// particles that will be detected by sensors. Increasing the ratio
  /// means there is a higher chance of particles reflecting and interfering
  /// with depth sensing, making the emitter appear more dense. Decreasing
  /// the ratio decreases the chance of particles reflecting and interfering
  /// with depth sensing, making it appear less dense.
  public: float scatterRatio = 0.65f;

  /// \brief Pose of the emitter
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName = "";

  /// \brief Pointer to the emitter's material properties.
  public: std::optional<sdf::Material> material;

  /// \brief The path to the file where this emitter was defined.
  public: std::string filePath = "";

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
ParticleEmitter::ParticleEmitter()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors ParticleEmitter::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  this->dataPtr->filePath = _sdf->FilePath();

  // Check that the provided SDF element is a <particle_emitter>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "particle_emitter")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a particle emitter, but the provided SDF element "
        "is not a <particle_emitter>."});
    return errors;
  }

  // Read the emitter's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A link name is required, but the name is not set."});
  }

  // Check that the emitter's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
        "The supplied particle emitter name [" + this->dataPtr->name +
        "] is reserved."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  if (!this->SetType(_sdf->Get<std::string>("type",
          this->TypeStr()).first))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Attempting to load a particle emitter, but the provided "
        "particle emitter type is missing or invalid."});
    return errors;
  }

  this->dataPtr->emitting = _sdf->Get<bool>("emitting",
      this->dataPtr->emitting).first;

  this->dataPtr->duration = _sdf->Get<double>("duration",
      this->dataPtr->duration).first;

  this->dataPtr->lifetime = _sdf->Get<double>("lifetime",
      this->dataPtr->lifetime).first;

  this->dataPtr->rate = _sdf->Get<double>("rate",
      this->dataPtr->rate).first;

  this->dataPtr->scaleRate = _sdf->Get<double>("scale_rate",
      this->dataPtr->scaleRate).first;

  this->dataPtr->minVelocity = _sdf->Get<double>("min_velocity",
      this->dataPtr->minVelocity).first;

  this->dataPtr->maxVelocity = _sdf->Get<double>("max_velocity",
      this->dataPtr->maxVelocity).first;

  this->dataPtr->size = _sdf->Get<ignition::math::Vector3d>("size",
      this->dataPtr->size).first;

  this->dataPtr->particleSize = _sdf->Get<ignition::math::Vector3d>(
      "particle_size", this->dataPtr->particleSize).first;

  this->dataPtr->colorStart = _sdf->Get<ignition::math::Color>(
      "color_start", this->dataPtr->colorStart).first;

  this->dataPtr->colorEnd = _sdf->Get<ignition::math::Color>(
      "color_end", this->dataPtr->colorEnd).first;

  this->dataPtr->colorRangeImage = _sdf->Get<std::string>(
      "color_range_image", this->dataPtr->colorRangeImage).first;

  this->dataPtr->topic = _sdf->Get<std::string>(
      "topic", this->dataPtr->topic).first;

  this->dataPtr->scatterRatio = _sdf->Get<float>(
      "particle_scatter_ratio", this->dataPtr->scatterRatio).first;

  if (_sdf->HasElement("material"))
  {
    this->dataPtr->material.emplace();
    Errors err = this->dataPtr->material->Load(_sdf->GetElement("material"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  return errors;
}

/////////////////////////////////////////////////
std::string ParticleEmitter::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
ParticleEmitterType ParticleEmitter::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetType(const ParticleEmitterType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
bool ParticleEmitter::SetType(const std::string &_typeStr)
{
  for (size_t i = 0; i < emitterTypeStrs.size(); ++i)
  {
    if (_typeStr == emitterTypeStrs[i])
    {
      this->dataPtr->type = static_cast<ParticleEmitterType>(i);
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
std::string ParticleEmitter::TypeStr() const
{
  size_t index = static_cast<int>(this->dataPtr->type);
  if (index < emitterTypeStrs.size())
    return emitterTypeStrs[index];
  return "point";
}

/////////////////////////////////////////////////
bool ParticleEmitter::Emitting() const
{
  return this->dataPtr->emitting;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetEmitting(bool _emitting)
{
  this->dataPtr->emitting = _emitting;
}

/////////////////////////////////////////////////
double ParticleEmitter::Duration() const
{
  return this->dataPtr->duration;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetDuration(double _duration)
{
  this->dataPtr->duration = _duration;
}

/////////////////////////////////////////////////
double ParticleEmitter::Lifetime() const
{
  return this->dataPtr->lifetime;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetLifetime(double _lifetime)
{
  this->dataPtr->lifetime = std::max(_lifetime, ignition::math::MIN_D);
}

/////////////////////////////////////////////////
double ParticleEmitter::Rate() const
{
  return this->dataPtr->rate;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetRate(double _rate)
{
  this->dataPtr->rate = std::max(_rate, 0.0);
}

/////////////////////////////////////////////////
double ParticleEmitter::ScaleRate() const
{
  return this->dataPtr->scaleRate;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetScaleRate(double _scaleRate)
{
  this->dataPtr->scaleRate = std::max(_scaleRate, 0.0);
}

/////////////////////////////////////////////////
double ParticleEmitter::MinVelocity() const
{
  return this->dataPtr->minVelocity;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetMinVelocity(double _vel)
{
  this->dataPtr->minVelocity = std::max(_vel, 0.0);
}

/////////////////////////////////////////////////
double ParticleEmitter::MaxVelocity() const
{
  return this->dataPtr->maxVelocity;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetMaxVelocity(double _vel)
{
  this->dataPtr->maxVelocity = std::max(_vel, 0.0);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ParticleEmitter::Size() const
{
  return this->dataPtr->size;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetSize(const ignition::math::Vector3d &_size)
{
  this->dataPtr->size = _size;
  this->dataPtr->size.Max(ignition::math::Vector3d::Zero);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ParticleEmitter::ParticleSize() const
{
  return this->dataPtr->particleSize;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetParticleSize(const ignition::math::Vector3d &_size)
{
  this->dataPtr->particleSize = _size;
  this->dataPtr->particleSize.Max(ignition::math::Vector3d::Zero);
}

/////////////////////////////////////////////////
ignition::math::Color ParticleEmitter::ColorStart() const
{
  return this->dataPtr->colorStart;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetColorStart(const ignition::math::Color &_colorStart)
{
  this->dataPtr->colorStart = _colorStart;
}

/////////////////////////////////////////////////
ignition::math::Color ParticleEmitter::ColorEnd() const
{
  return this->dataPtr->colorEnd;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetColorEnd(const ignition::math::Color &_colorEnd)
{
  this->dataPtr->colorEnd = _colorEnd;
}

/////////////////////////////////////////////////
std::string ParticleEmitter::ColorRangeImage() const
{
  return this->dataPtr->colorRangeImage;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetColorRangeImage(const std::string &_image)
{
  this->dataPtr->colorRangeImage = _image;
}

/////////////////////////////////////////////////
std::string ParticleEmitter::Topic() const
{
  return this->dataPtr->topic;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetTopic(const std::string &_topic)
{
  this->dataPtr->topic = _topic;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetScatterRatio(float _ratio)
{
  this->dataPtr->scatterRatio = _ratio;
}

/////////////////////////////////////////////////
float ParticleEmitter::ScatterRatio() const
{
  return this->dataPtr->scatterRatio;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &ParticleEmitter::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
const std::string &ParticleEmitter::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
sdf::SemanticPose ParticleEmitter::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
sdf::ElementPtr ParticleEmitter::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const sdf::Material *ParticleEmitter::Material() const
{
  return optionalToPointer(this->dataPtr->material);
}

/////////////////////////////////////////////////
void ParticleEmitter::SetMaterial(const sdf::Material &_material)
{
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
const std::string &ParticleEmitter::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void ParticleEmitter::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void ParticleEmitter::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
sdf::ElementPtr ParticleEmitter::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("particle_emitter.sdf", elem);

  // Set pose
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<ignition::math::Pose3d>(this->RawPose());

  elem->GetAttribute("name")->Set(this->Name());
  elem->GetAttribute("type")->Set(this->TypeStr());
  elem->GetElement("emitting")->Set(this->Emitting());
  elem->GetElement("duration")->Set(this->Duration());
  elem->GetElement("size")->Set(this->Size());
  elem->GetElement("particle_size")->Set(this->ParticleSize());
  elem->GetElement("lifetime")->Set(this->Lifetime());
  elem->GetElement("rate")->Set(this->Rate());
  elem->GetElement("min_velocity")->Set(this->MinVelocity());
  elem->GetElement("max_velocity")->Set(this->MaxVelocity());
  elem->GetElement("scale_rate")->Set(this->ScaleRate());
  elem->GetElement("color_start")->Set(this->ColorStart());
  elem->GetElement("color_end")->Set(this->ColorEnd());
  elem->GetElement("color_range_image")->Set(this->ColorRangeImage());
  elem->GetElement("topic")->Set(this->Topic());
  elem->GetElement("particle_scatter_ratio")->Set(this->ScatterRatio());

  if (this->dataPtr->material)
  {
    elem->InsertElement(this->dataPtr->material->ToElement(), true);
  }

  return elem;
}


