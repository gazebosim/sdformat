#include <map>
#include <string>
#include "sdf/SDFImpl.hh"
#include "sdf/Element.hh"

#ifndef SDF_MODEL_HH_
#define SDF_MODEL_HH_

namespace sdf
{
  class Joint;
  class Light;
  class Link;
  class Model;
  class World;

  class SDFORMAT_VISIBLE Root
  {
    public: Root() = default;

    public: bool Load(const std::string &_filename);
    public: bool Load(const sdf::SDFPtr _sdf);
    public: bool Load(ElementPtr _sdf);

    public: std::string Version() const;

    public: void Print(const std::string &_prefix = "") const;

    private: std::string version = "";
    private: std::map<std::string, World> worlds;
    private: std::map<std::string, Model> models;
    private: std::map<std::string, Light> lights;
  };

  class SDFORMAT_VISIBLE World
  {
    public: World() = default;

    public: bool Load(ElementPtr _sdf);

    public: void Print(const std::string &_prefix = "") const;

    public: std::string Name() const;

    private: std::string name = "";
    private: std::map<std::string, Model> models;
    private: std::map<std::string, Light> lights;
  };

  class SDFORMAT_VISIBLE Light
  {
    public: Light() = default;
    public: bool Load(sdf::ElementPtr _sdf);

    public: std::string Name() const;
    public: std::string Type() const;
    public: bool CastShadows() const;
    public: ignition::math::Color Diffuse() const;
    public: ignition::math::Color Specular() const;

    public: void Print(const std::string &_prefix = "") const;

    private: std::string name = "";
    private: std::string type = "";
    private: bool castShadows = false;
    private: ignition::math::Color diffuse;
    private: ignition::math::Color specular;

    protected: ignition::math::Pose3d pose;
    protected: std::string frame;
  };

  class SDFORMAT_VISIBLE Model
  {
    public: Model() = default;
    public: bool Load(sdf::ElementPtr _sdf);

    public: void Print(const std::string &_prefix = "") const;

    /// \brief Return the name of the model. A model name must be unique.
    /// \return Name of the model
    public: std::string Name() const;

    /// \brief Get the number of links.
    /// \return Number of links in the model.
    public: uint64_t LinkCount() const;

    /// \brief Find a link by name.
    /// \param[in] _name Name of the link to retrieve.
    /// \return Pointer to the link, nullptr if the link does not exist.
    public: const sdf::Link *FindLink(const std::string &_name) const;

    /// \brief Get the number of joints.
    /// \return Number of joints in the model.
    public: uint64_t JointCount() const;

    /// \brief Find a joint by name.
    /// \param[in] _name Name of the joint to retrieve.
    /// \return Pointer to the joint, nullptr if the joint does not exist.
    public: const sdf::Joint *FindJoint(const std::string &_name) const;

    /// \brief Get the number of nested models.
    /// \return Number of nested models in the model.
    public: uint64_t ModelCount() const;

    /// \brief Find a nested model by name.
    /// \param[in] _name Name of the nested model to retrieve.
    /// \return Pointer to the nested model,
    /// nullptr if the model does not exist.
    public: const sdf::Model *FindModel(const std::string &_name) const;

    private: std::string name = "";
    private: std::map<std::string, sdf::Link> links;
    private: std::map<std::string, sdf::Joint> joints;
    private: std::map<std::string, sdf::Model> models;
    private: ignition::math::Pose3d pose;
    private: std::string frame;
    private: bool isStatic;
    private: bool selfCollide;
    private: bool autoDisable;
    private: bool enableWind;
  };

  class SDFORMAT_VISIBLE Link
  {
    public: Link() = default;

    public: bool Load(sdf::ElementPtr _sdf);

    public: void Print(const std::string &_prefix = "") const;

    public: std::string Name() const;

    private: std::string name = "";
    protected: ignition::math::Pose3d pose;
    protected: std::string frame;
  };

  class SDFORMAT_VISIBLE Joint
  {
    public: Joint() = default;

    public: bool Load(sdf::ElementPtr _sdf);

    public: void Print(const std::string &_prefix = "") const;

    public: std::string Name() const;

    private: std::string name = "";
    protected: ignition::math::Pose3d pose;
    protected: std::string frame;
  };
}

#endif
