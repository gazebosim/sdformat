#ifndef SDF_DOM_MODEL_HH_
#define SDF_DOM_MODEL_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  class Link;
  class Joint;

  // Forward declare private data class.
  class ModelPrivate;

  class SDFORMAT_VISIBLE Model
  {
    /// \brief Default constructor
    public: Model();

    /// \brief Destructor
    public: ~Model();

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

    /// \brief Private data pointer.
    private: ModelPrivate *dataPtr;
  };
}
#endif

