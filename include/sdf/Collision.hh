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
#ifndef SDF_COLLISION_HH_
#define SDF_COLLISION_HH_

#include <memory>
#include <optional>
#include <string>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/ParserConfig.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declaration.
  class Geometry;
  class ParserConfig;
  class Surface;
  struct PoseRelativeToGraph;
  template <typename T> class ScopedGraph;

  /// \brief A collision element descibes the collision properties associated
  /// with a link. This can be different from the visual properties of a link.
  /// For example, simple collision models are often used to reduce
  /// computation time.
  class SDFORMAT_VISIBLE Collision
  {
    /// \brief Default constructor
    public: Collision();

    /// \brief Get the schema file name accessor
    public: static inline std::string_view SchemaFile();
    
    /// \brief Load the collision based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the collision based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \param[in] _config Parser configuration
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(sdf::ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Get the name of the collision.
    /// The name of the collision must be unique within the scope of a Link.
    /// \return Name of the collision.
    public: std::string Name() const;

    /// \brief Set the name of the collision.
    /// The name of the collision must be unique within the scope of a Link.
    /// \param[in] _name Name of the collision.
    public: void SetName(const std::string &_name);

    /// \brief Get the default density of a collision if its density is not
    /// specified.
    /// \return Default density.
    public: static double DensityDefault();

    /// \brief Get the density of the collision.
    /// \return Density of the collision.
    public: double Density() const;

    /// \brief Set the density of the collision.
    /// \param[in] _density Density of the collision.
    public: void SetDensity(double _density);

    /// \brief Get the ElementPtr to the <auto_inertia_params> element
    /// This element can be used as a parent element to hold user-defined
    /// params for the custom moment of inertia calculator.
    /// \return ElementPtr object for the <auto_inertia_params> element.
    public: sdf::ElementPtr AutoInertiaParams() const;

    /// \brief Function to set the auto inertia params using a
    /// sdf ElementPtr object
    /// \param[in] _autoInertiaParams ElementPtr to <auto_inertia_params>
    /// element
    public: void SetAutoInertiaParams(const sdf::ElementPtr _autoInertiaParams);

    /// \brief Get a pointer to the collisions's geometry.
    /// \return The collision's geometry.
    public: const Geometry *Geom() const;

    /// \brief Set the collision's geometry
    /// \param[in] _geom The geometry of the collision object
    public: void SetGeom(const Geometry &_geom);

    /// \brief Get a pointer to the collisions's surface parameters.
    /// \return The collision's surface parameters.
    public: const sdf::Surface *Surface() const;

    /// \brief Set the collision's surface parameters
    /// \param[in] _surface The surface parameters of the collision object
    public: void SetSurface(const sdf::Surface &_surface);

    /// \brief Get the pose of the collision object. This is the pose of the
    /// collison as specified in SDF
    /// (<collision><pose> ... </pose></collision>).
    /// \return The pose of the collision object.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the collision object.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The pose of the collision object.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get SemanticPose object of this object to aid in resolving
    /// poses.
    /// \return SemanticPose object for this link.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Calculate and return the MassMatrix for the collision
    /// \param[out] _errors A vector of Errors objects. Each errors contains an
    /// Error code and a message. An empty errors vector indicates no errors
    /// \param[out] _inertial An inertial object which will be set with the
    /// calculated inertial values
    /// \param[in] _config Custom parser configuration
    public: void CalculateInertial(sdf::Errors &_errors,
                                  gz::math::Inertiald &_inertial,
                                  const ParserConfig &_config);

    /// \brief Calculate and return the MassMatrix for the collision
    /// \param[out] _errors A vector of Errors objects. Each errors contains an
    /// Error code and a message. An empty errors vector indicates no errors
    /// \param[out] _inertial An inertial object which will be set with the
    /// calculated inertial values
    /// \param[in] _config Custom parser configuration
    /// \param[in] _density An optional density value to override the default
    /// collision density. This value is used instead of DefaultDensity()
    // if this collision's density has not been explicitly set.
    /// \param[in] _autoInertiaParams An ElementPtr to the auto_inertia_params
    /// element to be used if the auto_inertia_params have not been set in this
    /// collision.
    public: void CalculateInertial(
                    sdf::Errors &_errors,
                    gz::math::Inertiald &_inertial,
                    const ParserConfig &_config,
                    const std::optional<double> &_density,
                    sdf::ElementPtr _autoInertiaParams);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// collision.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated collision values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// collision.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated collision values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Give the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Give the scoped PoseRelativeToGraph to be used for resolving
    /// poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Allow Link::SetPoseRelativeToGraph to call SetXmlParentName
    /// and SetPoseRelativeToGraph, but Link::SetPoseRelativeToGraph is
    /// a private function, so we need to befriend the entire class.
    friend class Link;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
