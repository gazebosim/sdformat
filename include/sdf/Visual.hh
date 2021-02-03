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
#ifndef SDF_VISUAL_HH_
#define SDF_VISUAL_HH_

#include <memory>
#include <string>
#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>
#include "sdf/Box.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Element.hh"
#include "sdf/Material.hh"
#include "sdf/Plane.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Sphere.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class Geometry;
  struct PoseRelativeToGraph;
  template <typename T> class ScopedGraph;

  class SDFORMAT_VISIBLE Visual
  {
    /// \brief Default constructor
    public: Visual();

    /// \brief Load the visual based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the visual.
    /// The name of the visual must be unique within the scope of a Link.
    /// \return Name of the visual.
    public: std::string Name() const;

    /// \brief Set the name of the visual.
    /// The name of the visual must be unique within the scope of a Link.
    /// \param[in] _name Name of the visual.
    public: void SetName(const std::string &_name);

    /// \brief Get whether the visual casts shadows
    /// \return True if the visual casts shadows, false otherwise
    public: bool CastShadows() const;

    /// \brief Set whether the visual casts shadows
    /// \param[in] _castShadows True to cast shadows, false to not cast shadows
    public: void SetCastShadows(bool _castShadows);

    /// \brief Get the transparency value of the visual
    /// \return Transparency value
    public: float Transparency() const;

    /// \brief Set the transparency value for the visual
    /// \param[in] _transparency Transparency value between 0 and 1
    public: void SetTransparency(float _transparency);

    /// \brief Get a pointer to the visual's geometry.
    /// \return The visual's geometry.
    public: const Geometry *Geom() const;

    /// \brief Set the visual's geometry
    /// \param[in] _geom The geometry of the visual object
    public: void SetGeom(const Geometry &_geom);

    /// \brief Get the pose of the visual object. This is the pose of the
    /// visual as specified in SDF
    /// (<visual><pose> ... </pose></visual>).
    /// \return The pose of the visual object.
    public: const ignition::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the visual object.
    /// \sa const ignition::math::Pose3d &RawPose() const
    /// \param[in] _pose The pose of the visual object.
    public: void SetRawPose(const ignition::math::Pose3d &_pose);

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

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get a pointer to the visual's material properties. This can
    /// be a nullptr if material properties have not been set.
    /// \return Pointer to the visual's material properties. Nullptr
    /// indicates that material properties have not been set.
    public: const sdf::Material *Material() const;

    /// \brief Set the visual's material
    /// \param[in] _material The material of the visual object
    public: void SetMaterial(const sdf::Material &_material);

    /// \brief Get the visibility flags of a visual
    /// \return visibility flags
    public: uint32_t VisibilityFlags() const;

    /// \brief Set the visibility flags of a visual
    /// \param[in] _flags visibility flags
    public: void SetVisibilityFlags(uint32_t _flags);

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
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
