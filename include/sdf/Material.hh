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
#ifndef SDF_MATERIAL_HH_
#define SDF_MATERIAL_HH_

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class MaterialPrivate;

  /// \brief This class contains visual material properties.
  class SDFORMAT_VISIBLE Material
  {
    /// \brief Default constructor
    public: Material();

    /// \brief Move constructor
    /// \param[in] _material Material to move.
    public: Material(Material &&_material);

    /// \brief Destructor
    public: ~Material();

    /// \brief Load the material based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the ambient color. The ambient color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Ambient color.
    public: ignition::math::Color Ambient() const;

    /// \brief Set the ambient color. The ambient color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Ambient color.
    public: void SetAmbient(const ignition::math::Color &_color) const;

    /// \brief Get the diffuse color. The diffuse color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Diffuse color.
    public: ignition::math::Color Diffuse() const;

    /// \brief Set the diffuse color. The diffuse color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Diffuse color.
    public: void SetDiffuse(const ignition::math::Color &_color) const;

    /// \brief Get the specular color. The specular color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Specular color.
    public: ignition::math::Color Specular() const;

    /// \brief Set the specular color. The specular color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Specular color.
    public: void SetSpecular(const ignition::math::Color &_color) const;

    /// \brief Get the emissive color. The emissive color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \return Emissive color.
    public: ignition::math::Color Emissive() const;

    /// \brief Set the emissive color. The emissive color is
    /// specified by a set of three numbers representing red/green/blue,
    /// each in the range of [0,1].
    /// \param[in] _color Emissive color.
    public: void SetEmissive(const ignition::math::Color &_color) const;

    /// \brief Get whether dynamic lighting is enabled. The default
    /// value is true.
    /// \return False if dynamic lighting should be disabled.
    public: bool Lighting() const;

    /// \brief Set whether dynamic lighting is enabled.
    /// \param[in] _lighting False disables dynamic lighting.
    public: void SetLighting(const bool _lighting);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: MaterialPrivate *dataPtr = nullptr;
  };
}
#endif
