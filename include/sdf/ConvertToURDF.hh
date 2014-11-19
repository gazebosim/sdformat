/*
 * Copyright 2014 Open Source Robotics Foundation
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
#ifndef _SDF_CONVERT_TO_URDF_
#define _SDF_CONVERT_TO_URDF_

#include <sstream>
#include <string>
#include <map>
#include <sdf/sdf.hh>
#include "sdf/system_util.hh"

namespace sdf
{
  /// \brief Functions used to convert an SDF model to a URDF robot. This
  /// process is loosy since URDF does not contain all the XML elements
  /// need to represent a robot. Please refer to the SDF documentation for
  /// more information: http://sdformat.org
  class SDFORMAT_VISIBLE ConvertToURDF
  {
    /// \brief Convert an SDF file to URDF.
    /// \param[in] _file File name of the SDF file.
    /// \param[out] _result Converted URDF string.
    /// \return True on success.
    public: static bool ConvertFile(const std::string &_file,
                std::string &_result);

    /// \brief Convert an SDF string to URDF.
    /// \param[in] _sdfString String containing an SDF model.
    /// \param[out] _result Converted URDF string.
    /// \return True on success.
    public: static bool ConvertString(const std::string &_sdfString,
                std::string &_result);

    /// \brief Convert an SDF model to URDF.
    /// \param[in] _elem SDF element to convert
    /// \param[out] _result Converted URDF stream.
    /// \return True on success.
    private: static bool ConvertModel(sdf::ElementPtr _elem,
                std::ostringstream &_result);

    /// \brief Convert an SDF pose.
    /// \param[in] _elem SDF element to convert.
    /// \param[in] _prefix Prefix (spaces) used for formatting.
    /// \param[in] _offset Pose value to add to the pose contained in _elem.
    /// \param[out] _result Stream that contains the conversion result.
    /// \return True on success.
    private: static bool ConvertPose(sdf::ElementPtr _elem,
                const std::string &_prefix,
                std::ostringstream &_result,
                const sdf::Pose &_offset = sdf::Pose(0, 0, 0, 0, 0, 0));

    /// \brief Convert an SDF geomentry
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertGeometry(sdf::ElementPtr _elem,
                const std::string &_prefix, std::ostringstream &_result);

    /// \brief Convert an SDF collision
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[in] _offset Pose offset for the element.
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertCollision(sdf::ElementPtr _elem,
                const std::string &_prefix, const sdf::Pose &_offset,
                std::ostringstream &_result);

    /// \brief Convert an SDF visual
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[in] _offset Pose offset for the element.
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertVisual(sdf::ElementPtr _elem,
                const std::string &_prefix, const sdf::Pose &_offset,
                std::ostringstream &_result);

    /// \brief Convert an SDF inertia
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[in] _offset Pose offset for the element.
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertInertia(sdf::ElementPtr _elem,
                const std::string &_prefix, const sdf::Pose &_offset,
                std::ostringstream &_result);

    /// \brief Convert an SDF link
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[out] _linkPoses Map to store link poses. These are used
    /// when converting joints.
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertLink(sdf::ElementPtr _elem,
                const std::string &_prefix,
                std::map<std::string, sdf::Pose> &_linkPoses,
                std::ostringstream &_result);

    /// \brief Convert an SDF camera
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertCamera(sdf::ElementPtr _elem,
                 const std::string &_prefix, std::ostringstream &_result);

    /// \brief Convert an SDF ray
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertRay(sdf::ElementPtr _elem,
                const std::string &_prefix, std::ostringstream &_result);

    /// \brief Convert an SDF sensor.
    /// \param[in] _elem SDF element to convert.
    /// \param[in] _prefix Prefix (spaces) used for formatting.
    /// \param[in] _parent Name of the link that contains the sensor.
    /// \param[out] _result Stream that contains the conversion result.
    /// \return True on success.
    private: static bool ConvertSensor(sdf::ElementPtr _elem,
                const std::string &_prefix, const std::string &_parent,
                std::ostringstream &_result);

    /// \brief Convert an SDF joint
    /// \param[in] _elem SDF element to convert
    /// \param[in] _prefix Prefix (spaces) used for formatting
    /// \param[out] _linkPoses Map of link poses. These should be obtained
    /// by calling ConvertLink.
    /// \param[out] _result Stream that contains the conversion result
    /// \return True on success
    private: static bool ConvertJoint(sdf::ElementPtr _elem,
                const std::string &_prefix,
                std::map<std::string, sdf::Pose> &_linkPoses,
                std::ostringstream &_result);
  };
}

#endif
