/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ignition/utilities/ExtraTestMacros.hh>

#include "sdf/parser.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/sdf_config.h"
#include "test_config.h"

#ifdef _WIN32
  #define popen  _popen
  #define pclose _pclose
#endif

static const std::string g_sdfVersion(" --force-version " +
  std::string(SDF_VERSION_FULL));
static const std::string g_ignCommand(std::string(IGN_PATH) + "/ign");

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(check, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  // Check a good SDF file
  {
    std::string path = pathBase +"/box_plane_low_friction_test.world";

    // Check box_plane_low_friction_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check a bad SDF file
  {
    std::string path = pathBase +"/box_bad_test.world";

    // Check box_bad_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Required attribute"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (world)
  // that have duplicate names.
  {
    std::string path = pathBase +"/world_duplicate.sdf";

    // Check world_duplicate.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("World with name[default] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of different types (model, light)
  // that have duplicate names.
  {
    std::string path = pathBase +"/world_sibling_same_names.sdf";

    // Check world_sibling_same_names.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Non-unique names"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (link)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_duplicate_links.sdf";

    // Check model_duplicate_links.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("link with name[link] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of the same type (joint)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_duplicate_joints.sdf";

    // Check model_duplicate_joints.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("joint with name[joint] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of different types (link, joint)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_link_joint_same_name.sdf";

    // Check model_link_joint_same_name.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Non-unique names"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (collision)
  // that have duplicate names.
  {
    std::string path = pathBase +"/link_duplicate_sibling_collisions.sdf";

    // Check link_duplicate_sibling_collisions.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("collision with name[collision] "
                          "already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of the same type (visual)
  // that have duplicate names.
  {
    std::string path = pathBase +"/link_duplicate_sibling_visuals.sdf";

    // Check link_duplicate_sibling_visuals.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("visual with name[visual] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with cousin elements of the same type (collision)
  // that have duplicate names. This is a valid file.
  {
    std::string path = pathBase +"/link_duplicate_cousin_collisions.sdf";

    // Check link_duplicate_cousin_collisions.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with cousin elements of the same type (visual)
  // that have duplicate names. This is a valid file.
  {
    std::string path = pathBase +"/link_duplicate_cousin_visuals.sdf";

    // Check link_duplicate_cousin_visuals.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a joint with an invalid child link.
  {
    std::string path = pathBase +"/joint_invalid_child.sdf";

    // Check joint_invalid_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Child frame with name[invalid] specified by "
                          "joint with name[joint] not found in model with "
                          "name[joint_invalid_child]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with an invalid parent link.
  {
    std::string path = pathBase +"/joint_invalid_parent.sdf";

    // Check joint_invalid_parent.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("parent frame with name[invalid] specified by "
                          "joint with name[joint] not found in model with "
                          "name[joint_invalid_parent]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint that names itself as the child frame.
  {
    std::string path = pathBase +"/joint_invalid_self_child.sdf";

    // Check joint_invalid_self_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("FrameAttachedToGraph cycle detected, already "
                          "visited vertex [joint_invalid_self_child::self]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint that names itself as the parent frame.
  {
    std::string path = pathBase +"/joint_invalid_self_parent.sdf";

    // Check joint_invalid_self_parent.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("joint with name[self] in model with "
                          "name[joint_invalid_self_parent] must not specify "
                          "its own name as the parent frame."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with identical parent and child.
  {
    std::string path = pathBase +"/joint_invalid_parent_same_as_child.sdf";

    // Check joint_invalid_parent_same_as_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Joint with name[joint] must "
                          "specify different link names for parent and child, "
                          "while [link] was specified for both."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with parent parent frame that resolves
  // to the same value as the child.
  {
    std::string path =
        pathBase + "/joint_invalid_resolved_parent_same_as_child.sdf";

    // Check joint_invalid_resolved_parent_same_as_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("specified parent frame [J1] and child frame [L2] "
                          "that both resolve to [L2], but they should resolve "
                          "to different values."),
              std::string::npos) << output;
  }

  // Check an SDF file with the world specified as a child link.
  {
    std::string path = pathBase +"/joint_child_world.sdf";

    // Check joint_child_world.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Joint with name[joint] specified invalid "
                          "child link [world]."),
              std::string::npos) << output;
  }

  // Check an SDF file with the world specified as a parent link.
  // This is a valid file.
  {
    std::string path = pathBase +"/joint_parent_world.sdf";

    // Check joint_parent_world.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a frame specified as the joint child.
  // This is a valid file.
  {
    std::string path = pathBase +"/joint_child_frame.sdf";

    // Check joint_child_frame.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a frame specified as the joint parent.
  // This is a valid file.
  {
    std::string path = pathBase +"/joint_parent_frame.sdf";

    // Check joint_parent_frame.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with the second link specified as the canonical link.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_canonical_link.sdf";

    // Check model_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with an invalid link specified as the canonical link.
  {
    std::string path = pathBase +"/model_invalid_canonical_link.sdf";

    // Check model_invalid_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("canonical_link with name[link3] not found in "
                          "model with name[model_invalid_canonical_link]."),
              std::string::npos) << output;
  }

  // Check an SDF file with an invalid model without links.
  {
    std::string path = pathBase +"/model_without_links.sdf";

    // Check model_without_links.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("A model must have at least one link."),
              std::string::npos) << output;
  }

  // Check an SDF file with a nested model.
  {
    std::string path = pathBase +"/nested_model.sdf";

    // Check nested_model.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that has a nested canonical link.
  {
    std::string path = pathBase +"/nested_canonical_link.sdf";

    // Check nested_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that has a nested canonical link
  // that is explicitly specified by //model/@canonical_link using ::
  // syntax.
  {
    std::string path = pathBase +"/nested_explicit_canonical_link.sdf";

    // Check nested_explicit_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that a nested model without a link.
  {
    std::string path = pathBase +"/nested_without_links_invalid.sdf";

    // Check nested_without_links_invalid.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("A model must have at least one link."),
              std::string::npos) << output;
  }

  // Check an invalid SDF file that uses reserved names.
  {
    std::string path = pathBase +"/model_invalid_reserved_names.sdf";

    // Check model_invalid_reserved_names.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("The supplied link name [world] is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied link name [__link__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied visual name [__visual__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied collision name [__collision__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied joint name [__joint__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied frame name [__frame__] "
                          "is reserved."),
              std::string::npos) << output;
  }

  // Check that validity checks are disabled inside <plugin> elements
  {
    std::string path = pathBase +"/ignore_sdf_in_plugin.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check that validity checks are disabled inside namespaced elements
  {
    std::string path = pathBase +"/ignore_sdf_in_namespaced_elements.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_attached_to.sdf";

    // Check model_frame_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames attached_to joints.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_attached_to_joint.sdf";

    // Check model_frame_attached_to_joint.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames attached_to a nested model.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_attached_to_nested_model.sdf";

    // Check model_frame_attached_to_nested_model.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames with invalid attached_to attributes.
  {
    std::string path = pathBase +"/model_frame_invalid_attached_to.sdf";

    // Check model_frame_invalid_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("attached_to name[A] specified by frame with "
                          "name[F3] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_frame_invalid_attached_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("attached_to name[F4] is identical to frame "
                          "name[F4], causing a graph cycle in model with "
                          "name[model_frame_invalid_attached_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a cycle in its FrameAttachedTo graph
  {
    std::string path = pathBase +"/model_frame_invalid_attached_to_cycle.sdf";

    // Check model_frame_invalid_attached_to_cycle.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(std::string::npos,
        output.find("FrameAttachedToGraph cycle detected, already visited "
                    "vertex [model_frame_invalid_attached_to_cycle::F1]."))
        << output;
    EXPECT_NE(std::string::npos,
        output.find("FrameAttachedToGraph cycle detected, already visited "
                    "vertex [model_frame_invalid_attached_to_cycle::F2]."))
        << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/world_frame_attached_to.sdf";

    // Check world_frame_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with world frames with invalid attached_to attributes.
  {
    std::string path = pathBase +"/world_frame_invalid_attached_to.sdf";

    // Check world_frame_invalid_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("attached_to name[A] specified by frame with "
                          "name[F] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_attached_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("attached_to name[self_cycle] is identical "
                          "to frame name[self_cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_attached_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with links using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_link_relative_to.sdf";

    // Check model_link_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model links with invalid relative_to attributes.
  {
    std::string path = pathBase +"/model_invalid_link_relative_to.sdf";

    // Check model_invalid_link_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("relative_to name[A] specified by link with "
                          "name[L] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_link_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[self_cycle] is identical to "
                          "link name[self_cycle], causing a graph cycle in "
                          "model with name[model_invalid_link_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with nested_models using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_nested_model_relative_to.sdf";

    // Check model_nested_model_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with nested_models using nested links/frames as joint
  // parent or child frames.
  // This is a valid file.
  {
    std::string path = pathBase +"/joint_nested_parent_child.sdf";

    // Check model_nested_model_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with joints using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_joint_relative_to.sdf";

    // Check model_joint_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model joints with invalid relative_to attributes.
  {
    std::string path = pathBase +"/model_invalid_joint_relative_to.sdf";

    // Check model_invalid_joint_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("relative_to name[A] specified by joint with "
                          "name[J] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_joint_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[Jcycle] is identical to "
                          "joint name[Jcycle], causing a graph cycle in "
                          "model with name[model_invalid_joint_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with model frames using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_relative_to.sdf";

    // Check model_frame_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames relative_to joints.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_relative_to_joint.sdf";

    // Check model_frame_relative_to_joint.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames with invalid relative_to attributes.
  {
    std::string path = pathBase +"/model_invalid_frame_relative_to.sdf";

    // Check model_invalid_frame_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("relative_to name[A] specified by frame with "
                          "name[F] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_frame_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[cycle] is identical to "
                          "frame name[cycle], causing a graph cycle in model "
                          "with name[model_invalid_frame_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a cycle in its PoseRelativeTo graph
  {
    std::string path = pathBase +"/model_invalid_frame_relative_to_cycle.sdf";

    // Check model_invalid_frame_relative_to_cycle.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(std::string::npos,
        output.find("PoseRelativeToGraph cycle detected, already visited "
                    "vertex [model_invalid_frame_relative_to_cycle::F1]."))
        << output;
    EXPECT_NE(std::string::npos,
        output.find("PoseRelativeToGraph cycle detected, already visited "
                    "vertex [model_invalid_frame_relative_to_cycle::F2]."))
        << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/world_frame_relative_to.sdf";

    // Check world_frame_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with world frames with invalid relative_to attributes.
  {
    std::string path = pathBase +"/world_frame_invalid_relative_to.sdf";

    // Check world_frame_invalid_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("relative_to name[A] specified by model with "
                          "name[M] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[cycle] is identical "
                          "to model name[cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[A] specified by frame with "
                          "name[F] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[self_cycle] is identical "
                          "to frame name[self_cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
  }
  // Check an SDF file with an invalid frame specified as the placement_frame
  // attribute.
  {
    std::string path = pathBase + "/model_invalid_placement_frame.sdf";

    // Check model_invalid_placement_frame.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(
        output.find("unable to find unique frame with name [link3] in graph"),
        std::string::npos)
        << output;
  }

  // Check an SDF file with an valid nested model cross references
  {
    std::string path = pathBase + "/nested_model_cross_references.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF model file with an invalid usage of __root__
  {
    std::string path = pathBase + "/model_invalid_root_reference.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [relative_to]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a"
                    " value of attribute [attached_to]"),
        std::string::npos)
        << output;
  }
  // Check an SDF world file with an invalid usage of __root__
  {
    // Set SDF_PATH so that included models can be found
    sdf::testing::setenv(
      "SDF_PATH", sdf::testing::SourceFile("test", "integration", "model"));
    std::string path = pathBase + "/world_invalid_root_reference.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [relative_to]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [attached_to]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [placement_frame]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [canonical_link]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [parent_frame]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of element [placement_frame]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find(
            "The supplied joint child name [__root__] is not valid"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find(
            "The supplied joint parent name [__root__] is not valid"),
        std::string::npos)
        << output;
  }
  // Check an SDF world file with an valid usage of __root__
  {
    std::string path = pathBase + "/world_valid_root_reference.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }
}

/////////////////////////////////////////////////
TEST(check_shapes_sdf, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  {
    std::string path = pathBase +"/shapes.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output);
  }

  {
    std::string path = pathBase +"/shapes_world.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output);
  }
}

/////////////////////////////////////////////////
TEST(check_model_sdf, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/integration/model/box";

  // Check a good SDF file by passing the absolute path
  {
    std::string path = pathBase +"/model.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output);
  }

  // Check a good SDF file from the same folder by passing a relative path
  {
    std::string path = "model.sdf";

    std::string output =
      custom_exec_str("cd " + pathBase + " && " +
                      g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output);
  }
}

/////////////////////////////////////////////////
TEST(describe, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  // Get the description
  std::string output =
    custom_exec_str(g_ignCommand + " sdf -d " + g_sdfVersion);
  EXPECT_FALSE(output.empty());

  // The first line should start with the following text.
  EXPECT_EQ(0u, output.find("<element name ='sdf' required ='1'"));
}

/////////////////////////////////////////////////
TEST(print, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  // Check a good SDF file
  {
    std::string path = pathBase +"/box_plane_low_friction_test.world";
    sdf::SDFPtr sdf(new sdf::SDF());
    EXPECT_TRUE(sdf::init(sdf));
    EXPECT_TRUE(sdf::readFile(path, sdf));

    // Check box_plane_low_friction_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -p " + path + g_sdfVersion);
    EXPECT_EQ(sdf->Root()->ToString(""), output);
  }

  // Check a bad SDF file
  {
    std::string path = pathBase +"/box_bad_test.world";

    // Check box_bad_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -p " + path + g_sdfVersion);
    EXPECT_TRUE(output.find("Required attribute") != std::string::npos);
  }
}

/////////////////////////////////////////////////
TEST(GraphCmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(WorldPoseRelativeTo))
{
  const std::string pathBase = std::string(PROJECT_SOURCE_PATH) + "/test/sdf";

  // world pose relative_to graph
  const std::string path =
    pathBase + "/world_relative_to_nested_reference.sdf";

  const std::string output =
    custom_exec_str(g_ignCommand + " sdf -g pose " + path + g_sdfVersion);

  std::stringstream expected;
  expected << "digraph {\n"
    << "  0 [label=\"__root__ (0)\"];\n"
    << "  1 [label=\"world (1)\"];\n"
    << "  2 [label=\"M1 (2)\"];\n"
    << "  3 [label=\"M1::__model__ (3)\"];\n"
    << "  4 [label=\"M1::L1 (4)\"];\n"
    << "  5 [label=\"M1::L2 (5)\"];\n"
    << "  6 [label=\"M1::J1 (6)\"];\n"
    << "  7 [label=\"M1::F1 (7)\"];\n"
    << "  8 [label=\"M1::CM1 (8)\"];\n"
    << "  9 [label=\"M1::CM1::__model__ (9)\"];\n"
    << "  10 [label=\"M1::CM1::L (10)\"];\n"
    << "  11 [label=\"F2 (11)\"];\n"
    << "  12 [label=\"F3 (12)\"];\n"
    << "  13 [label=\"F4 (13)\"];\n"
    << "  14 [label=\"F5 (14)\"];\n"
    << "  15 [label=\"F6 (15)\"];\n"
    << "  16 [label=\"F7 (16)\"];\n"
    << "  0 -> 1 [label=1];\n"
    << "  2 -> 3 [label=0];\n"
    << "  3 -> 4 [label=1];\n"
    << "  3 -> 5 [label=1];\n"
    << "  8 -> 9 [label=0];\n"
    << "  9 -> 10 [label=1];\n"
    << "  5 -> 6 [label=1];\n"
    << "  5 -> 7 [label=1];\n"
    << "  3 -> 8 [label=1];\n"
    << "  1 -> 2 [label=1];\n"
    << "  2 -> 11 [label=1];\n"
    << "  4 -> 12 [label=1];\n"
    << "  6 -> 13 [label=1];\n"
    << "  7 -> 14 [label=1];\n"
    << "  8 -> 15 [label=1];\n"
    << "  10 -> 16 [label=1];\n"
    << "}";
  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
TEST(GraphCmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(ModelPoseRelativeTo))
{
  const std::string pathBase = std::string(PROJECT_SOURCE_PATH) + "/test/sdf";
  const std::string path = pathBase + "/model_relative_to_nested_reference.sdf";
  const std::string output =
    custom_exec_str(g_ignCommand + " sdf -g pose " + path + g_sdfVersion);

  std::stringstream expected;
  expected << "digraph {\n"
    << "  0 [label=\"__root__ (0)\"];\n"
    << "  1 [label=\"parent_model (1)\"];\n"
    << "  2 [label=\"parent_model::__model__ (2)\"];\n"
    << "  3 [label=\"parent_model::L (3)\"];\n"
    << "  4 [label=\"parent_model::M1 (4)\"];\n"
    << "  5 [label=\"parent_model::M1::__model__ (5)\"];\n"
    << "  6 [label=\"parent_model::M1::L1 (6)\"];\n"
    << "  7 [label=\"parent_model::M1::L2 (7)\"];\n"
    << "  8 [label=\"parent_model::M1::J1 (8)\"];\n"
    << "  9 [label=\"parent_model::M1::F1 (9)\"];\n"
    << "  10 [label=\"parent_model::M1::CM1 (10)\"];\n"
    << "  11 [label=\"parent_model::M1::CM1::__model__ (11)\"];\n"
    << "  12 [label=\"parent_model::M1::CM1::L (12)\"];\n"
    << "  13 [label=\"parent_model::M2 (13)\"];\n"
    << "  14 [label=\"parent_model::M2::__model__ (14)\"];\n"
    << "  15 [label=\"parent_model::M2::L (15)\"];\n"
    << "  16 [label=\"parent_model::M3 (16)\"];\n"
    << "  17 [label=\"parent_model::M3::__model__ (17)\"];\n"
    << "  18 [label=\"parent_model::M3::L (18)\"];\n"
    << "  19 [label=\"parent_model::M4 (19)\"];\n"
    << "  20 [label=\"parent_model::M4::__model__ (20)\"];\n"
    << "  21 [label=\"parent_model::M4::L (21)\"];\n"
    << "  22 [label=\"parent_model::M5 (22)\"];\n"
    << "  23 [label=\"parent_model::M5::__model__ (23)\"];\n"
    << "  24 [label=\"parent_model::M5::L (24)\"];\n"
    << "  25 [label=\"parent_model::M6 (25)\"];\n"
    << "  26 [label=\"parent_model::M6::__model__ (26)\"];\n"
    << "  27 [label=\"parent_model::M6::L (27)\"];\n"
    << "  28 [label=\"parent_model::M7 (28)\"];\n"
    << "  29 [label=\"parent_model::M7::__model__ (29)\"];\n"
    << "  30 [label=\"parent_model::M7::L (30)\"];\n"
    << "  1 -> 2 [label=0];\n"
    << "  2 -> 3 [label=1];\n"
    << "  4 -> 5 [label=0];\n"
    << "  5 -> 6 [label=1];\n"
    << "  5 -> 7 [label=1];\n"
    << "  10 -> 11 [label=0];\n"
    << "  11 -> 12 [label=1];\n"
    << "  7 -> 8 [label=1];\n"
    << "  7 -> 9 [label=1];\n"
    << "  5 -> 10 [label=1];\n"
    << "  13 -> 14 [label=0];\n"
    << "  14 -> 15 [label=1];\n"
    << "  16 -> 17 [label=0];\n"
    << "  17 -> 18 [label=1];\n"
    << "  19 -> 20 [label=0];\n"
    << "  20 -> 21 [label=1];\n"
    << "  22 -> 23 [label=0];\n"
    << "  23 -> 24 [label=1];\n"
    << "  25 -> 26 [label=0];\n"
    << "  26 -> 27 [label=1];\n"
    << "  28 -> 29 [label=0];\n"
    << "  29 -> 30 [label=1];\n"
    << "  2 -> 4 [label=1];\n"
    << "  4 -> 13 [label=1];\n"
    << "  6 -> 16 [label=1];\n"
    << "  8 -> 19 [label=1];\n"
    << "  9 -> 22 [label=1];\n"
    << "  10 -> 25 [label=1];\n"
    << "  12 -> 28 [label=1];\n"
    << "  0 -> 1 [label=1];\n"
    << "}";

  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
TEST(GraphCmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(WorldFrameAttachedTo))
{
  const std::string pathBase = std::string(PROJECT_SOURCE_PATH) + "/test/sdf";
  const std::string path = pathBase + "/world_nested_frame_attached_to.sdf";
  const std::string output =
      custom_exec_str(g_ignCommand + " sdf -g frame " + path + g_sdfVersion);

  std::stringstream expected;

  expected << "digraph {\n"
           << "  0 [label=\"world (0)\"];\n"
           << "  1 [label=\"M1 (1)\"];\n"
           << "  2 [label=\"M1::__model__ (2)\"];\n"
           << "  3 [label=\"M1::L (3)\"];\n"
           << "  4 [label=\"M1::J1 (4)\"];\n"
           << "  5 [label=\"M1::F0 (5)\"];\n"
           << "  6 [label=\"M1::M2 (6)\"];\n"
           << "  7 [label=\"M1::M2::__model__ (7)\"];\n"
           << "  8 [label=\"M1::M2::L (8)\"];\n"
           << "  9 [label=\"world_frame (9)\"];\n"
           << "  10 [label=\"F0 (10)\"];\n"
           << "  11 [label=\"F1 (11)\"];\n"
           << "  12 [label=\"F2 (12)\"];\n"
           << "  13 [label=\"F3 (13)\"];\n"
           << "  14 [label=\"F4 (14)\"];\n"
           << "  15 [label=\"F5 (15)\"];\n"
           << "  16 [label=\"F6 (16)\"];\n"
           << "  1 -> 2 [label=0];\n"
           << "  6 -> 7 [label=0];\n"
           << "  7 -> 8 [label=1];\n"
           << "  4 -> 5 [label=1];\n"
           << "  5 -> 8 [label=1];\n"
           << "  2 -> 3 [label=1];\n"
           << "  9 -> 0 [label=1];\n"
           << "  10 -> 0 [label=1];\n"
           << "  11 -> 1 [label=1];\n"
           << "  12 -> 3 [label=1];\n"
           << "  13 -> 6 [label=1];\n"
           << "  14 -> 8 [label=1];\n"
           << "  15 -> 5 [label=1];\n"
           << "  16 -> 4 [label=1];\n"
           << "}";
  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
TEST(GraphCmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(ModelFrameAttachedTo))
{
  const std::string pathBase = std::string(PROJECT_SOURCE_PATH) + "/test/sdf";
  const std::string path = pathBase + "/model_nested_frame_attached_to.sdf";
  const std::string output =
      custom_exec_str(g_ignCommand + " sdf -g frame " + path + g_sdfVersion);

  std::stringstream expected;

  expected
      << "digraph {\n"
      << "  0 [label=\"__root__ (0)\"];\n"
      << "  1 [label=\"nested_model_frame_attached_to (1)\"];\n"
      << "  2 [label=\"nested_model_frame_attached_to::__model__ (2)\"];\n"
      << "  3 [label=\"nested_model_frame_attached_to::L (3)\"];\n"
      << "  4 [label=\"nested_model_frame_attached_to::F0 (4)\"];\n"
      << "  5 [label=\"nested_model_frame_attached_to::F1 (5)\"];\n"
      << "  6 [label=\"nested_model_frame_attached_to::F2 (6)\"];\n"
      << "  7 [label=\"nested_model_frame_attached_to::F3 (7)\"];\n"
      << "  8 [label=\"nested_model_frame_attached_to::F4 (8)\"];\n"
      << "  9 [label=\"nested_model_frame_attached_to::M1 (9)\"];\n"
      << "  10 [label=\"nested_model_frame_attached_to::M1::__model__ "
         "(10)\"];\n"
      << "  11 [label=\"nested_model_frame_attached_to::M1::L (11)\"];\n"
      << "  12 [label=\"nested_model_frame_attached_to::M1::J1 (12)\"];\n"
      << "  13 [label=\"nested_model_frame_attached_to::M1::F (13)\"];\n"
      << "  14 [label=\"nested_model_frame_attached_to::M1::M2 (14)\"];\n"
      << "  15 [label=\"nested_model_frame_attached_to::M1::M2::__model__ "
         "(15)\"];\n"
      << "  16 [label=\"nested_model_frame_attached_to::M1::M2::L (16)\"];\n"
      << "  1 -> 2 [label=0];\n"
      << "  9 -> 10 [label=0];\n"
      << "  14 -> 15 [label=0];\n"
      << "  15 -> 16 [label=1];\n"
      << "  12 -> 13 [label=1];\n"
      << "  13 -> 14 [label=1];\n"
      << "  10 -> 11 [label=1];\n"
      << "  4 -> 9 [label=1];\n"
      << "  5 -> 11 [label=1];\n"
      << "  6 -> 5 [label=1];\n"
      << "  7 -> 13 [label=1];\n"
      << "  8 -> 12 [label=1];\n"
      << "  2 -> 3 [label=1];\n"
      << "}";

  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  // Set IGN_CONFIG_PATH to the directory where the .yaml configuration file
  // is located.
  sdf::testing::setenv("IGN_CONFIG_PATH", IGN_CONFIG_PATH);

  // Make sure that we load the library recently built and not the one installed
  // in your system. This is done by placing the the current build directory
  // first in the LD_LIBRARY_PATH environment variable.
  //
  // We need to keep the existing LD_LIBRARY_PATH so that libsdformat.so can
  // find its dependency.
#ifndef _WIN32
  std::string testLibraryPath = IGN_TEST_LIBRARY_PATH;

  std::string currentLibraryPath;
  if (sdf::testing::env("LD_LIBRARY_PATH", currentLibraryPath))
  {
    testLibraryPath = testLibraryPath + ":" + currentLibraryPath;
  }

  sdf::testing::setenv("LD_LIBRARY_PATH", testLibraryPath);
#endif

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
