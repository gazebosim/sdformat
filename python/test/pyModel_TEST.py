# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz.math import Pose3d
from sdformat import (Plugin, Model, Joint, Link, Error, Frame,
                      SemanticPose, SDFErrorsException)
import sdformat as sdf
import math
import unittest

class ModelTEST(unittest.TestCase):

    def test_default_construction(self):
        model = Model()
        self.assertFalse(model.name())

        model.set_name("test_model")
        self.assertEqual("test_model", model.name())

        self.assertFalse(model.static())
        model.set_static(True)
        self.assertTrue(model.static())

        self.assertFalse(model.self_collide())
        model.set_self_collide(True)
        self.assertTrue(model.self_collide())

        self.assertTrue(model.allow_auto_disable())
        model.set_allow_auto_disable(False)
        self.assertFalse(model.allow_auto_disable())

        self.assertFalse(model.enable_wind())
        model.set_enable_wind(True)
        self.assertTrue(model.enable_wind())

        self.assertEqual(0, model.model_count())
        self.assertEqual(None, model.model_by_index(0))
        self.assertEqual(None, model.model_by_index(1))
        self.assertEqual(None, model.model_by_name(""))
        self.assertEqual(None, model.model_by_name("default"))
        self.assertEqual(None, model.model_by_name("a::b"))
        self.assertEqual(None, model.model_by_name("a::b::c"))
        self.assertEqual(None, model.model_by_name("::::"))
        self.assertFalse(model.model_name_exists(""))
        self.assertFalse(model.model_name_exists("default"))
        self.assertFalse(model.model_name_exists("a::b"))
        self.assertFalse(model.model_name_exists("a::b::c"))
        self.assertFalse(model.model_name_exists("::::"))

        self.assertEqual(0, model.link_count())
        self.assertEqual(None, model.link_by_index(0))
        self.assertEqual(None, model.link_by_index(1))
        self.assertEqual(None, model.link_by_name(""))
        self.assertEqual(None, model.link_by_name("default"))
        self.assertEqual(None, model.link_by_name("a::b"))
        self.assertEqual(None, model.link_by_name("a::b::c"))
        self.assertEqual(None, model.link_by_name("::::"))
        self.assertFalse(model.link_name_exists(""))
        self.assertFalse(model.link_name_exists("default"))
        self.assertFalse(model.link_name_exists("a::b"))
        self.assertFalse(model.link_name_exists("a::b::c"))
        self.assertFalse(model.link_name_exists("::::"))

        self.assertEqual(0, model.joint_count())
        self.assertEqual(None, model.joint_by_index(0))
        self.assertEqual(None, model.joint_by_index(1))
        self.assertEqual(None, model.joint_by_name(""))
        self.assertEqual(None, model.joint_by_name("default"))
        self.assertEqual(None, model.joint_by_name("a::b"))
        self.assertEqual(None, model.joint_by_name("a::b::c"))
        self.assertEqual(None, model.joint_by_name("::::"))
        self.assertFalse(model.joint_name_exists(""))
        self.assertFalse(model.joint_name_exists("default"))
        self.assertFalse(model.joint_name_exists("a::b"))
        self.assertFalse(model.joint_name_exists("a::b::c"))
        self.assertFalse(model.joint_name_exists("::::"))

        self.assertEqual(0, model.frame_count())
        self.assertEqual(None, model.frame_by_index(0))
        self.assertEqual(None, model.frame_by_index(1))
        self.assertEqual(None, model.frame_by_name(""))
        self.assertEqual(None, model.frame_by_name("default"))
        self.assertEqual(None, model.frame_by_name("a::b"))
        self.assertEqual(None, model.frame_by_name("a::b::c"))
        self.assertEqual(None, model.frame_by_name("::::"))
        self.assertFalse(model.frame_name_exists(""))
        self.assertFalse(model.frame_name_exists("default"))
        self.assertFalse(model.frame_name_exists("a::b"))
        self.assertFalse(model.frame_name_exists("a::b::c"))
        self.assertFalse(model.frame_name_exists("::::"))

        self.assertFalse(model.canonical_link_name())
        self.assertEqual(None, model.canonical_link())
        model.set_canonical_link_name("link")
        self.assertEqual("link", model.canonical_link_name())
        self.assertEqual(None, model.canonical_link())

        self.assertFalse(model.placement_frame_name())
        model.set_placement_frame_name("test_frame")
        self.assertEqual("test_frame", model.placement_frame_name())

        self.assertEqual(Pose3d.ZERO, model.raw_pose())
        self.assertFalse(model.pose_relative_to())

        semanticPose = model.semantic_pose()
        self.assertEqual(model.raw_pose(), semanticPose.raw_pose())
        self.assertFalse(semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        model.set_raw_pose(Pose3d(1, 2, 3, 0, 0, math.pi))
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, math.pi), model.raw_pose())

        model.set_pose_relative_to("world")
        self.assertEqual("world", model.pose_relative_to())

        semanticPose = model.semantic_pose()
        self.assertEqual(model.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("world", semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        with self.assertRaises(SDFErrorsException) as cm:
            model.validate_graphs()
        errors = cm.exception.errors
        self.assertEqual(2, len(errors))
        self.assertEqual(errors[0].code(), sdf.ErrorCode.FRAME_ATTACHED_TO_GRAPH_ERROR)
        self.assertEqual(errors[0].message(),
          "FrameAttachedToGraph error: scope does not point to a valid graph.")
        self.assertEqual(errors[1].code(), sdf.ErrorCode.POSE_RELATIVE_TO_GRAPH_ERROR)
        self.assertEqual(errors[1].message(),
            "PoseRelativeToGraph error: scope does not point to a valid graph.")

        # model doesn't have graphs, so no names should exist in graphs
        self.assertFalse(model.name_exists_in_frame_attached_to_graph(""));
        self.assertFalse(model.name_exists_in_frame_attached_to_graph("link"));


    def test_copy_construction(self):
        model = Model()
        model.set_name("test_model")

        model2 = Model(model)
        self.assertEqual("test_model", model2.name())


    def test_add_link(self):
        model = Model()
        self.assertEqual(0, model.link_count())

        link = Link()
        link.set_name("link1")
        self.assertTrue(model.add_link(link))
        self.assertEqual(1, model.link_count())
        self.assertFalse(model.add_link(link))
        self.assertEqual(1, model.link_count())

        model.clear_links()
        self.assertEqual(0, model.link_count())

        self.assertTrue(model.add_link(link))
        self.assertEqual(1, model.link_count())
        linkFromModel = model.link_by_index(0)
        self.assertNotEqual(None, linkFromModel)
        self.assertEqual(linkFromModel.name(), link.name())


    def test_add_joint(self):
        model = Model()
        self.assertEqual(0, model.joint_count())

        joint = Joint()
        joint.set_name("joint1")
        self.assertTrue(model.add_joint(joint))
        self.assertEqual(1, model.joint_count())
        self.assertFalse(model.add_joint(joint))
        self.assertEqual(1, model.joint_count())

        model.clear_joints()
        self.assertEqual(0, model.joint_count())

        self.assertTrue(model.add_joint(joint))
        self.assertEqual(1, model.joint_count())
        jointFromModel = model.joint_by_index(0)
        self.assertNotEqual(None, jointFromModel)
        self.assertEqual(jointFromModel.name(), joint.name())


    def test_add_model(self):
        model = Model()
        self.assertEqual(0, model.model_count())

        nestedModel = Model()
        nestedModel.set_name("model1")
        self.assertTrue(model.add_model(nestedModel))
        self.assertEqual(1, model.model_count())
        self.assertFalse(model.add_model(nestedModel))
        self.assertEqual(1, model.model_count())

        model.clear_models()
        self.assertEqual(0, model.model_count())

        self.assertTrue(model.add_model(nestedModel))
        self.assertEqual(1, model.model_count())
        modelFromModel = model.model_by_index(0)
        self.assertNotEqual(None, modelFromModel)
        self.assertEqual(modelFromModel.name(), nestedModel.name())


    def test_add_modify_frame(self):
        model = Model()
        self.assertEqual(0, model.frame_count())

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(model.add_frame(frame))
        self.assertEqual(1, model.frame_count())
        self.assertFalse(model.add_frame(frame))
        self.assertEqual(1, model.frame_count())

        model.clear_frames()
        self.assertEqual(0, model.frame_count())

        self.assertTrue(model.add_frame(frame))
        self.assertEqual(1, model.frame_count())

        frameFromModel = model.frame_by_index(0)
        self.assertNotEqual(None, frameFromModel)
        self.assertEqual(frameFromModel.name(), frame.name())

        mutableFrame = model.frame_by_index(0)
        mutableFrame.set_name("newname1")
        self.assertEqual(mutableFrame.name(), model.frame_by_index(0).name())

        mutableframe_by_name = model.frame_by_name("frame1")
        self.assertEqual(None, mutableframe_by_name)
        mutableframe_by_name = model.frame_by_name("newname1")
        self.assertNotEqual(None, mutableframe_by_name)
        self.assertEqual("newname1", model.frame_by_name("newname1").name())


    def test_uri(self):
        model = Model()
        uri = "https:#fuel.gazebosim.org/1.0/openrobotics/models/my-model"

        model.set_uri(uri)
        self.assertEqual(uri, model.uri())


    def test_mutable_by_index(self):
        model = Model()
        model.set_name("model1")
        self.assertEqual(0, model.model_count())

        link = Link()
        link.set_name("link1")
        self.assertTrue(model.add_link(link))

        joint = Joint()
        joint.set_name("joint1")
        self.assertTrue(model.add_joint(joint))

        nestedModel = Model()
        nestedModel.set_name("child1")
        self.assertTrue(model.add_model(nestedModel))

        # Modify the link
        l = model.link_by_index(0)
        self.assertNotEqual(None, l)
        self.assertEqual("link1", l.name())
        l.set_name("link2")
        self.assertEqual("link2", model.link_by_index(0).name())

        # Modify the joint
        j = model.joint_by_index(0)
        self.assertNotEqual(None, j)
        self.assertEqual("joint1", j.name())
        j.set_name("joint2")
        self.assertEqual("joint2", model.joint_by_index(0).name())

        # Modify the nested model
        m = model.model_by_index(0)
        self.assertNotEqual(None, m)
        self.assertEqual("child1", m.name())
        m.set_name("child2")
        self.assertEqual("child2", model.model_by_index(0).name())


    def test_mutable_by_name(self):
        model = Model()
        model.set_name("model1")
        self.assertEqual(0, model.model_count())

        link = Link()
        link.set_name("link1")
        self.assertTrue(model.add_link(link))

        joint = Joint()
        joint.set_name("joint1")
        self.assertTrue(model.add_joint(joint))

        nestedModel = Model()
        nestedModel.set_name("child1")
        self.assertTrue(model.add_model(nestedModel))

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(model.add_frame(frame))

        # Modify the link
        l = model.link_by_name("link1")
        self.assertNotEqual(None, l)
        self.assertEqual("link1", l.name())
        l.set_name("link2")
        self.assertFalse(model.link_name_exists("link1"))
        self.assertTrue(model.link_name_exists("link2"))

        # Modify the joint
        j = model.joint_by_name("joint1")
        self.assertNotEqual(None, j)
        self.assertEqual("joint1", j.name())
        j.set_name("joint2")
        self.assertFalse(model.joint_name_exists("joint1"))
        self.assertTrue(model.joint_name_exists("joint2"))

        # Modify the nested model
        m = model.model_by_name("child1")
        self.assertNotEqual(None, m)
        self.assertEqual("child1", m.name())
        m.set_name("child2")
        self.assertFalse(model.model_name_exists("child1"))
        self.assertTrue(model.model_name_exists("child2"))

        # Modify the frame
        f = model.frame_by_name("frame1")
        self.assertNotEqual(None, f)
        self.assertEqual("frame1", f.name())
        f.set_name("frame2")
        self.assertFalse(model.frame_name_exists("frame1"))
        self.assertTrue(model.frame_name_exists("frame2"))


    def test_plugins(self):
        model = Model()
        self.assertEqual(0, len(model.plugins()))

        plugin = Plugin()
        plugin.set_name("name1")
        plugin.set_filename("filename1")

        model.add_plugin(plugin)
        self.assertEqual(1, len(model.plugins()))

        plugin.set_name("name2")
        model.add_plugin(plugin)
        self.assertEqual(2, len(model.plugins()))

        self.assertEqual("name1", model.plugins()[0].name())
        self.assertEqual("name2", model.plugins()[1].name())

        model.clear_plugins()
        self.assertEqual(0, len(model.plugins()))


if __name__ == '__main__':
    unittest.main()
