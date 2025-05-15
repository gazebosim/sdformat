# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required _by_ applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz.math import Color, Vector3d, SphericalCoordinates
from gz.math import Inertiald, MassMatrix3d, Pose3d, Vector3d
from sdformat import (Atmosphere, Gui, Physics, Plugin, Error,
                      Frame, Joint, Light, Model, ParserConfig, Root, Scene, World)
import sdformat as sdf
import unittest
import math

# TODO(ahcorde)
# - Add Actor when the sdf::Classes are ported

class WorldTEST(unittest.TestCase):

    def test_default_construction(self):
        world = World()
        self.assertFalse(world.name())
        self.assertEqual(Vector3d(0, 0, -9.80665), world.gravity())
        self.assertEqual(Vector3d(5.5645e-6, 22.8758e-6, -42.3884e-6),
                world.magnetic_field())
        self.assertEqual("default", world.audio_device())
        self.assertEqual(Vector3d.ZERO, world.wind_linear_velocity())

        self.assertEqual(0, world.model_count())
        self.assertEqual(None, world.model_by_index(0))
        self.assertEqual(None, world.model_by_index(1))
        self.assertFalse(world.model_name_exists(""))
        self.assertFalse(world.model_name_exists("default"))
        self.assertFalse(world.model_name_exists("a::b"))
        self.assertFalse(world.model_name_exists("a::b::c"))
        self.assertFalse(world.model_name_exists("::::"))
        self.assertEqual(None, world.model_by_name(""))
        self.assertEqual(None, world.model_by_name("default"))
        self.assertEqual(None, world.model_by_name("a::b"))
        self.assertEqual(None, world.model_by_name("a::b::c"))
        self.assertEqual(None, world.model_by_name("::::"))

        self.assertEqual(0, world.frame_count())
        self.assertEqual(None, world.frame_by_index(0))
        self.assertEqual(None, world.frame_by_index(1))
        self.assertFalse(world.frame_name_exists(""))
        self.assertFalse(world.frame_name_exists("a::b"))
        self.assertFalse(world.frame_name_exists("a::b::c"))
        self.assertFalse(world.frame_name_exists("::::"))
        self.assertEqual(None, world.frame_by_name(""))
        self.assertEqual(None, world.frame_by_name("default"))
        self.assertEqual(None, world.frame_by_name("a::b"))
        self.assertEqual(None, world.frame_by_name("a::b::c"))
        self.assertEqual(None, world.frame_by_name("::::"))

        self.assertEqual(0, world.joint_count())
        self.assertEqual(None, world.joint_by_index(0))
        self.assertEqual(None, world.joint_by_index(1))
        self.assertFalse(world.joint_name_exists(""))
        self.assertFalse(world.joint_name_exists("default"))
        self.assertFalse(world.joint_name_exists("a::b"))
        self.assertFalse(world.joint_name_exists("a::b::c"))
        self.assertFalse(world.joint_name_exists("::::"))
        self.assertEqual(None, world.joint_by_name(""))
        self.assertEqual(None, world.joint_by_name("default"))
        self.assertEqual(None, world.joint_by_name("a::b"))
        self.assertEqual(None, world.joint_by_name("a::b::c"))
        self.assertEqual(None, world.joint_by_name("::::"))

        self.assertEqual(1, world.physics_count())

        errors = world.validate_graphs()
        self.assertEqual(2, len(errors))
        self.assertEqual(errors[0].code(), sdf.ErrorCode.FRAME_ATTACHED_TO_GRAPH_ERROR)
        self.assertEqual(errors[0].message(),
          "FrameAttachedToGraph error: scope does not point to a valid graph.")
        self.assertEqual(errors[1].code(), sdf.ErrorCode.POSE_RELATIVE_TO_GRAPH_ERROR)
        self.assertEqual(errors[1].message(),
          "PoseRelativeToGraph error: scope does not point to a valid graph.")

        # world doesn't have graphs, so no names should exist in graphs
        self.assertFalse(world.name_exists_in_frame_attached_to_graph(""))
        self.assertFalse(world.name_exists_in_frame_attached_to_graph("link"))


    def test_copy_construction(self):
        world = World()
        atmosphere = Atmosphere()
        atmosphere.set_pressure(0.1)
        world.set_atmosphere(atmosphere)
        world.set_audio_device("test_audio_device")
        world.set_gravity(Vector3d(1, 0, 0))
        world.set_spherical_coordinates(SphericalCoordinates())

        gui = Gui()
        gui.set_fullscreen(True)
        world.set_gui(gui)

        scene = Scene()
        scene.set_grid(True)
        world.set_scene(scene)

        world.set_magnetic_field(Vector3d(0, 1, 0))
        world.set_name("test_world")

        world.set_wind_linear_velocity(Vector3d(0, 0, 1))

        world2 = World(world)

        self.assertTrue(None != world.atmosphere())
        self.assertAlmostEqual(0.1, world.atmosphere().pressure())
        self.assertTrue(None != world.spherical_coordinates())
        self.assertEqual(SphericalCoordinates.EARTH_WGS84,
          world.spherical_coordinates().surface())
        self.assertEqual("test_audio_device", world.audio_device())
        self.assertEqual(Vector3d.UNIT_X, world.gravity())

        self.assertTrue(None != world.gui())
        self.assertEqual(gui.fullscreen(), world.gui().fullscreen())

        self.assertTrue(None != world.scene())
        self.assertEqual(scene.grid(), world.scene().grid())

        self.assertEqual(Vector3d.UNIT_Y, world.magnetic_field())
        self.assertEqual(Vector3d.UNIT_Z, world.wind_linear_velocity())
        self.assertEqual("test_world", world.name())

        self.assertTrue(None != world2.atmosphere())
        self.assertAlmostEqual(0.1, world2.atmosphere().pressure())
        self.assertEqual("test_audio_device", world2.audio_device())
        self.assertEqual(Vector3d.UNIT_X, world2.gravity())

        self.assertTrue(None != world2.gui())
        self.assertEqual(gui.fullscreen(), world2.gui().fullscreen())

        self.assertTrue(None != world2.scene())
        self.assertEqual(scene.grid(), world2.scene().grid())

        self.assertEqual(Vector3d.UNIT_Y, world2.magnetic_field())
        self.assertEqual(Vector3d.UNIT_Z, world2.wind_linear_velocity())
        self.assertEqual("test_world", world2.name())


    def test_set(self):
        world = World()
        self.assertFalse(world.name())

        world.set_name("default")
        self.assertEqual("default", world.name())

        world.set_audio_device("/dev/audio")
        self.assertEqual("/dev/audio", world.audio_device())

        world.set_wind_linear_velocity(Vector3d(0, 1 , 2))
        self.assertEqual(Vector3d(0, 1, 2), world.wind_linear_velocity())

        world.set_gravity(Vector3d(1, -2, 4))
        self.assertEqual(Vector3d(1, -2, 4), world.gravity())

        world.set_magnetic_field(Vector3d(1.2, -2.3, 4.5))
        self.assertEqual(Vector3d(1.2, -2.3, 4.5), world.magnetic_field())

    def test_set_gui(self):
        gui = Gui()

        world = World()
        self.assertEqual(None, world.gui())

        world.set_gui(gui)
        self.assertNotEqual(None, world.gui())
        self.assertFalse(world.gui().fullscreen())

    def test_set_physics(self):
        world = World()
        self.assertNotEqual(None, world.physics_default())
        physics = Physics()
        physics.set_name("physics1")
        physics.set_default(True)
        physics.set_engine_type("bullet")
        physics.set_max_step_size(1.234)
        physics.set_real_time_factor(2.45)

        world.clear_plugins()

        self.assertTrue(world.add_physics(physics))
        self.assertFalse(world.add_physics(physics))

        physics = world.physics_default()

        self.assertTrue(physics.is_default())
        self.assertEqual("bullet", physics.engine_type())
        self.assertAlmostEqual(1.234, physics.max_step_size())
        self.assertAlmostEqual(2.45, physics.real_time_factor())

    def test_set_scene(self):
        world = World()
        self.assertNotEqual(None, world.scene())

        scene = Scene()
        scene.set_ambient(Color.BLUE)
        scene.set_background(Color.RED)
        scene.set_grid(True)
        scene.set_shadows(True)
        scene.set_origin_visual(True)
        world.set_scene(scene)

        self.assertNotEqual(None, world.scene())
        self.assertEqual(Color.BLUE, world.scene().ambient())
        self.assertEqual(Color.RED, world.scene().background())
        self.assertTrue(world.scene().grid())
        self.assertTrue(world.scene().shadows())
        self.assertTrue(world.scene().origin_visual())

    def test_add_model(self):
        world = World()
        self.assertEqual(0, world.model_count())

        model = Model()
        model.set_name("model1")
        self.assertTrue(world.add_model(model))
        self.assertEqual(1, world.model_count())
        self.assertFalse(world.add_model(model))
        self.assertEqual(1, world.model_count())

        world.clear_models()
        self.assertEqual(0, world.model_count())

        self.assertTrue(world.add_model(model))
        self.assertEqual(1, world.model_count())
        modelFromWorld = world.model_by_index(0)
        self.assertNotEqual(None, modelFromWorld)
        self.assertEqual(modelFromWorld.name(), model.name())


    def test_add_modify_frame(self):
        world = World()
        self.assertEqual(0, world.frame_count())

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(world.add_frame(frame))
        self.assertEqual(1, world.frame_count())
        self.assertFalse(world.add_frame(frame))
        self.assertEqual(1, world.frame_count())

        world.clear_frames()
        self.assertEqual(0, world.frame_count())

        self.assertTrue(world.add_frame(frame))
        self.assertEqual(1, world.frame_count())
        frameFromWorld = world.frame_by_index(0)
        self.assertNotEqual(None, frameFromWorld)
        self.assertEqual(frameFromWorld.name(), frame.name())

        mutableFrame = world.frame_by_index(0)
        mutableFrame.set_name("newName1")
        self.assertEqual(mutableFrame.name(), world.frame_by_index(0).name())

        mutableframe_by_Name = world.frame_by_name("frame1")
        self.assertEqual(None, mutableframe_by_Name)
        mutableframe_by_Name = world.frame_by_name("newName1")
        self.assertNotEqual(None, mutableframe_by_Name)
        self.assertEqual("newName1", world.frame_by_name("newName1").name())

# ########################/
# TEST(DOMWorld, AddActor)
# {
#   world = World()
#   self.assertEqual(0, world.Actor_count())
#
#   sdf::Actor actor
#   actor.set_name("actor1")
#   self.assertTrue(world.add_Actor(actor))
#   self.assertEqual(1, world.Actor_count())
#   self.assertFalse(world.add_Actor(actor))
#   self.assertEqual(1, world.Actor_count())
#
#   world.clear_Actors()
#   self.assertEqual(0, world.Actor_count())
#
#   self.assertTrue(world.add_Actor(actor))
#   self.assertEqual(1, world.Actor_count())
#   const sdf::Actor *actorFromWorld = world.Actor_by_index(0)
#   self.assertNotEqual(None, actorFromWorld)
#   self.assertEqual(actorFromWorld.name(), actor.name())
# }
#

    def add_light(self):
        world = World()
        self.assertEqual(0, world.light_count())

        light = Light()
        light.set_name("light1")
        self.assertTrue(world.add_light(light))
        self.assertEqual(1, world.light_count())
        self.assertFalse(world.add_light(light))
        self.assertEqual(1, world.light_count())

        world.clear_lights()
        self.assertEqual(0, world.light_count())

        self.assertTrue(world.add_light(light))
        self.assertEqual(1, world.light_count())
        lightFromWorld = world.light_by_index(0)
        self.assertNotEqual(None, lightFromWorld)
        self.assertEqual(lightFromWorld.name(), light.name())

    def test_mutable_by_index(self):
        world = World()

        model = Model()
        model.set_name("model1")
        self.assertTrue(world.add_model(model))

        # sdf::Actor actor
        # actor.set_name("actor1")
        # self.assertTrue(world.add_Actor(actor))
        #
        light = Light()
        light.set_name("light1")
        self.assertTrue(world.add_light(light))

        physics = Physics()
        physics.set_name("physics1")
        self.assertTrue(world.add_physics(physics))

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(world.add_frame(frame))

        joint = Joint()
        joint.set_name("joint1")
        self.assertTrue(world.add_joint(joint))

        # Modify the model
        m = world.model_by_index(0)
        self.assertNotEqual(None, m)
        self.assertEqual("model1", m.name())
        m.set_name("model2")
        self.assertEqual("model2", world.model_by_index(0).name())

        # # Modify the actor
        # sdf::Actor *a = world.Actor_by_index(0)
        # self.assertNotEqual(None, a)
        # self.assertEqual("actor1", a.name())
        # a.set_name("actor2")
        # self.assertEqual("actor2", world.Actor_by_index(0).name())
        #
        # Modify the light
        l = world.light_by_index(0)
        self.assertNotEqual(None, l)
        self.assertEqual("light1", l.name())
        l.set_name("light2")
        self.assertEqual("light2", world.light_by_index(0).name())

        # Modify the physics
        p = world.physics_by_index(1)
        self.assertNotEqual(None, p)
        self.assertEqual("physics1", p.name())
        p.set_name("physics2")
        self.assertEqual("physics2", world.physics_by_index(1).name())

        # Modify the frame
        f = world.frame_by_index(0)
        self.assertNotEqual(None, f)
        self.assertEqual("frame1", f.name())
        f.set_name("frame2")
        self.assertEqual("frame2", world.frame_by_index(0).name())

        # Modify the joint
        j = world.joint_by_index(0)
        self.assertNotEqual(None, j)
        self.assertEqual("joint1", j.name())
        j.set_name("joint2")
        self.assertEqual("joint2", world.joint_by_index(0).name())

    def test_mutable_by_name(self):
        world = World()

        model = Model()
        model.set_name("model1")
        self.assertTrue(world.add_model(model))

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(world.add_frame(frame))

        joint = Joint()
        joint.set_name("joint1")
        self.assertTrue(world.add_joint(joint))

        # Modify the model
        m = world.model_by_name("model1")
        self.assertNotEqual(None, m)
        self.assertEqual("model1", m.name())
        m.set_name("model2")
        self.assertFalse(world.model_by_name("model1"))
        self.assertTrue(world.model_by_name("model2"))

        # Modify the frame
        f = world.frame_by_name("frame1")
        self.assertNotEqual(None, f)
        self.assertEqual("frame1", f.name())
        f.set_name("frame2")
        self.assertFalse(world.frame_by_name("frame1"))
        self.assertTrue(world.frame_by_name("frame2"))

        # Modify the joint
        j = world.joint_by_name("joint1")
        self.assertNotEqual(None, j)
        self.assertEqual("joint1", j.name())
        j.set_name("joint2")
        self.assertFalse(world.joint_by_name("joint1"))
        self.assertTrue(world.joint_by_name("joint2"))

    def test_plugins(self):
        world = World()
        self.assertEqual(0, len(world.plugins()))

        plugin = Plugin()
        plugin.set_name("name1")
        plugin.set_filename("filename1")

        world.add_plugin(plugin)
        self.assertEqual(1, len(world.plugins()))

        plugin.set_name("name2")
        world.add_plugin(plugin)
        self.assertEqual(2, len(world.plugins()))

        self.assertEqual("name1", world.plugins()[0].name())
        self.assertEqual("name2", world.plugins()[1].name())

        world.clear_plugins()
        self.assertEqual(0, len(world.plugins()))

    def test_resolve_auto_inertials(self):
        sdf = "<?xml version=\"1.0\"?>" + \
        " <sdf version=\"1.11\">" + \
        "  <world name='inertial_test_world'>" + \
        "   <model name='shapes'>" + \
        "     <link name='link'>" + \
        "       <inertial auto='true' />" + \
        "       <collision name='box_col'>" + \
        "         <density>1240.0</density>" + \
        "         <geometry>" + \
        "           <box>" + \
        "             <size>2 2 2</size>" + \
        "           </box>" + \
        "         </geometry>" + \
        "       </collision>" + \
        "     </link>" + \
        "   </model>" + \
        "  </world>" + \
        " </sdf>"

        root = Root()
        sdfParserConfig = ParserConfig()
        errors = root.load_sdf_string(sdf, sdfParserConfig)
        self.assertEqual(None, errors)

        world = root.world_by_index(0)
        model = world.model_by_index(0)
        link = model.link_by_index(0)

        errors = []
        root.resolve_auto_inertials(errors, sdfParserConfig)

        l = 2.0
        w = 2.0
        h = 2.0

        expectedMass = l * w * h * 1240.0
        ixx = (1.0 / 12.0) * expectedMass * (w * w + h * h)
        iyy = (1.0 / 12.0) * expectedMass * (l * l + h * h)
        izz = (1.0 / 12.0) * expectedMass * (l * l + w * w)

        expectedMassMat = MassMatrix3d(expectedMass, Vector3d(ixx, iyy, izz), Vector3d.ZERO)

        expectedInertial = Inertiald()
        expectedInertial.set_mass_matrix(expectedMassMat)
        expectedInertial.set_pose(Pose3d.ZERO)

        self.assertEqual(0, len(errors))
        self.assertEqual(expectedInertial, link.inertial())

if __name__ == '__main__':
    unittest.main()
