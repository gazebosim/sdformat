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
from ignition.math import Pose3d, Vector3d, SphericalCoordinates
from sdformat import Error, Frame, Light, Model, World
import unittest
import math

# TODO(ahcorde)
# - Add Atmosphere, GUI, Actor, Light, Scene and Plugin tests when the sdf::Classes are ported

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
        self.assertFalse(world.frame_name_exists("default"))

        self.assertEqual(0, world.frame_count())
        self.assertEqual(None, world.frame_by_index(0))
        self.assertEqual(None, world.frame_by_index(1))
        self.assertFalse(world.frame_name_exists(""))
        self.assertFalse(world.frame_name_exists("default"))

        self.assertEqual(1, world.physics_count())

        errors = world.validate_graphs()
        self.assertEqual(2, len(errors))
        self.assertEqual(errors[0].code(), Error.ErrorCode.FRAME_ATTACHED_TO_GRAPH_ERROR)
        self.assertEqual(errors[0].message(),
          "FrameAttachedToGraph error: scope does not point to a valid graph.")
        self.assertEqual(errors[1].code(), Error.ErrorCode.POSE_RELATIVE_TO_GRAPH_ERROR)
        self.assertEqual(errors[1].message(),
          "PoseRelativeToGraph error: scope does not point to a valid graph.")


    def test_copy_construction(self):
        world = World()
        # atmosphere = Atmosphere()
        # atmosphere.set_pressure(0.1)
        # world.set_Atmosphere(atmosphere)
        world.set_audio_device("test_audio_device")
        world.set_gravity(Vector3d(1, 0, 0))
        world.set_spherical_coordinates(SphericalCoordinates())

        # sdf::Gui gui
        # gui.set_Fullscreen(true)
        # world.set_Gui(gui)
        #
        # sdf::Scene scene
        # scene.set_Grid(true)
        # world.set_Scene(scene)

        world.set_magnetic_field(Vector3d(0, 1, 0))
        world.set_name("test_world")

        world.set_wind_linear_velocity(Vector3d(0, 0, 1))

        world2 = World(world)

        # self.assertTrue(None != world.Atmosphere())
        # self.assertAlmostEqual(0.1, world.Atmosphere().Pressure())
        self.assertTrue(None != world.spherical_coordinates())
        self.assertEqual(SphericalCoordinates.EARTH_WGS84,
          world.spherical_coordinates().surface())
        self.assertEqual("test_audio_device", world.audio_device())
        self.assertEqual(Vector3d.UNIT_X, world.gravity())

        # self.assertTrue(None != world.Gui())
        # self.assertEqual(gui.Fullscreen(), world.Gui().Fullscreen())

        # self.assertTrue(None != world.Scene())
        # self.assertEqual(scene.Grid(), world.Scene().Grid())

        self.assertEqual(Vector3d.UNIT_Y, world.magnetic_field())
        self.assertEqual(Vector3d.UNIT_Z, world.wind_linear_velocity())
        self.assertEqual("test_world", world.name())

        # self.assertTrue(None != world2.Atmosphere())
        # self.assertAlmostEqual(0.1, world2.Atmosphere().Pressure())
        self.assertEqual("test_audio_device", world2.audio_device())
        self.assertEqual(Vector3d.UNIT_X, world2.gravity())

        # self.assertTrue(None != world2.Gui())
        # self.assertEqual(gui.Fullscreen(), world2.Gui().Fullscreen())
        #
        # self.assertTrue(None != world2.Scene())
        # self.assertEqual(scene.Grid(), world2.Scene().Grid())

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

#
# ########################/
# TEST(DOMWorld, SetGui)
# {
#   sdf::Gui gui
#   gui.set_Fullscreen(true)
#
#   world = World()
#   self.assertEqual(None, world.Gui())
#
#   world.set_Gui(gui)
#   self.assertNotEqual(None, world.Gui())
#   self.assertTrue(world.Gui().Fullscreen())
# }
#
# ########################/
# TEST(DOMWorld, SetScene)
# {
#   world = World()
#   self.assertEqual(None, world.Scene())
#
#   sdf::Scene scene
#   scene.set_Ambient(Color::Blue)
#   scene.set_Background(Color::Red)
#   scene.set_Grid(true)
#   scene.set_Shadows(true)
#   scene.set_OriginVisual(true)
#   world.set_Scene(scene)
#
#   self.assertNotEqual(None, world.Scene())
#   self.assertEqual(Color::Blue, world.Scene().Ambient())
#   self.assertEqual(Color::Red, world.Scene().Background())
#   self.assertTrue(world.Scene().Grid())
#   self.assertTrue(world.Scene().Shadows())
#   self.assertTrue(world.Scene().OriginVisual())
# }

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
        #
        # sdf::Physics physics
        # physics.set_name("physics1")
        # self.assertTrue(world.add_Physics(physics))

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(world.add_frame(frame))

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

        # # Modify the physics
        # sdf::Physics *p = world.physics_by_index(1)
        # self.assertNotEqual(None, p)
        # self.assertEqual("physics1", p.name())
        # p.set_name("physics2")
        # self.assertEqual("physics2", world.physics_by_index(1).name())

        # Modify the frame
        f = world.frame_by_index(0)
        self.assertNotEqual(None, f)
        self.assertEqual("frame1", f.name())
        f.set_name("frame2")
        self.assertEqual("frame2", world.frame_by_index(0).name())


    def test_mutable_by_name(self):
        world = World()

        model = Model()
        model.set_name("model1")
        self.assertTrue(world.add_model(model))

        frame = Frame()
        frame.set_name("frame1")
        self.assertTrue(world.add_frame(frame))

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
#
# ########################/
# TEST(DOMWorld, Plugins)
# {
#   world = World()
#   self.assertTrue(world.Plugins().empty())
#
#   sdf::Plugin plugin
#   plugin.set_name("name1")
#   plugin.set_Filename("filename1")
#
#   world.add_Plugin(plugin)
#   ASSERT_EQ(1, world.Plugins().size())
#
#   plugin.set_name("name2")
#   world.add_Plugin(plugin)
#   ASSERT_EQ(2, world.Plugins().size())
#
#   self.assertEqual("name1", world.Plugins()[0].name())
#   self.assertEqual("name2", world.Plugins()[1].name())
#
#   world.clear_Plugins()
#   self.assertTrue(world.Plugins().empty())
# }
#


if __name__ == '__main__':
    unittest.main()
