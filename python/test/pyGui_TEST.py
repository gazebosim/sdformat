# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from sdformat import Gui, Plugin
import unittest


class GuiTEST(unittest.TestCase):

    def test_default_construction(self):
        gui = Gui()
        self.assertFalse(gui.fullscreen())


    def test_copy_construction(self):
        gui = Gui()
        gui.set_fullscreen(True)
        self.assertTrue(gui.fullscreen())

        gui2 = Gui(gui)
        self.assertTrue(gui2.fullscreen())


    def test_set(self):
        gui = Gui()
        gui.set_fullscreen(True)
        self.assertTrue(gui.fullscreen())


    def test_equal(self):
        gui = Gui()
        gui.set_fullscreen(True)
        gui2 = Gui(gui)

        self.assertTrue(gui == gui2)
        gui.set_fullscreen(False)
        self.assertFalse(gui == gui2)


    def test_plugin(self):
        gui = Gui()

        gui.set_fullscreen(True)

        for j in range(2):
            for i in range(3):
                plugin = Plugin()
                plugin.set_name("name" + str(i))
                plugin.set_filename("filename" + str(i))
                gui.add_plugin(plugin)
                gui.add_plugin(plugin)
            if j == 0:
              self.assertEqual(6, gui.plugin_count())
              self.assertEqual(6, len(gui.plugins()))
              gui.clear_plugins()
              self.assertEqual(0, gui.plugin_count())
              self.assertEqual(0, len(gui.plugins()))

        self.assertEqual(6, gui.plugin_count())

if __name__ == '__main__':
    unittest.main()
