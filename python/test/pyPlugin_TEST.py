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
from gz_test_deps.sdformat import Plugin
import unittest


class PluginTEST(unittest.TestCase):

    def test_default_construction(self):
        plugin = Plugin()

        self.assertFalse(plugin.name())
        self.assertFalse(plugin.filename())

        plugin.set_name("my-plugin")
        self.assertEqual("my-plugin", plugin.name())

        plugin.set_filename("filename.so")
        self.assertEqual("filename.so", plugin.filename())

    def test_copy_construction(self):
        plugin = Plugin()
        plugin.set_name("pluginname")
        plugin.set_filename("filename")

        plugin2 = Plugin(plugin)
        self.assertEqual("pluginname", plugin2.name())
        self.assertEqual("filename", plugin2.filename())

        self.assertEqual("pluginname", plugin.name())
        self.assertEqual("filename", plugin.filename())

    def test_assigment(self):
        plugin = Plugin()
        plugin.set_name("pluginname")
        plugin.set_filename("filename")

        plugin2 = plugin
        self.assertEqual("pluginname", plugin2.name())
        self.assertEqual("filename", plugin2.filename())

        self.assertEqual("pluginname", plugin.name())
        self.assertEqual("filename", plugin.filename())

    def test_output_stream(self):
        plugin = Plugin()
        plugin.set_name("my-plugin")
        self.assertEqual("my-plugin", plugin.name())

        plugin.set_filename("filename.so")
        self.assertEqual("filename.so", plugin.filename())

        # The expected plugin output string.
        expected = """<plugin name='my-plugin' filename='filename.so'/>\n"""
        self.assertEqual(expected, str(plugin))

    def test_insert_string_content(self):
        plugin = Plugin("my-filename", "my-name",
          "<render_engine>ogre2</render_engine>")
        self.assertEqual("my-filename", plugin.filename())
        self.assertEqual("my-name", plugin.name())

        extraContent = """<with_attribute value='bar'>1.234</with_attribute>
        <sibling>hello</sibling>
        <with_children>
        <child1>goodbye</child1>
        <child2>goodbye</child2>
        </with_children>"""

        # Insert more content using a string
        self.assertTrue(plugin.insert_content(extraContent))

        completeContent = "  <render_engine>ogre2</render_engine>" + extraContent

        completePlugin = "<plugin name='my-name' filename='my-filename'>\n" + completeContent + "</plugin>\n"

        # Try out curly braces initialization
        plugin2 = Plugin(plugin.filename(), plugin.name(), completeContent)

        # Try to insert poorly formed XML, which should fail and not modify the
        # content.
        self.assertFalse(plugin2.insert_content("<a></b>"))

        # An empty string will also fail and not modify the content
        self.assertFalse(plugin2.insert_content(""))

        # Constructing a new plugin with no content
        plugin3 = Plugin("a filename", "a name")
        self.assertEqual("a filename", plugin3.filename())
        self.assertEqual("a name", plugin3.name())

        # Constructing a new plugin with bad XML content
        plugin4 = Plugin("filename", "name", "<garbage>")
        self.assertEqual("filename", plugin4.filename())
        self.assertEqual("name", plugin4.name())

        # Constructing a new plugin with bad XML content
        plugin5 = Plugin("filename", "name", "    ")
        self.assertEqual("filename", plugin5.filename())
        self.assertEqual("name", plugin5.name())

    def test_equality_operators(self):
        plugin = Plugin("my-filename", "my-name",
          "<render_engine>ogre2</render_engine>")
        plugin2 = Plugin(plugin)
        plugin3 = Plugin()

        self.assertEqual(plugin, plugin2)
        self.assertNotEqual(plugin, plugin3)
        self.assertNotEqual(plugin2, plugin3)

        # Test contents
        plugin2.clear_contents()
        self.assertNotEqual(plugin, plugin2)
        plugin.clear_contents()
        self.assertEqual(plugin, plugin2)

        # test name
        plugin2.set_name("new-name")
        self.assertNotEqual(plugin, plugin2)
        plugin.set_name("new-name")
        self.assertEqual(plugin, plugin2)

        # test filename
        plugin2.set_filename("new-filename")
        self.assertNotEqual(plugin, plugin2)
        plugin.set_filename("new-filename")
        self.assertEqual(plugin, plugin2)


if __name__ == '__main__':
    unittest.main()
