# Copyright (C) 2026 Open Source Robotics Foundation

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
from gz.math import Vector3d
from sdformat import CustomInertiaCalcProperties, Element, Mesh
import unittest


class CustomInertiaCalcPropertiesTEST(unittest.TestCase):

    def test_default_construction(self):
        props = CustomInertiaCalcProperties()
        self.assertEqual(1000.0, props.density())
        self.assertIsNone(props.mesh())
        self.assertIsNone(props.auto_inertia_params())

    def test_construction(self):
        mesh = Mesh()
        mesh.set_uri("mesh.dae")
        mesh.set_scale(Vector3d(2, 2, 2))
        params = Element()
        params.set_name("auto_inertia_params")

        props = CustomInertiaCalcProperties(1240.0, mesh, params)
        self.assertEqual(1240.0, props.density())
        self.assertEqual("mesh.dae", props.mesh().uri())
        self.assertEqual(Vector3d(2, 2, 2), props.mesh().scale())
        self.assertEqual("auto_inertia_params",
                         props.auto_inertia_params().get_name())

    def test_set(self):
        props = CustomInertiaCalcProperties()

        props.set_density(1240.0)
        self.assertEqual(1240.0, props.density())

        mesh = Mesh()
        mesh.set_uri("mesh.dae")
        props.set_mesh(mesh)
        self.assertEqual("mesh.dae", props.mesh().uri())

        params = Element()
        params.set_name("auto_inertia_params")
        props.set_auto_inertia_params(params)
        self.assertEqual("auto_inertia_params",
                         props.auto_inertia_params().get_name())

    def test_copy_construction(self):
        props = CustomInertiaCalcProperties()
        props.set_density(1240.0)

        props2 = CustomInertiaCalcProperties(props)
        self.assertEqual(1240.0, props2.density())

        props2.set_density(500.0)
        self.assertEqual(1240.0, props.density())
        self.assertEqual(500.0, props2.density())

    def test_copy(self):
        props = CustomInertiaCalcProperties()
        props.set_density(1240.0)

        props2 = copy.copy(props)
        self.assertEqual(1240.0, props2.density())

        props2.set_density(500.0)
        self.assertEqual(1240.0, props.density())
        self.assertEqual(500.0, props2.density())

    def test_deepcopy(self):
        props = CustomInertiaCalcProperties()
        props.set_density(1240.0)
        mesh = Mesh()
        mesh.set_uri("mesh.dae")
        props.set_mesh(mesh)

        props2 = copy.deepcopy(props)
        self.assertEqual(1240.0, props2.density())
        self.assertEqual("mesh.dae", props2.mesh().uri())

        props2.set_density(500.0)
        self.assertEqual(1240.0, props.density())
        self.assertEqual(500.0, props2.density())


if __name__ == '__main__':
    unittest.main()
