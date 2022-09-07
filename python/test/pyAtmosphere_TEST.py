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

from gz.math import Temperature
from sdformat import Atmosphere
import sdformat as sdf
import unittest

class AtmosphereTEST(unittest.TestCase):


  def test_default_construction(self):
    atmosphere = Atmosphere()
    self.assertEqual(sdf.AtmosphereType.ADIABATIC, atmosphere.type())
    self.assertAlmostEqual(288.15, atmosphere.temperature().kelvin())
    self.assertAlmostEqual(-0.0065, atmosphere.temperature_gradient())
    self.assertAlmostEqual(101325, atmosphere.pressure())


  def test_set(self):
    atmosphere = Atmosphere()
    atmosphere.set_type(sdf.AtmosphereType.ADIABATIC)
    self.assertEqual(sdf.AtmosphereType.ADIABATIC, atmosphere.type())

    atmosphere.set_temperature(Temperature(123.23))
    self.assertAlmostEqual(123.23, atmosphere.temperature().kelvin())

    atmosphere.set_temperature_gradient(-1.65)
    self.assertAlmostEqual(-1.65, atmosphere.temperature_gradient())

    atmosphere.set_pressure(76531.3)
    self.assertAlmostEqual(76531.3, atmosphere.pressure())


  def test_copy_assignment(self):
    atmosphere = Atmosphere()
    atmosphere.set_temperature(Temperature(123.23))
    atmosphere.set_temperature_gradient(-1.65)
    atmosphere.set_pressure(76531.3)

    atmosphere2 = atmosphere
    self.assertEqual(sdf.AtmosphereType.ADIABATIC, atmosphere2.type())
    self.assertAlmostEqual(123.23, atmosphere2.temperature().kelvin())
    self.assertAlmostEqual(-1.65, atmosphere2.temperature_gradient())
    self.assertAlmostEqual(76531.3, atmosphere2.pressure())

    self.assertEqual(atmosphere, atmosphere2)


if __name__ == '__main__':
    unittest.main()
