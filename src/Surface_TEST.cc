/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <gz/math/Vector3.hh>
#include "sdf/Surface.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMsurface, DefaultConstruction)
{
  sdf::Surface surface;
  EXPECT_EQ(nullptr, surface.Element());
  EXPECT_EQ(surface.Contact()->CollideBitmask(), 0xFF);
  EXPECT_EQ(surface.Contact()->Element(), nullptr);
  EXPECT_EQ(surface.Friction()->Element(), nullptr);
}

/////////////////////////////////////////////////
TEST(DOMsurface, CopyOperator)
{
  sdf::Surface surface1;
  sdf::Contact contact;
  sdf::ODE ode;
  ode.SetMu(0.1);
  ode.SetMu2(0.2);
  ode.SetSlip1(3);
  ode.SetSlip2(4);
  ode.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction;
  friction.SetODE(ode);
  contact.SetCollideBitmask(0x12);
  surface1.SetContact(contact);
  surface1.SetFriction(friction);

  sdf::Surface surface2(surface1);
  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip2(), 4);
  EXPECT_EQ(surface2.Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMsurface, CopyAssignmentOperator)
{
  sdf::Surface surface1;
  sdf::Contact contact;
  sdf::ODE ode;
  ode.SetMu(0.1);
  ode.SetMu2(0.2);
  ode.SetSlip1(3);
  ode.SetSlip2(4);
  ode.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction;
  friction.SetODE(ode);
  contact.SetCollideBitmask(0x12);
  surface1.SetContact(contact);
  surface1.SetFriction(friction);

  sdf::Surface surface2 = surface1;
  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip2(), 4);
  EXPECT_EQ(surface2.Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMsurface, CopyAssignmentAfterMove)
{
  sdf::Surface surface1;
  sdf::Surface surface2;

  sdf::Contact contact1;
  contact1.SetCollideBitmask(0x12);
  surface1.SetContact(contact1);
  sdf::Contact contact2;
  contact2.SetCollideBitmask(0x34);
  surface2.SetContact(contact2);

  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::ODE ode2;
  ode2.SetMu(0.2);
  ode2.SetMu2(0.1);
  ode2.SetSlip1(7);
  ode2.SetSlip2(8);
  ode2.SetFdir1(gz::math::Vector3d(3, 2, 1));
  sdf::Friction friction2;
  friction2.SetODE(ode2);

  surface1.SetFriction(friction1);
  surface2.SetFriction(friction2);

  sdf::Surface tmp = std::move(surface1);
  surface1 = surface2;
  surface2 = tmp;

  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
  EXPECT_EQ(surface1.Contact()->CollideBitmask(), 0x34);
  EXPECT_DOUBLE_EQ(surface1.Friction()->ODE()->Mu(), 0.2);
  EXPECT_DOUBLE_EQ(surface1.Friction()->ODE()->Mu2(), 0.1);
  EXPECT_DOUBLE_EQ(surface1.Friction()->ODE()->Slip1(), 7);
  EXPECT_DOUBLE_EQ(surface1.Friction()->ODE()->Slip2(), 8);
  EXPECT_EQ(surface1.Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(3, 2, 1));
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip2(), 4);
  EXPECT_EQ(surface2.Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMsurface, ToElement)
{
  sdf::Contact contact;
  sdf::Friction friction;
  sdf::Surface surface1;
  sdf::ODE ode;
  ode.SetMu(0.1);
  ode.SetMu2(0.2);
  ode.SetSlip1(3);
  ode.SetSlip2(4);
  ode.SetFdir1(gz::math::Vector3d(1, 2, 3));
  friction.SetODE(ode);
  sdf::BulletFriction bullet;
  bullet.SetFriction(0.3);
  bullet.SetFriction2(0.5);
  bullet.SetFdir1(gz::math::Vector3d(2, 1, 4));
  bullet.SetRollingFriction(1.3);
  friction.SetBulletFriction(bullet);
  sdf::Torsional torsional;
  torsional.SetCoefficient(0.5);
  torsional.SetUsePatchRadius(false);
  torsional.SetPatchRadius(0.1);
  torsional.SetSurfaceRadius(0.3);
  torsional.SetODESlip(0.2);
  friction.SetTorsional(torsional);
  contact.SetCollideBitmask(0x12);
  surface1.SetContact(contact);
  surface1.SetFriction(friction);

  sdf::ElementPtr elem = surface1.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Surface surface2;
  surface2.Load(elem);

  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip2(), 4);
  EXPECT_EQ(surface2.Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));

  EXPECT_DOUBLE_EQ(0.3, surface2.Friction()->BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(0.5, surface2.Friction()->BulletFriction()->Friction2());
  EXPECT_EQ(gz::math::Vector3d(2, 1, 4),
            surface2.Friction()->BulletFriction()->Fdir1());
  EXPECT_DOUBLE_EQ(1.3,
                  surface2.Friction()->BulletFriction()->RollingFriction());

  EXPECT_DOUBLE_EQ(0.5, surface2.Friction()->Torsional()->Coefficient());
  EXPECT_FALSE(surface2.Friction()->Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.1, surface2.Friction()->Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(0.3, surface2.Friction()->Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.2, surface2.Friction()->Torsional()->ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMsurface, ToElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  sdf::Errors errors;
  sdf::Contact contact;
  sdf::Friction friction;
  sdf::Surface surface1;
  sdf::ODE ode;
  ode.SetMu(0.1);
  ode.SetMu2(0.2);
  ode.SetSlip1(3);
  ode.SetSlip2(4);
  ode.SetFdir1(gz::math::Vector3d(1, 2, 3));
  friction.SetODE(ode);
  contact.SetCollideBitmask(0x12);
  surface1.SetContact(contact);
  surface1.SetFriction(friction);

  sdf::ElementPtr elem = surface1.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Surface surface2;
  errors = surface2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(surface2.Contact()->CollideBitmask(), 0x12);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip2(), 4);
  EXPECT_EQ(surface2.Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();

}

/////////////////////////////////////////////////
TEST(DOMcontact, DefaultConstruction)
{
  sdf::Contact contact;
  EXPECT_EQ(nullptr, contact.Element());
  EXPECT_EQ(contact.CollideBitmask(), 0xFF);
  EXPECT_EQ(contact.Element(), nullptr);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CopyOperator)
{
  sdf::Contact contact1;
  contact1.SetCollideBitmask(0x12);

  sdf::Contact contact2(contact1);
  EXPECT_EQ(contact2.CollideBitmask(), 0x12);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CopyAssignmentOperator)
{
  sdf::Contact contact1;
  contact1.SetCollideBitmask(0x12);

  sdf::Contact contact2 = contact1;
  EXPECT_EQ(contact2.CollideBitmask(), 0x12);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CopyAssignmentAfterMove)
{
  sdf::Contact contact1;
  sdf::Contact contact2;

  contact1.SetCollideBitmask(0x12);
  contact2.SetCollideBitmask(0x34);

  sdf::Contact tmp = std::move(contact1);
  contact1 = contact2;
  contact2 = tmp;

  EXPECT_EQ(contact2.CollideBitmask(), 0x12);
  EXPECT_EQ(contact1.CollideBitmask(), 0x34);
}

/////////////////////////////////////////////////
TEST(DOMcontact, CollideBitmask)
{
  sdf::Contact contact;
  contact.SetCollideBitmask(0x67);
  EXPECT_EQ(contact.CollideBitmask(), 0x67);
}

/////////////////////////////////////////////////
TEST(DOMfriction, SetFriction)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::BulletFriction bullet;
  bullet.SetFriction(0.3);
  bullet.SetFriction2(0.5);
  bullet.SetFdir1(gz::math::Vector3d(2, 1, 4));
  bullet.SetRollingFriction(1.3);
  friction1.SetBulletFriction(bullet);

  sdf::Torsional torsional;
  torsional.SetCoefficient(0.5);
  torsional.SetUsePatchRadius(false);
  torsional.SetPatchRadius(0.1);
  torsional.SetSurfaceRadius(0.3);
  torsional.SetODESlip(0.2);
  friction1.SetTorsional(torsional);

  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip2(), 4);
  EXPECT_EQ(friction1.ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));

  EXPECT_DOUBLE_EQ(0.3, friction1.BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(0.5, friction1.BulletFriction()->Friction2());
  EXPECT_EQ(gz::math::Vector3d(2, 1, 4), friction1.BulletFriction()->Fdir1());
  EXPECT_DOUBLE_EQ(1.3, friction1.BulletFriction()->RollingFriction());

  EXPECT_DOUBLE_EQ(0.5, friction1.Torsional()->Coefficient());
  EXPECT_FALSE(friction1.Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.1, friction1.Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(0.3, friction1.Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.2, friction1.Torsional()->ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMfriction, CopyOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::BulletFriction bullet;
  bullet.SetFriction(0.3);
  bullet.SetFriction2(0.5);
  bullet.SetFdir1(gz::math::Vector3d(2, 1, 4));
  bullet.SetRollingFriction(1.3);
  friction1.SetBulletFriction(bullet);

  sdf::Torsional torsional;
  torsional.SetCoefficient(0.5);
  torsional.SetUsePatchRadius(false);
  torsional.SetPatchRadius(0.1);
  torsional.SetSurfaceRadius(0.3);
  torsional.SetODESlip(0.2);
  friction1.SetTorsional(torsional);

  sdf::Friction friction2(friction1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip2(), 4);
  EXPECT_EQ(friction2.ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));

  EXPECT_DOUBLE_EQ(0.3, friction2.BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(0.5, friction2.BulletFriction()->Friction2());
  EXPECT_EQ(gz::math::Vector3d(2, 1, 4), friction2.BulletFriction()->Fdir1());
  EXPECT_DOUBLE_EQ(1.3, friction2.BulletFriction()->RollingFriction());

  EXPECT_DOUBLE_EQ(0.5, friction2.Torsional()->Coefficient());
  EXPECT_FALSE(friction2.Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.1, friction2.Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(0.3, friction2.Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.2, friction2.Torsional()->ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMfriction, CopyAssignmentOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::BulletFriction bullet;
  bullet.SetFriction(0.3);
  bullet.SetFriction2(0.5);
  bullet.SetFdir1(gz::math::Vector3d(2, 1, 4));
  bullet.SetRollingFriction(1.3);
  friction1.SetBulletFriction(bullet);

  sdf::Torsional torsional;
  torsional.SetCoefficient(0.5);
  torsional.SetUsePatchRadius(false);
  torsional.SetPatchRadius(0.1);
  torsional.SetSurfaceRadius(0.3);
  torsional.SetODESlip(0.2);
  friction1.SetTorsional(torsional);

  sdf::Friction friction2 = friction1;
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip2(), 4);
  EXPECT_EQ(friction2.ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));

  EXPECT_DOUBLE_EQ(0.3, friction2.BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(0.5, friction2.BulletFriction()->Friction2());
  EXPECT_EQ(gz::math::Vector3d(2, 1, 4), friction2.BulletFriction()->Fdir1());
  EXPECT_DOUBLE_EQ(1.3, friction2.BulletFriction()->RollingFriction());

  EXPECT_DOUBLE_EQ(0.5, friction2.Torsional()->Coefficient());
  EXPECT_FALSE(friction2.Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.1, friction2.Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(0.3, friction2.Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.2, friction2.Torsional()->ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMfriction, CopyAssignmentAfterMove)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::ODE ode2;
  ode2.SetMu(0.2);
  ode2.SetMu2(0.1);
  ode2.SetSlip1(7);
  ode2.SetSlip2(8);
  ode2.SetFdir1(gz::math::Vector3d(3, 2, 1));
  sdf::Friction friction2;
  friction2.SetODE(ode2);

  sdf::BulletFriction bullet1;
  bullet1.SetFriction(0.3);
  bullet1.SetFriction2(0.5);
  bullet1.SetFdir1(gz::math::Vector3d(2, 1, 4));
  bullet1.SetRollingFriction(1.3);
  friction1.SetBulletFriction(bullet1);

  sdf::BulletFriction bullet2;
  bullet2.SetFriction(0.1);
  bullet2.SetFriction2(0.2);
  bullet2.SetFdir1(gz::math::Vector3d(3, 4, 5));
  bullet2.SetRollingFriction(3.1);
  friction2.SetBulletFriction(bullet2);

  sdf::Torsional torsional1;
  torsional1.SetCoefficient(0.5);
  torsional1.SetUsePatchRadius(false);
  torsional1.SetPatchRadius(0.1);
  torsional1.SetSurfaceRadius(0.3);
  torsional1.SetODESlip(0.2);
  friction1.SetTorsional(torsional1);

  sdf::Torsional torsional2;
  torsional2.SetCoefficient(1.5);
  torsional2.SetUsePatchRadius(true);
  torsional2.SetPatchRadius(1.1);
  torsional2.SetSurfaceRadius(3.3);
  torsional2.SetODESlip(2.2);
  friction2.SetTorsional(torsional2);

  sdf::Friction tmp = std::move(friction1);
  friction1 = friction2;
  friction2 = tmp;

  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu(), 0.2);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu2(), 0.1);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip1(), 7);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip2(), 8);
  EXPECT_EQ(friction1.ODE()->Fdir1(),
            gz::math::Vector3d(3, 2, 1));
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip2(), 4);
  EXPECT_EQ(friction2.ODE()->Fdir1(),
            gz::math::Vector3d(1, 2, 3));

  EXPECT_DOUBLE_EQ(0.3, friction2.BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(0.5, friction2.BulletFriction()->Friction2());
  EXPECT_EQ(gz::math::Vector3d(2, 1, 4), friction2.BulletFriction()->Fdir1());
  EXPECT_DOUBLE_EQ(1.3, friction2.BulletFriction()->RollingFriction());
  EXPECT_DOUBLE_EQ(0.1, friction1.BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(0.2, friction1.BulletFriction()->Friction2());
  EXPECT_EQ(gz::math::Vector3d(3, 4, 5), friction1.BulletFriction()->Fdir1());
  EXPECT_DOUBLE_EQ(3.1, friction1.BulletFriction()->RollingFriction());

  EXPECT_DOUBLE_EQ(0.5, friction2.Torsional()->Coefficient());
  EXPECT_FALSE(friction2.Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.1, friction2.Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(0.3, friction2.Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.2, friction2.Torsional()->ODESlip());
  EXPECT_DOUBLE_EQ(1.5, friction1.Torsional()->Coefficient());
  EXPECT_TRUE(friction1.Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(1.1, friction1.Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(3.3, friction1.Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(2.2, friction1.Torsional()->ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMode, DefaultValues)
{
  sdf::ODE ode;
  EXPECT_EQ(nullptr, ode.Element());
  EXPECT_DOUBLE_EQ(ode.Mu(), 1.0);
  EXPECT_DOUBLE_EQ(ode.Mu2(), 1.0);
  EXPECT_DOUBLE_EQ(ode.Slip1(), 0);
  EXPECT_DOUBLE_EQ(ode.Slip2(), 0);
  EXPECT_EQ(ode.Fdir1(),
            gz::math::Vector3d(0, 0, 0));
}

/////////////////////////////////////////////////
TEST(DOMode, CopyOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));

  sdf::ODE ode2(ode1);
  EXPECT_DOUBLE_EQ(ode2.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode2.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode2.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode2.Slip2(), 4);
  EXPECT_EQ(ode2.Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMode, CopyAssignmentOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));

  sdf::ODE ode2 = ode1;
  EXPECT_DOUBLE_EQ(ode2.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode2.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode2.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode2.Slip2(), 4);
  EXPECT_EQ(ode2.Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMode, CopyAssignmentAfterMove)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));

  sdf::ODE ode2;
  ode2.SetMu(0.2);
  ode2.SetMu2(0.1);
  ode2.SetSlip1(7);
  ode2.SetSlip2(8);
  ode2.SetFdir1(gz::math::Vector3d(3, 2, 1));

  sdf::ODE tmp = std::move(ode1);
  ode1 = ode2;
  ode2 = tmp;

  EXPECT_DOUBLE_EQ(ode1.Mu(), 0.2);
  EXPECT_DOUBLE_EQ(ode1.Mu2(), 0.1);
  EXPECT_DOUBLE_EQ(ode1.Slip1(), 7);
  EXPECT_DOUBLE_EQ(ode1.Slip2(), 8);
  EXPECT_EQ(ode1.Fdir1(),
            gz::math::Vector3d(3, 2, 1));
  EXPECT_DOUBLE_EQ(ode2.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode2.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode2.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode2.Slip2(), 4);
  EXPECT_EQ(ode2.Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMode, Set)
{
  sdf::ODE ode1;

  EXPECT_DOUBLE_EQ(ode1.Mu(), 1.0);
  EXPECT_DOUBLE_EQ(ode1.Mu2(), 1.0);
  EXPECT_DOUBLE_EQ(ode1.Slip1(), 0);
  EXPECT_DOUBLE_EQ(ode1.Slip2(), 0);
  EXPECT_EQ(ode1.Fdir1(),
            gz::math::Vector3d(0, 0, 0));

  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  EXPECT_DOUBLE_EQ(ode1.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode1.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode1.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode1.Slip2(), 4);
  EXPECT_EQ(ode1.Fdir1(),
            gz::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMbullet, DefaultValues)
{
  sdf::BulletFriction bullet;
  EXPECT_EQ(nullptr, bullet.Element());
  EXPECT_DOUBLE_EQ(1.0, bullet.Friction());
  EXPECT_DOUBLE_EQ(1.0, bullet.Friction2());
  EXPECT_EQ(gz::math::Vector3d(0, 0, 0), bullet.Fdir1());
  EXPECT_DOUBLE_EQ(1.0, bullet.RollingFriction());
}

/////////////////////////////////////////////////
TEST(DOMbullet, CopyOperator)
{
  sdf::BulletFriction bullet1;
  bullet1.SetFriction(0.1);
  bullet1.SetFriction2(0.2);
  bullet1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  bullet1.SetRollingFriction(4.0);

  sdf::BulletFriction bullet2(bullet1);
  EXPECT_DOUBLE_EQ(0.1, bullet2.Friction());
  EXPECT_DOUBLE_EQ(0.2, bullet2.Friction2());
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), bullet2.Fdir1());
  EXPECT_DOUBLE_EQ(4.0, bullet2.RollingFriction());
}

/////////////////////////////////////////////////
TEST(DOMbullet, CopyAssignmentOperator)
{
  sdf::BulletFriction bullet1;
  bullet1.SetFriction(0.1);
  bullet1.SetFriction2(0.2);
  bullet1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  bullet1.SetRollingFriction(4.0);

  sdf::BulletFriction bullet2 = bullet1;
  EXPECT_DOUBLE_EQ(0.1, bullet2.Friction());
  EXPECT_DOUBLE_EQ(0.2, bullet2.Friction2());
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), bullet2.Fdir1());
  EXPECT_DOUBLE_EQ(4.0, bullet2.RollingFriction());
}

/////////////////////////////////////////////////
TEST(DOMbullet, CopyAssignmentAfterMove)
{
  sdf::BulletFriction bullet1;
  bullet1.SetFriction(0.1);
  bullet1.SetFriction2(0.2);
  bullet1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  bullet1.SetRollingFriction(4.0);

  sdf::BulletFriction bullet2;
  bullet2.SetFriction(0.2);
  bullet2.SetFriction2(0.1);
  bullet2.SetFdir1(gz::math::Vector3d(3, 2, 1));
  bullet2.SetRollingFriction(3.0);

  sdf::BulletFriction tmp = std::move(bullet1);
  bullet1 = bullet2;
  bullet2 = tmp;

  EXPECT_DOUBLE_EQ(0.2, bullet1.Friction());
  EXPECT_DOUBLE_EQ(0.1, bullet1.Friction2());
  EXPECT_EQ(gz::math::Vector3d(3, 2, 1), bullet1.Fdir1());
  EXPECT_DOUBLE_EQ(3.0, bullet1.RollingFriction());

  EXPECT_DOUBLE_EQ(0.1, bullet2.Friction());
  EXPECT_DOUBLE_EQ(0.2, bullet2.Friction2());
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), bullet2.Fdir1());
  EXPECT_DOUBLE_EQ(4.0, bullet2.RollingFriction());
}

/////////////////////////////////////////////////
TEST(DOMbullet, Set)
{
  sdf::BulletFriction bullet1;

  EXPECT_DOUBLE_EQ(bullet1.Friction(), 1.0);
  EXPECT_DOUBLE_EQ(bullet1.Friction2(), 1.0);
  EXPECT_EQ(bullet1.Fdir1(),
            gz::math::Vector3d(0, 0, 0));
  EXPECT_DOUBLE_EQ(bullet1.RollingFriction(), 1.0);

  bullet1.SetFriction(0.1);
  bullet1.SetFriction2(0.2);
  bullet1.SetFdir1(gz::math::Vector3d(1, 2, 3));
  bullet1.SetRollingFriction(4);

  EXPECT_DOUBLE_EQ(0.1, bullet1.Friction());
  EXPECT_DOUBLE_EQ(0.2, bullet1.Friction2());
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), bullet1.Fdir1());
  EXPECT_DOUBLE_EQ(4.0, bullet1.RollingFriction());
}

/////////////////////////////////////////////////
TEST(DOMtorsional, DefaultValues)
{
  sdf::Torsional torsional;
  EXPECT_EQ(nullptr, torsional.Element());
  EXPECT_DOUBLE_EQ(1.0, torsional.Coefficient());
  EXPECT_TRUE(torsional.UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.0, torsional.PatchRadius());
  EXPECT_DOUBLE_EQ(0.0, torsional.SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.0, torsional.ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMtorsional, CopyOperator)
{
  sdf::Torsional torsional1;
  torsional1.SetCoefficient(0.1);
  torsional1.SetUsePatchRadius(false);
  torsional1.SetPatchRadius(0.2);
  torsional1.SetSurfaceRadius(4.0);
  torsional1.SetODESlip(1.0);

  sdf::Torsional torsional2(torsional1);
  EXPECT_DOUBLE_EQ(0.1, torsional2.Coefficient());
  EXPECT_FALSE(torsional2.UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.2, torsional2.PatchRadius());
  EXPECT_DOUBLE_EQ(4.0, torsional2.SurfaceRadius());
  EXPECT_DOUBLE_EQ(1.0, torsional2.ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMtorsional, CopyAssignmentOperator)
{
  sdf::Torsional torsional1;
  torsional1.SetCoefficient(0.1);
  torsional1.SetUsePatchRadius(false);
  torsional1.SetPatchRadius(0.2);
  torsional1.SetSurfaceRadius(4.0);
  torsional1.SetODESlip(1.0);

  sdf::Torsional torsional2 = torsional1;
  EXPECT_DOUBLE_EQ(0.1, torsional2.Coefficient());
  EXPECT_FALSE(torsional2.UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.2, torsional2.PatchRadius());
  EXPECT_DOUBLE_EQ(4.0, torsional2.SurfaceRadius());
  EXPECT_DOUBLE_EQ(1.0, torsional2.ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMtorsional, CopyAssignmentAfterMove)
{
  sdf::Torsional torsional1;
  torsional1.SetCoefficient(0.1);
  torsional1.SetUsePatchRadius(false);
  torsional1.SetPatchRadius(0.2);
  torsional1.SetSurfaceRadius(4.0);
  torsional1.SetODESlip(1.0);

  sdf::Torsional torsional2;
  torsional2.SetCoefficient(1.1);
  torsional2.SetUsePatchRadius(true);
  torsional2.SetPatchRadius(1.2);
  torsional2.SetSurfaceRadius(4.1);
  torsional2.SetODESlip(1.1);

  sdf::Torsional tmp = std::move(torsional1);
  torsional1 = torsional2;
  torsional2 = tmp;

  EXPECT_DOUBLE_EQ(0.1, torsional2.Coefficient());
  EXPECT_FALSE(torsional2.UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.2, torsional2.PatchRadius());
  EXPECT_DOUBLE_EQ(4.0, torsional2.SurfaceRadius());
  EXPECT_DOUBLE_EQ(1.0, torsional2.ODESlip());

  EXPECT_DOUBLE_EQ(1.1, torsional1.Coefficient());
  EXPECT_TRUE(torsional1.UsePatchRadius());
  EXPECT_DOUBLE_EQ(1.2, torsional1.PatchRadius());
  EXPECT_DOUBLE_EQ(4.1, torsional1.SurfaceRadius());
  EXPECT_DOUBLE_EQ(1.1, torsional1.ODESlip());
}

/////////////////////////////////////////////////
TEST(DOMtorsional, Set)
{
  sdf::Torsional torsional1;

  EXPECT_DOUBLE_EQ(1.0, torsional1.Coefficient());
  EXPECT_TRUE(torsional1.UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.0, torsional1.PatchRadius());
  EXPECT_DOUBLE_EQ(0.0, torsional1.SurfaceRadius());
  EXPECT_DOUBLE_EQ(0.0, torsional1.ODESlip());

  torsional1.SetCoefficient(0.1);
  torsional1.SetUsePatchRadius(false);
  torsional1.SetPatchRadius(0.2);
  torsional1.SetSurfaceRadius(4);
  torsional1.SetODESlip(1.0);

  EXPECT_DOUBLE_EQ(0.1, torsional1.Coefficient());
  EXPECT_FALSE(torsional1.UsePatchRadius());
  EXPECT_DOUBLE_EQ(0.2, torsional1.PatchRadius());
  EXPECT_DOUBLE_EQ(4.0, torsional1.SurfaceRadius());
  EXPECT_DOUBLE_EQ(1.0, torsional1.ODESlip());
}
