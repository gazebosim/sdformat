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
#include "sdf/Surface.hh"

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
  ode.SetFdir1(ignition::math::Vector3d(1, 2, 3));
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
            ignition::math::Vector3d(1, 2, 3));
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
  ode.SetFdir1(ignition::math::Vector3d(1, 2, 3));
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
            ignition::math::Vector3d(1, 2, 3));
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
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::ODE ode2;
  ode2.SetMu(0.2);
  ode2.SetMu2(0.1);
  ode2.SetSlip1(7);
  ode2.SetSlip2(8);
  ode2.SetFdir1(ignition::math::Vector3d(3, 2, 1));
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
            ignition::math::Vector3d(3, 2, 1));
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(surface2.Friction()->ODE()->Slip2(), 4);
  EXPECT_EQ(surface2.Friction()->ODE()->Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
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
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);
  EXPECT_EQ(nullptr, friction1.Element());
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip2(), 4);
  EXPECT_EQ(friction1.ODE()->Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMfriction, CopyOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::Friction friction2(friction1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip2(), 4);
  EXPECT_EQ(friction2.ODE()->Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMfriction, CopyAssignmentOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::Friction friction2 = friction1;
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip2(), 4);
  EXPECT_EQ(friction2.ODE()->Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMfriction, CopyAssignmentAfterMove)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  sdf::Friction friction1;
  friction1.SetODE(ode1);

  sdf::ODE ode2;
  ode2.SetMu(0.2);
  ode2.SetMu2(0.1);
  ode2.SetSlip1(7);
  ode2.SetSlip2(8);
  ode2.SetFdir1(ignition::math::Vector3d(3, 2, 1));
  sdf::Friction friction2;
  friction2.SetODE(ode2);

  sdf::Friction tmp = std::move(friction1);
  friction1 = friction2;
  friction2 = tmp;

  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu(), 0.2);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu2(), 0.1);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip1(), 7);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip2(), 8);
  EXPECT_EQ(friction1.ODE()->Fdir1(),
            ignition::math::Vector3d(3, 2, 1));
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction2.ODE()->Slip2(), 4);
  EXPECT_EQ(friction2.ODE()->Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMfriction, Set)
{
  sdf::ODE ode1;
  sdf::Friction friction1;

  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu(), 1.0);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu2(), 1.0);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip1(), 0);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip2(), 0);
  EXPECT_EQ(friction1.ODE()->Fdir1(),
            ignition::math::Vector3d(0, 0, 0));

  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  friction1.SetODE(ode1);

  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu(), 0.1);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip1(), 3);
  EXPECT_DOUBLE_EQ(friction1.ODE()->Slip2(), 4);
  EXPECT_EQ(friction1.ODE()->Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
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
            ignition::math::Vector3d(0, 0, 0));
}

/////////////////////////////////////////////////
TEST(DOMode, CopyOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));

  sdf::ODE ode2(ode1);
  EXPECT_DOUBLE_EQ(ode2.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode2.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode2.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode2.Slip2(), 4);
  EXPECT_EQ(ode2.Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMode, CopyAssignmentOperator)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));

  sdf::ODE ode2 = ode1;
  EXPECT_DOUBLE_EQ(ode2.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode2.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode2.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode2.Slip2(), 4);
  EXPECT_EQ(ode2.Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(DOMode, CopyAssignmentAfterMove)
{
  sdf::ODE ode1;
  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));

  sdf::ODE ode2;
  ode2.SetMu(0.2);
  ode2.SetMu2(0.1);
  ode2.SetSlip1(7);
  ode2.SetSlip2(8);
  ode2.SetFdir1(ignition::math::Vector3d(3, 2, 1));

  sdf::ODE tmp = std::move(ode1);
  ode1 = ode2;
  ode2 = tmp;

  EXPECT_DOUBLE_EQ(ode1.Mu(), 0.2);
  EXPECT_DOUBLE_EQ(ode1.Mu2(), 0.1);
  EXPECT_DOUBLE_EQ(ode1.Slip1(), 7);
  EXPECT_DOUBLE_EQ(ode1.Slip2(), 8);
  EXPECT_EQ(ode1.Fdir1(),
            ignition::math::Vector3d(3, 2, 1));
  EXPECT_DOUBLE_EQ(ode2.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode2.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode2.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode2.Slip2(), 4);
  EXPECT_EQ(ode2.Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
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
            ignition::math::Vector3d(0, 0, 0));

  ode1.SetMu(0.1);
  ode1.SetMu2(0.2);
  ode1.SetSlip1(3);
  ode1.SetSlip2(4);
  ode1.SetFdir1(ignition::math::Vector3d(1, 2, 3));
  EXPECT_DOUBLE_EQ(ode1.Mu(), 0.1);
  EXPECT_DOUBLE_EQ(ode1.Mu2(), 0.2);
  EXPECT_DOUBLE_EQ(ode1.Slip1(), 3);
  EXPECT_DOUBLE_EQ(ode1.Slip2(), 4);
  EXPECT_EQ(ode1.Fdir1(),
            ignition::math::Vector3d(1, 2, 3));
}
