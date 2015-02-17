/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <set>

#include "urdf_model/model.h"
#include "urdf_model/link.h"
#include "urdf_parser/urdf_parser.h"

#include "sdf/parser_urdf.hh"
#include "sdf/sdf.hh"

using namespace sdf;

typedef boost::shared_ptr<urdf::Collision> UrdfCollisionPtr;
typedef boost::shared_ptr<urdf::Visual> UrdfVisualPtr;
typedef boost::shared_ptr<urdf::Link> UrdfLinkPtr;
typedef boost::shared_ptr<const urdf::Link> ConstUrdfLinkPtr;
typedef boost::shared_ptr<TiXmlElement> TiXmlElementPtr;
typedef boost::shared_ptr<SDFExtension> SDFExtensionPtr;
typedef std::map<std::string, std::vector<SDFExtensionPtr> >
  StringSDFExtensionPtrMap;

/// create SDF geometry block based on URDF
StringSDFExtensionPtrMap g_extensions;
bool g_reduceFixedJoints;
bool g_enforceLimits;
std::string g_collisionExt = "_collision";
std::string g_visualExt = "_visual";
urdf::Pose g_initialRobotPose;
bool g_initialRobotPoseValid = false;
std::set<std::string> g_fixedJointsNotReduced;

/// \brief parser xml string into urdf::Vector3
/// \param[in] _key XML key where vector3 value might be
/// \param[in] _scale scalar scale for the vector3
/// \return a urdf::Vector3
urdf::Vector3 ParseVector3(TiXmlNode* _key, double _scale = 1.0);
urdf::Vector3 ParseVector3(const std::string &_str, double _scale = 1.0);

/// insert extensions into collision geoms
void InsertSDFExtensionCollision(TiXmlElement *_elem,
    const std::string &_linkName);

/// insert extensions into model
void InsertSDFExtensionRobot(TiXmlElement *_elem);

/// insert extensions into visuals
void InsertSDFExtensionVisual(TiXmlElement *_elem,
    const std::string &_linkName);


/// insert extensions into joints
void InsertSDFExtensionJoint(TiXmlElement *_elem,
    const std::string &_jointName);

/// reduced fixed joints:  check if a fixed joint should be lumped
///   checking both the joint type and if disabledFixedJointLumping
///   option is set
bool FixedJointShouldBeReduced(boost::shared_ptr<urdf::Joint> _jnt);

/// reduced fixed joints:  apply transform reduction for ray sensors
///   in extensions when doing fixed joint reduction
void ReduceSDFExtensionSensorTransformReduction(
      std::vector<TiXmlElementPtr>::iterator _blobIt,
      sdf::Pose _reductionTransform);

/// reduced fixed joints:  apply transform reduction for projectors in
///   extensions when doing fixed joint reduction
void ReduceSDFExtensionProjectorTransformReduction(
      std::vector<TiXmlElementPtr>::iterator _blobIt,
      sdf::Pose _reductionTransform);


/// reduced fixed joints:  apply transform reduction to extensions
///   when doing fixed joint reduction
void ReduceSDFExtensionsTransform(SDFExtensionPtr _ge);

/// reduce fixed joints:  lump joints to parent link
void ReduceJointsToParent(UrdfLinkPtr _link);

/// reduce fixed joints:  lump collisions to parent link
void ReduceCollisionsToParent(UrdfLinkPtr _link);

/// reduce fixed joints:  lump visuals to parent link
void ReduceVisualsToParent(UrdfLinkPtr _link);

/// reduce fixed joints:  lump inertial to parent link
void ReduceInertialToParent(UrdfLinkPtr /*_link*/);

/// create SDF Collision block based on URDF
void CreateCollision(TiXmlElement* _elem, ConstUrdfLinkPtr _link,
      UrdfCollisionPtr _collision,
      const std::string &_oldLinkName = std::string(""));

/// create SDF Visual block based on URDF
void CreateVisual(TiXmlElement *_elem, ConstUrdfLinkPtr _link,
      UrdfVisualPtr _visual, const std::string &_oldLinkName = std::string(""));

/// create SDF Joint block based on URDF
void CreateJoint(TiXmlElement *_root, ConstUrdfLinkPtr _link,
      sdf::Pose &_currentTransform);

/// insert extensions into links
void InsertSDFExtensionLink(TiXmlElement *_elem, const std::string &_linkName);

/// create visual blocks from urdf visuals
void CreateVisuals(TiXmlElement* _elem, ConstUrdfLinkPtr _link);

/// create collision blocks from urdf collisions
void CreateCollisions(TiXmlElement* _elem, ConstUrdfLinkPtr _link);

/// create SDF Inertial block based on URDF
void CreateInertial(TiXmlElement *_elem, ConstUrdfLinkPtr _link);

/// append transform (pose) to the end of the xml element
void AddTransform(TiXmlElement *_elem, const sdf::Pose &_transform);

/// create SDF from URDF link
void CreateSDF(TiXmlElement *_root, ConstUrdfLinkPtr _link,
      const sdf::Pose &_transform);

/// create SDF Link block based on URDF
void CreateLink(TiXmlElement *_root, ConstUrdfLinkPtr _link,
      sdf::Pose &_currentTransform);


/// print collision groups for debugging purposes
void PrintCollisionGroups(UrdfLinkPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in joint
///   inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionJointFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in gripper
///   inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionGripperFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in projector
/// inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionProjectorFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in plugins
///   inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionPluginFrameReplace(
      std::vector<TiXmlElementPtr>::iterator _blobIt,
      UrdfLinkPtr _link, const std::string &_pluginName,
      const std::string &_elementName, sdf::Pose _reductionTransform);

/// reduced fixed joints:  apply appropriate frame updates in urdf
///   extensions when doing fixed joint reduction
void ReduceSDFExtensionContactSensorFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link);

/// \brief reduced fixed joints:  apply appropriate updates to urdf
///   extensions when doing fixed joint reduction
///
/// Take the link's existing list of gazebo extensions, transfer them
/// into parent link.  Along the way, update local transforms by adding
/// the additional transform to parent.  Also, look through all
/// referenced link names with plugins and update references to current
/// link to the parent link. (ReduceSDFExtensionFrameReplace())
///
/// \param[in] _link pointer to urdf link, its extensions will be reduced
void ReduceSDFExtensionToParent(UrdfLinkPtr _link);

/// reduced fixed joints:  apply appropriate frame updates
///   in urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionFrameReplace(SDFExtensionPtr _ge, UrdfLinkPtr _link);


/// get value from <key value="..."/> pair and return it as string
std::string GetKeyValueAsString(TiXmlElement* _elem);


/// \brief append key value pair to the end of the xml element
/// \param[in] _elem pointer to xml element
/// \param[in] _key string containing key to add to xml element
/// \param[in] _value string containing value for the key added
void AddKeyValue(TiXmlElement *_elem, const std::string &_key,
                     const std::string &_value);

/// \brief convert values to string
/// \param[in] _count number of values in _values array
/// \param[in] _values array of double values
/// \return a string
std::string Values2str(unsigned int _count, const double *_values);


void CreateGeometry(TiXmlElement* _elem,
    boost::shared_ptr<urdf::Geometry> _geometry);

std::string GetGeometryBoundingBox(boost::shared_ptr<urdf::Geometry> _geometry,
    double *_sizeVals);

sdf::Pose inverseTransformToParentFrame(sdf::Pose _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform);

/// reduced fixed joints: transform to parent frame
urdf::Pose TransformToParentFrame(urdf::Pose _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform);

/// reduced fixed joints: transform to parent frame
sdf::Pose TransformToParentFrame(sdf::Pose _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform);

/// reduced fixed joints: transform to parent frame
sdf::Pose TransformToParentFrame(sdf::Pose _transformInLinkFrame,
    sdf::Pose _parentToLinkTransform);

/// reduced fixed joints: utility to copy between urdf::Pose and
///   math::Pose
sdf::Pose CopyPose(urdf::Pose _pose);

/// reduced fixed joints: utility to copy between urdf::Pose and
///   math::Pose
urdf::Pose CopyPose(sdf::Pose _pose);

/////////////////////////////////////////////////
urdf::Vector3 ParseVector3(const std::string &_str, double _scale)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, _str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(_scale
            * boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << _str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return urdf::Vector3(0, 0, 0);
      }
    }
  }

  if (vals.size() == 3)
    return urdf::Vector3(vals[0], vals[1], vals[2]);
  else
    return urdf::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
urdf::Vector3 ParseVector3(TiXmlNode *_key, double _scale)
{
  if (_key != NULL)
  {
    TiXmlElement *key = _key->ToElement();
    if (key != NULL)
    {
      return ParseVector3(GetKeyValueAsString(key), _scale);
    }
  }
  sdferr << "key[" << _key->Value() << "] does not contain a Vector3\n";

  return urdf::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
/// \brief convert Vector3 to string
/// \param[in] _vector a urdf::Vector3
/// \return a string
std::string Vector32Str(const urdf::Vector3 _vector)
{
  std::stringstream ss;
  ss << _vector.x;
  ss << " ";
  ss << _vector.y;
  ss << " ";
  ss << _vector.z;
  return ss.str();
}

////////////////////////////////////////////////////////////////////////////////
void ReduceCollisionToParent(UrdfLinkPtr _link,
    const std::string &_groupName, UrdfCollisionPtr _collision)
{
  boost::shared_ptr<std::vector<UrdfCollisionPtr> > cols;
#if USE_EXTERNAL_URDF && defined(URDF_GE_0P3)
  if (_link->collision)
  {
    cols.reset(new std::vector<UrdfCollisionPtr>);
    cols->push_back(_link->collision);
  }
  else
  {
    cols = boost::shared_ptr<std::vector<UrdfCollisionPtr> >(
            &_link->collision_array);
  }
#else
  cols = _link->getCollisions(_groupName);
#endif

  if (!cols)
  {
    // group does not exist, create one and add to map
    cols.reset(new std::vector<UrdfCollisionPtr>);
    // new group name, create add vector to map and add Collision to the vector
    _link->collision_groups.insert(make_pair(_groupName, cols));
  }

  // group exists, add Collision to the vector in the map
  std::vector<UrdfCollisionPtr>::iterator colIt =
    find(cols->begin(), cols->end(), _collision);
  if (colIt != cols->end())
    sdfwarn << "attempted to add collision to link ["
      << _link->name
      << "], but it already exists under group ["
      << _groupName << "]\n";
  else
    cols->push_back(_collision);
}

////////////////////////////////////////////////////////////////////////////////
void ReduceVisualToParent(UrdfLinkPtr _link,
    const std::string &_groupName, UrdfVisualPtr _visual)
{
  boost::shared_ptr<std::vector<UrdfVisualPtr> > viss;
#if USE_EXTERNAL_URDF && defined(URDF_GE_0P3)
  if (_link->visual)
  {
    viss.reset(new std::vector<UrdfVisualPtr>);
    viss->push_back(_link->visual);
  }
  else
  {
    viss = boost::shared_ptr<std::vector<UrdfVisualPtr> >(&_link->visual_array);
  }
#else
  viss = _link->getVisuals(_groupName);
#endif

  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<UrdfVisualPtr>);
    // new group name, create vector, add vector to map and
    //   add Visual to the vector
    _link->visual_groups.insert(make_pair(_groupName, viss));
    sdfdbg << "successfully added a new visual group name ["
          << _groupName << "]\n";
  }

  // group exists, add Visual to the vector in the map if it's not there
  std::vector<UrdfVisualPtr>::iterator visIt
    = find(viss->begin(), viss->end(), _visual);
  if (visIt != viss->end())
    sdfwarn << "attempted to add visual to link ["
      << _link->name
      << "], but it already exists under group ["
      << _groupName << "]\n";
  else
    viss->push_back(_visual);
}

////////////////////////////////////////////////////////////////////////////////
/// reduce fixed joints by lumping inertial, visual and
// collision elements of the child link into the parent link
void ReduceFixedJoints(TiXmlElement *_root, UrdfLinkPtr _link)
{
  // if child is attached to self by fixed _link first go up the tree,
  //   check it's children recursively
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    if (FixedJointShouldBeReduced(_link->child_links[i]->parent_joint))
      ReduceFixedJoints(_root, _link->child_links[i]);

  // reduce this _link's stuff up the tree to parent but skip first joint
  //   if it's the world
  if (_link->getParent() && _link->getParent()->name != "world" &&
      _link->parent_joint && FixedJointShouldBeReduced(_link->parent_joint) )
  {
    sdfdbg << "Fixed Joint Reduction: extension lumping from ["
           << _link->name << "] to [" << _link->getParent()->name << "]\n";

    // lump sdf extensions to parent, (give them new reference _link names)
    ReduceSDFExtensionToParent(_link);

    // reduce _link elements to parent
    ReduceInertialToParent(_link);
    ReduceVisualsToParent(_link);
    ReduceCollisionsToParent(_link);
    ReduceJointsToParent(_link);
  }

  // continue down the tree for non-fixed joints
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    if (!FixedJointShouldBeReduced(_link->child_links[i]->parent_joint))
      ReduceFixedJoints(_root, _link->child_links[i]);
}

// ODE dMatrix
typedef double dMatrix3[4*3];
typedef double dVector3[4];
#define _R(i,j) R[(i)*4+(j)]
#define _I(i,j) I[(i)*4+(j)]
#define dRecip(x) ((1.0f/(x)))

struct dMass;
void dMassSetZero (dMass *m);
void dMassSetParameters (dMass *, double themass,
    double cgx, double cgy, double cgz,
    double I11, double I22, double I33,
    double I12, double I13, double I23);

struct dMass
{
  double mass;
  dVector3 c;
  dMatrix3 I;

  dMass()
  { dMassSetZero (this); }

  void setZero()
  { dMassSetZero (this); }

  void setParameters (double themass, double cgx, double cgy, double cgz,
      double I11, double I22, double I33,
      double I12, double I13, double I23)
  { dMassSetParameters (this,themass,cgx,cgy,cgz,I11,I22,I33,I12,I13,I23); }
};

void dSetZero (double *a, int n)
{
  // dAASSERT (a && n >= 0);
  double *acurr = a;
  int ncurr = n;
  while (ncurr > 0)
  {
    *(acurr++) = 0;
    --ncurr;
  }
}

void dMassSetZero (dMass *m)
{
  // dAASSERT (m);
  m->mass = 0.0;
  dSetZero(m->c, sizeof(m->c) / sizeof(double));
  dSetZero(m->I, sizeof(m->I) / sizeof(double));
}


void dMassSetParameters (dMass *m, double themass,
    double cgx, double cgy, double cgz,
    double I11, double I22, double I33,
    double I12, double I13, double I23)
{
  // dAASSERT (m);
  dMassSetZero (m);
  m->mass = themass;
  m->c[0] = cgx;
  m->c[1] = cgy;
  m->c[2] = cgz;
  m->_I(0,0) = I11;
  m->_I(1,1) = I22;
  m->_I(2,2) = I33;
  m->_I(0,1) = I12;
  m->_I(0,2) = I13;
  m->_I(1,2) = I23;
  m->_I(1,0) = I12;
  m->_I(2,0) = I13;
  m->_I(2,1) = I23;
  // dMassCheck (m);
}

void dRFromEulerAngles (dMatrix3 R, double phi, double theta, double psi)
{
  double sphi,cphi,stheta,ctheta,spsi,cpsi;
  // dAASSERT (R);
  sphi = sin(phi);
  cphi = cos(phi);
  stheta = sin(theta);
  ctheta = cos(theta);
  spsi = sin(psi);
  cpsi = cos(psi);
  _R(0,0) = cpsi*ctheta;
  _R(0,1) = spsi*ctheta;
  _R(0,2) =-stheta;
  _R(0,3) = 0.0;
  _R(1,0) = cpsi*stheta*sphi - spsi*cphi;
  _R(1,1) = spsi*stheta*sphi + cpsi*cphi;
  _R(1,2) = ctheta*sphi;
  _R(1,3) = 0.0;
  _R(2,0) = cpsi*stheta*cphi + spsi*sphi;
  _R(2,1) = spsi*stheta*cphi - cpsi*sphi;
  _R(2,2) = ctheta*cphi;
  _R(2,3) = 0.0;
}

double _dCalcVectorDot3(const double *a, const double *b, unsigned step_a,
unsigned step_b)
{
  return a[0] * b[0] + a[step_a] * b[step_b] + a[2 * step_a] * b[2 * step_b];
}

double dCalcVectorDot3 (const double *a, const double *b)
{
  return _dCalcVectorDot3(a,b,1,1);
}

double dCalcVectorDot3_41 (const double *a, const double *b)
{
  return _dCalcVectorDot3(a,b,4,1);
}

void dMultiply0_331(double *res, const double *a, const double *b)
{
  double res_0, res_1, res_2;
  res_0 = dCalcVectorDot3(a, b);
  res_1 = dCalcVectorDot3(a + 4, b);
  res_2 = dCalcVectorDot3(a + 8, b);
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

void dMultiply1_331(double *res, const double *a, const double *b)
{
  double res_0, res_1, res_2;
  res_0 = dCalcVectorDot3_41(a, b);
  res_1 = dCalcVectorDot3_41(a + 1, b);
  res_2 = dCalcVectorDot3_41(a + 2, b);
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

void dMultiply0_133(double *res, const double *a, const double *b)
{
  dMultiply1_331(res, b, a);
}

void dMultiply0_333(double *res, const double *a, const double *b)
{
  dMultiply0_133(res + 0, a + 0, b);
  dMultiply0_133(res + 4, a + 4, b);
  dMultiply0_133(res + 8, a + 8, b);
}

void dMultiply2_333(double *res, const double *a, const double *b)
{
  dMultiply0_331(res + 0, b, a + 0);
  dMultiply0_331(res + 4, b, a + 4);
  dMultiply0_331(res + 8, b, a + 8);
}

void dMassRotate (dMass *m, const dMatrix3 R)
{
  // if the body is rotated by `R' relative to its point of reference,
  // the new inertia about the point of reference is:
  //
  //   R * I * R'
  //
  // where I is the old inertia.

  dMatrix3 t1;
  double t2[3];

  // dAASSERT (m);

  // rotate inertia matrix
  dMultiply2_333 (t1,m->I,R);
  dMultiply0_333 (m->I,R,t1);

  // ensure perfect symmetry
  m->_I(1,0) = m->_I(0,1);
  m->_I(2,0) = m->_I(0,2);
  m->_I(2,1) = m->_I(1,2);

  // rotate center of mass
  dMultiply0_331 (t2,R,m->c);
  m->c[0] = t2[0];
  m->c[1] = t2[1];
  m->c[2] = t2[2];
}

void dSetCrossMatrixPlus(double *res, const double *a, unsigned skip)
{
  const double a_0 = a[0], a_1 = a[1], a_2 = a[2];
  res[1] = -a_2;
  res[2] = +a_1;
  res[skip+0] = +a_2;
  res[skip+2] = -a_0;
  res[2*skip+0] = -a_1;
  res[2*skip+1] = +a_0;
}

void dMassTranslate (dMass *m, double x, double y, double z)
{
  // if the body is translated by `a' relative to its point of reference,
  // the new inertia about the point of reference is:
  //
  //   I + mass*(crossmat(c)^2 - crossmat(c+a)^2)
  //
  // where c is the existing center of mass and I is the old inertia.

  int i,j;
  dMatrix3 ahat,chat,t1,t2;
  double a[3];

  // dAASSERT (m);

  // adjust inertia matrix
  dSetZero (chat,12);
  dSetCrossMatrixPlus (chat,m->c,4);
  a[0] = x + m->c[0];
  a[1] = y + m->c[1];
  a[2] = z + m->c[2];
  dSetZero (ahat,12);
  dSetCrossMatrixPlus (ahat,a,4);
  dMultiply0_333 (t1,ahat,ahat);
  dMultiply0_333 (t2,chat,chat);
  for (i=0; i<3; i++) for (j=0; j<3; j++)
    m->_I(i,j) += m->mass * (t2[i*4+j]-t1[i*4+j]);

  // ensure perfect symmetry
  m->_I(1,0) = m->_I(0,1);
  m->_I(2,0) = m->_I(0,2);
  m->_I(2,1) = m->_I(1,2);

  // adjust center of mass
  m->c[0] += x;
  m->c[1] += y;
  m->c[2] += z;
}

void dMassAdd (dMass *a, const dMass *b)
{
  int i;
  double denom = dRecip (a->mass + b->mass);
  for (i=0; i<3; i++) a->c[i] = (a->c[i]*a->mass + b->c[i]*b->mass)*denom;
  a->mass += b->mass;
  for (i=0; i<12; i++) a->I[i] += b->I[i];
}

/////////////////////////////////////////////////
/// print mass for link for debugging
void PrintMass(const std::string &_linkName, const dMass &_mass)
{
  sdfdbg << "LINK NAME: [" << _linkName << "] from dMass\n";
  sdfdbg << "     MASS: [" << _mass.mass << "]\n";
  sdfdbg << "       CG: [" << _mass.c[0] << ", " << _mass.c[1] << ", "
  << _mass.c[2] << "]\n";
  sdfdbg << "        I: [" << _mass.I[0] << ", " << _mass.I[1] << ", "
  << _mass.I[2] << "]\n";
  sdfdbg << "           [" << _mass.I[4] << ", " << _mass.I[5] << ", "
  << _mass.I[6] << "]\n";
  sdfdbg << "           [" << _mass.I[8] << ", " << _mass.I[9] << ", "
  << _mass.I[10] << "]\n";
}

/////////////////////////////////////////////////
/// print mass for link for debugging
void PrintMass(const UrdfLinkPtr _link)
{
  sdfdbg << "LINK NAME: [" << _link->name << "] from dMass\n";
  sdfdbg << "     MASS: [" << _link->inertial->mass << "]\n";
  sdfdbg << "       CG: [" << _link->inertial->origin.position.x << ", "
    << _link->inertial->origin.position.y << ", "
    << _link->inertial->origin.position.z << "]\n";
  sdfdbg << "        I: [" << _link->inertial->ixx << ", "
    << _link->inertial->ixy << ", "
    << _link->inertial->ixz << "]\n";
  sdfdbg << "           [" << _link->inertial->ixy << ", "
    << _link->inertial->iyy << ", "
    << _link->inertial->iyz << "]\n";
  sdfdbg << "           [" << _link->inertial->ixz << ", "
    << _link->inertial->iyz << ", "
    << _link->inertial->izz << "]\n";
}

/////////////////////////////////////////////////
/// reduce fixed joints:  lump inertial to parent link
void ReduceInertialToParent(UrdfLinkPtr _link)
{
  // now lump all contents of this _link to parent
  if (_link->inertial)
  {
    dMatrix3 R;
    double phi, theta, psi;

    // get parent mass (in parent link cg frame)
    dMass parentMass;

    if (!_link->getParent()->inertial)
      _link->getParent()->inertial.reset(new urdf::Inertial);

    dMassSetParameters(&parentMass, _link->getParent()->inertial->mass,
        0, 0, 0,
        _link->getParent()->inertial->ixx, _link->getParent()->inertial->iyy,
        _link->getParent()->inertial->izz, _link->getParent()->inertial->ixy,
        _link->getParent()->inertial->ixz, _link->getParent()->inertial->iyz);

    // transform parent inertia to parent link origin
    _link->getParent()->inertial->origin.rotation.getRPY(phi, theta, psi);
    dRFromEulerAngles(R, -phi,      0,    0);
    dMassRotate(&parentMass, R);
    dRFromEulerAngles(R,    0, -theta,    0);
    dMassRotate(&parentMass, R);
    dRFromEulerAngles(R,    0,      0, -psi);
    dMassRotate(&parentMass, R);

    // un-translate link mass from cg(inertial frame) into link frame
    dMassTranslate(&parentMass,
        _link->getParent()->inertial->origin.position.x,
        _link->getParent()->inertial->origin.position.y,
        _link->getParent()->inertial->origin.position.z);

    PrintMass("parent: " + _link->getParent()->name, parentMass);
    // PrintMass(_link->getParent());

    //////////////////////////////////////////////
    //                                          //
    // create a_link mass (in _link's cg frame) //
    //                                          //
    //////////////////////////////////////////////
    dMass linkMass;
    dMassSetParameters(&linkMass, _link->inertial->mass,
        0, 0, 0,
        _link->inertial->ixx, _link->inertial->iyy, _link->inertial->izz,
        _link->inertial->ixy, _link->inertial->ixz, _link->inertial->iyz);

    PrintMass("link : " + _link->name, linkMass);

    ////////////////////////////////////////////
    //                                        //
    // from cg (inertial frame) to link frame //
    //                                        //
    ////////////////////////////////////////////

    // Un-rotate _link mass from cg(inertial frame) into link frame
    _link->inertial->origin.rotation.getRPY(phi, theta, psi);
    dRFromEulerAngles(R, -phi,      0,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0, -theta,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0,      0, -psi);
    dMassRotate(&linkMass, R);

    // un-translate link mass from cg(inertial frame) into link frame
    dMassTranslate(&linkMass,
        _link->inertial->origin.position.x,
        _link->inertial->origin.position.y,
        _link->inertial->origin.position.z);

    ////////////////////////////////////////////
    //                                        //
    // from link frame to parent link frame   //
    //                                        //
    ////////////////////////////////////////////

    // un-rotate _link mass into parent link frame
    _link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(
        phi, theta, psi);
    dRFromEulerAngles(R, -phi,      0,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0, -theta,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0,      0, -psi);
    dMassRotate(&linkMass, R);

    // un-translate _link mass into parent link frame
    dMassTranslate(&linkMass,
        _link->parent_joint->parent_to_joint_origin_transform.position.x,
        _link->parent_joint->parent_to_joint_origin_transform.position.y,
        _link->parent_joint->parent_to_joint_origin_transform.position.z);

    PrintMass("link in parent link: " + _link->name, linkMass);

    //
    // now linkMass is in the parent frame, add linkMass to parentMass
    // new parentMass should be combined inertia,
    // centered at parent link inertial frame.
    //

    dMassAdd(&parentMass, &linkMass);

    PrintMass("combined: " + _link->getParent()->name, parentMass);

    //
    // Set new combined inertia in parent link frame into parent link urdf
    //

    // save combined mass
    _link->getParent()->inertial->mass = parentMass.mass;

    // save CoG location
    _link->getParent()->inertial->origin.position.x  = parentMass.c[0];
    _link->getParent()->inertial->origin.position.y  = parentMass.c[1];
    _link->getParent()->inertial->origin.position.z  = parentMass.c[2];

    // get MOI at new CoG location
    dMassTranslate(&parentMass,
      -_link->getParent()->inertial->origin.position.x,
      -_link->getParent()->inertial->origin.position.y,
      -_link->getParent()->inertial->origin.position.z);

    // rotate MOI at new CoG location
    _link->getParent()->inertial->origin.rotation.getRPY(phi, theta, psi);
    dRFromEulerAngles(R, phi, theta, psi);
    dMassRotate(&parentMass, R);

    // save new combined MOI
    _link->getParent()->inertial->ixx  = parentMass.I[0+4*0];
    _link->getParent()->inertial->iyy  = parentMass.I[1+4*1];
    _link->getParent()->inertial->izz  = parentMass.I[2+4*2];
    _link->getParent()->inertial->ixy  = parentMass.I[0+4*1];
    _link->getParent()->inertial->ixz  = parentMass.I[0+4*2];
    _link->getParent()->inertial->iyz  = parentMass.I[1+4*2];

    // final urdf inertia check
    PrintMass(_link->getParent());
  }
}

/////////////////////////////////////////////////
/// reduce fixed joints:  lump visuals to parent link
void ReduceVisualsToParent(UrdfLinkPtr _link)
{
  // lump visual to parent
  // lump all visual to parent, assign group name
  // "lump::"+group name+"::'+_link name
  // lump but keep the _link name in(/as) the group name,
  // so we can correlate visuals to visuals somehow.
  for (std::map<std::string,
      boost::shared_ptr<std::vector<UrdfVisualPtr> > >::iterator
      visualsIt = _link->visual_groups.begin();
      visualsIt != _link->visual_groups.end(); ++visualsIt)
  {
    if (visualsIt->first.find(std::string("lump::")) == 0)
    {
      // it's a previously lumped mesh, re-lump under same _groupName
      std::string lumpGroupName = visualsIt->first;
      sdfdbg << "re-lumping group name [" << lumpGroupName
             << "] to link [" << _link->getParent()->name << "]\n";
      for (std::vector<UrdfVisualPtr>::iterator
          visualIt = visualsIt->second->begin();
          visualIt != visualsIt->second->end(); ++visualIt)
      {
        // transform visual origin from _link frame to parent link
        // frame before adding to parent
        (*visualIt)->origin = TransformToParentFrame((*visualIt)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        ReduceVisualToParent(_link->getParent(), lumpGroupName,
            *visualIt);
      }
    }
    else
    {
      // default and any other groups meshes
      std::string lumpGroupName = std::string("lump::")+_link->name;
      sdfdbg << "adding modified lump group name [" << lumpGroupName
             << "] to link [" << _link->getParent()->name << "].\n";
      for (std::vector<UrdfVisualPtr>::iterator
          visualIt = visualsIt->second->begin();
          visualIt != visualsIt->second->end(); ++visualIt)
      {
        // transform visual origin from _link frame to
        // parent link frame before adding to parent
        (*visualIt)->origin = TransformToParentFrame((*visualIt)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified visual to parent
        ReduceVisualToParent(_link->getParent(), lumpGroupName,
            *visualIt);
      }
    }
  }
}

/////////////////////////////////////////////////
/// reduce fixed joints:  lump collisions to parent link
void ReduceCollisionsToParent(UrdfLinkPtr _link)
{
  // lump collision parent
  // lump all collision to parent, assign group name
  // "lump::"+group name+"::'+_link name
  // lump but keep the _link name in(/as) the group name,
  // so we can correlate visuals to collisions somehow.
  for (std::map<std::string,
      boost::shared_ptr<std::vector<UrdfCollisionPtr> > >::iterator
      collisionsIt = _link->collision_groups.begin();
      collisionsIt != _link->collision_groups.end(); ++collisionsIt)
  {
    if (collisionsIt->first.find(std::string("lump::")) == 0)
    {
      // if it's a previously lumped mesh, relump under same _groupName
      std::string lumpGroupName = collisionsIt->first;
      sdfdbg << "re-lumping collision [" << collisionsIt->first
             << "] for link [" << _link->name
             << "] to parent [" << _link->getParent()->name
             << "] with group name [" << lumpGroupName << "]\n";
      for (std::vector<UrdfCollisionPtr>::iterator
          collisionIt = collisionsIt->second->begin();
          collisionIt != collisionsIt->second->end(); ++collisionIt)
      {
        // transform collision origin from _link frame to
        // parent link frame before adding to parent
        (*collisionIt)->origin = TransformToParentFrame(
            (*collisionIt)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);
        // add the modified collision to parent
        ReduceCollisionToParent(_link->getParent(), lumpGroupName,
            *collisionIt);
      }
    }
    else
    {
      // default and any other group meshes
      std::string lumpGroupName = std::string("lump::")+_link->name;
      sdfdbg << "lumping collision [" << collisionsIt->first
             << "] for link [" << _link->name
             << "] to parent [" << _link->getParent()->name
             << "] with group name [" << lumpGroupName << "]\n";
      for (std::vector<UrdfCollisionPtr>::iterator
          collisionIt = collisionsIt->second->begin();
          collisionIt != collisionsIt->second->end(); ++collisionIt)
      {
        // transform collision origin from _link frame to
        // parent link frame before adding to parent
        (*collisionIt)->origin = TransformToParentFrame(
            (*collisionIt)->origin,
            _link->parent_joint->parent_to_joint_origin_transform);

        // add the modified collision to parent
        ReduceCollisionToParent(_link->getParent(), lumpGroupName,
            *collisionIt);
      }
    }
  }
  // this->PrintCollisionGroups(_link->getParent());
}

/////////////////////////////////////////////////
/// reduce fixed joints:  lump joints to parent link
void ReduceJointsToParent(UrdfLinkPtr _link)
{
  // set child link's parentJoint's parent link to
  // a parent link up stream that does not have a fixed parentJoint
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
  {
    boost::shared_ptr<urdf::Joint> parentJoint =
      _link->child_links[i]->parent_joint;
    if (!FixedJointShouldBeReduced(parentJoint))
    {
      // go down the tree until we hit a parent joint that is not fixed
      UrdfLinkPtr newParentLink = _link;
      sdf::Pose jointAnchorTransform;
      while (newParentLink->parent_joint &&
          newParentLink->getParent()->name != "world" &&
          FixedJointShouldBeReduced(newParentLink->parent_joint) )
      {
        jointAnchorTransform = jointAnchorTransform * jointAnchorTransform;
        parentJoint->parent_to_joint_origin_transform =
          TransformToParentFrame(
              parentJoint->parent_to_joint_origin_transform,
              newParentLink->parent_joint->parent_to_joint_origin_transform);
        newParentLink = newParentLink->getParent();
      }
      // now set the _link->child_links[i]->parent_joint's parent link to
      // the newParentLink
      _link->child_links[i]->setParent(newParentLink);
      parentJoint->parent_link_name = newParentLink->name;
      // and set the _link->child_links[i]->parent_joint's
      // parent_to_joint_origin_transform as the aggregated anchor transform?
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string lowerStr(const std::string &_str)
{
  std::string out = _str;
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}

////////////////////////////////////////////////////////////////////////////////
URDF2SDF::URDF2SDF()
{
  // default options
  g_enforceLimits = true;
  g_reduceFixedJoints = true;
}

////////////////////////////////////////////////////////////////////////////////
URDF2SDF::~URDF2SDF()
{
}



////////////////////////////////////////////////////////////////////////////////
std::string Values2str(unsigned int _count, const double *_values)
{
  std::stringstream ss;
  for (unsigned int i = 0 ; i < _count ; ++i)
  {
    if (i > 0)
      ss << " ";
    ss << _values[i];
  }
  return ss.str();
}

////////////////////////////////////////////////////////////////////////////////
void AddKeyValue(TiXmlElement *_elem, const std::string &_key,
    const std::string &_value)
{
  TiXmlElement* childElem = _elem->FirstChildElement(_key);
  if (childElem)
  {
    std::string oldValue = GetKeyValueAsString(childElem);
    if (oldValue != _value)
      sdfwarn << "multiple inconsistent <" << _key
        << "> exists due to fixed joint reduction"
        << " overwriting previous value [" << oldValue
        << "] with [" << _value << "].\n";
    else
       sdfdbg << "multiple consistent <" << _key
              << "> exists with [" << _value
              << "] due to fixed joint reduction.\n";
    _elem->RemoveChild(childElem);  // remove old _elem
  }

  TiXmlElement *ekey = new TiXmlElement(_key);
  TiXmlText *textEkey = new TiXmlText(_value);
  ekey->LinkEndChild(textEkey);
  _elem->LinkEndChild(ekey);
}

////////////////////////////////////////////////////////////////////////////////
void AddTransform(TiXmlElement *_elem, const sdf::Pose &_transform)
{
  sdf::Vector3 e = _transform.rot.GetAsEuler();
  double cpose[6] = { _transform.pos.x, _transform.pos.y,
    _transform.pos.z, e.x, e.y, e.z };

  /* set geometry transform */
  AddKeyValue(_elem, "pose", Values2str(6, cpose));
}

////////////////////////////////////////////////////////////////////////////////
std::string GetKeyValueAsString(TiXmlElement* _elem)
{
  std::string valueStr;
  if (_elem->Attribute("value"))
  {
    valueStr = _elem->Attribute("value");
  }
  else if (_elem->FirstChild())
    /// @todo: FIXME: comment out check for now, different tinyxml
    /// versions fails to compile:
    //  && _elem->FirstChild()->Type() == TiXmlNode::TINYXML_TEXT)
  {
    valueStr = _elem->FirstChild()->ValueStr();
  }
  return valueStr;
}

/////////////////////////////////////////////////
void ParseRobotOrigin(TiXmlDocument &_urdfXml)
{
  TiXmlElement *robotXml = _urdfXml.FirstChildElement("robot");
  TiXmlElement *originXml = robotXml->FirstChildElement("origin");
  if (originXml)
  {
    g_initialRobotPose.position = ParseVector3(
        std::string(originXml->Attribute("xyz")));
    urdf::Vector3 rpy = ParseVector3(std::string(originXml->Attribute("rpy")));
    g_initialRobotPose.rotation.setFromRPY(rpy.x, rpy.y, rpy.z);
    g_initialRobotPoseValid = true;
  }
}

/////////////////////////////////////////////////
void InsertRobotOrigin(TiXmlElement *_elem)
{
  if (g_initialRobotPoseValid)
  {
    /* set transform */
    double pose[6];
    pose[0] = g_initialRobotPose.position.x;
    pose[1] = g_initialRobotPose.position.y;
    pose[2] = g_initialRobotPose.position.z;
    g_initialRobotPose.rotation.getRPY(pose[3], pose[4], pose[5]);
    AddKeyValue(_elem, "pose", Values2str(6, pose));
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::ParseSDFExtension(TiXmlDocument &_urdfXml)
{
  TiXmlElement* robotXml = _urdfXml.FirstChildElement("robot");

  // Get all SDF extension elements, put everything in
  //   g_extensions map, containing a key string
  //   (link/joint name) and values
  for (TiXmlElement* sdfXml = robotXml->FirstChildElement("gazebo");
      sdfXml; sdfXml = sdfXml->NextSiblingElement("gazebo"))
  {
    const char* ref = sdfXml->Attribute("reference");
    std::string refStr;
    if (!ref)
    {
      // copy extensions for robot (outside of link/joint)
      refStr.clear();
    }
    else
    {
      // copy extensions for link/joint
      refStr = std::string(ref);
    }

    if (g_extensions.find(refStr) ==
        g_extensions.end())
    {
      // create extension map for reference
      std::vector<SDFExtensionPtr> ge;
      g_extensions.insert(std::make_pair(refStr, ge));
    }

    // create and insert a new SDFExtension into the map
    SDFExtensionPtr sdf(new SDFExtension());

    // begin parsing xml node
    for (TiXmlElement *childElem = sdfXml->FirstChildElement();
        childElem; childElem = childElem->NextSiblingElement())
    {
      sdf->oldLinkName = refStr;

      // go through all elements of the extension,
      //   extract what we know, and save the rest in blobs
      // @todo:  somehow use sdf definitions here instead of hard coded
      //         objects

      // material
      if (childElem->ValueStr() == "material")
      {
        sdf->material = GetKeyValueAsString(childElem);
      }
      else if (childElem->ValueStr() == "visual")
      {
        // a place to store converted doc
        for (TiXmlElement* e = childElem->FirstChildElement(); e;
            e = e->NextSiblingElement())
        {
          TiXmlDocument xmlNewDoc;

          std::ostringstream origStream;
          origStream << *e;
          sdfdbg << "visual extension [" << origStream.str() << "] not " <<
                   "converted from URDF, probably already in SDF format.";
          xmlNewDoc.Parse(origStream.str().c_str());

          // save all unknown stuff in a vector of blobs
          TiXmlElementPtr blob(
            new TiXmlElement(*xmlNewDoc.FirstChildElement()));
          sdf->visual_blobs.push_back(blob);
        }
      }
      else if (childElem->ValueStr() == "static")
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        // default of setting static flag is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          sdf->setStaticFlag = true;
        else
          sdf->setStaticFlag = false;
      }
      else if (childElem->ValueStr() == "turnGravityOff")
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        // default of gravity is true
        if (lowerStr(valueStr) == "false" || lowerStr(valueStr) == "no" ||
            valueStr == "0")
          sdf->gravity = true;
        else
          sdf->gravity = false;
      }
      else if (childElem->ValueStr() == "dampingFactor")
      {
        sdf->isDampingFactor = true;
        sdf->dampingFactor = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "maxVel")
      {
        sdf->isMaxVel = true;
        sdf->maxVel = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "minDepth")
      {
        sdf->isMinDepth = true;
        sdf->minDepth = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "mu1")
      {
        sdf->isMu1 = true;
        sdf->mu1 = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "mu2")
      {
        sdf->isMu2 = true;
        sdf->mu2 = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "fdir1")
      {
        sdf->fdir1 = GetKeyValueAsString(childElem);
      }
      else if (childElem->ValueStr() == "kp")
      {
        sdf->isKp = true;
        sdf->kp = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "kd")
      {
        sdf->isKd = true;
        sdf->kd = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "selfCollide")
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        // default of selfCollide is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          sdf->selfCollide = true;
        else
          sdf->selfCollide = false;
      }
      else if (childElem->ValueStr() == "maxContacts")
      {
        sdf->isMaxContacts = true;
        sdf->maxContacts = boost::lexical_cast<int>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "laserRetro")
      {
        sdf->isLaserRetro = true;
        sdf->laserRetro = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "stopCfm")
      {
        sdf->isStopCfm = true;
        sdf->stopCfm = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "stopErp")
      {
        sdf->isStopErp = true;
        sdf->stopErp = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "stopKp")
      {
        sdf->isStopKp = true;
        sdf->stopKp = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "stopKd")
      {
        sdf->isStopKd = true;
        sdf->stopKd = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "initialJointPosition")
      {
        sdf->isInitialJointPosition = true;
        sdf->initialJointPosition = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "fudgeFactor")
      {
        sdf->isFudgeFactor = true;
        sdf->fudgeFactor = boost::lexical_cast<double>(
            GetKeyValueAsString(childElem).c_str());
      }
      else if (childElem->ValueStr() == "provideFeedback")
      {
        sdf->isProvideFeedback = true;
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          sdf->provideFeedback = true;
        else
          sdf->provideFeedback = false;
      }
      else if (childElem->ValueStr() == "canonicalBody")
      {
        sdfdbg << "do nothing with canonicalBody\n";
      }
      else if (childElem->ValueStr() == "cfmDamping" ||
               childElem->ValueStr() == "implicitSpringDamper")
      {
        if (childElem->ValueStr() == "cfmDamping")
          sdfwarn << "Note that cfmDamping is being deprecated by "
                  << "implicitSpringDamper, please replace instances "
                  << "of cfmDamping with implicitSpringDamper in your model.\n";

        sdf->isImplicitSpringDamper = true;
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
          sdf->implicitSpringDamper = true;
        else
          sdf->implicitSpringDamper = false;
      }
      else if (childElem->ValueStr() == "disableFixedJointLumping")
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          g_fixedJointsNotReduced.insert(refStr);
        }
      }
      else
      {
        // a place to store converted doc
        TiXmlDocument xmlNewDoc;

        std::ostringstream stream;
        stream << *childElem;
        sdfdbg << "extension [" << stream.str() <<
          "] not converted from URDF, probably already in SDF format.\n";
        xmlNewDoc.Parse(stream.str().c_str());

        // save all unknown stuff in a vector of blobs
        TiXmlElementPtr blob(new TiXmlElement(*xmlNewDoc.FirstChildElement()));
        sdf->blobs.push_back(blob);
      }
    }

    // insert into my map
    (g_extensions.find(refStr))->second.push_back(sdf);
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionCollision(TiXmlElement *_elem,
    const std::string &_linkName)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
        ge != sdfIt->second.end(); ++ge)
    {
      if (((*ge)->oldLinkName == _linkName) ||
          (_elem->Attribute("name") &&
           (std::string(_elem->Attribute("name")) ==
           _linkName + g_collisionExt + std::string("_") + (*ge)->oldLinkName)))
      {
        TiXmlElement *surface = new TiXmlElement("surface");
        TiXmlElement *friction = new TiXmlElement("friction");
        TiXmlElement *frictionOde = new TiXmlElement("ode");
        TiXmlElement *contact = new TiXmlElement("contact");
        TiXmlElement *contactOde = new TiXmlElement("ode");

        // insert mu1, mu2, kp, kd for collision
        if ((*ge)->isMu1)
          AddKeyValue(frictionOde, "mu",
              Values2str(1, &(*ge)->mu1));
        if ((*ge)->isMu2)
          AddKeyValue(frictionOde, "mu2",
              Values2str(1, &(*ge)->mu2));
        if (!(*ge)->fdir1.empty())
          AddKeyValue(frictionOde, "fdir1", (*ge)->fdir1);
        if ((*ge)->isKp)
          AddKeyValue(contactOde, "kp", Values2str(1, &(*ge)->kp));
        if ((*ge)->isKd)
          AddKeyValue(contactOde, "kd", Values2str(1, &(*ge)->kd));
        // max contact interpenetration correction velocity
        if ((*ge)->isMaxVel)
          AddKeyValue(contactOde, "max_vel",
              Values2str(1, &(*ge)->maxVel));
        // contact interpenetration margin tolerance
        if ((*ge)->isMinDepth)
          AddKeyValue(contactOde, "min_depth",
              Values2str(1, &(*ge)->minDepth));
        if ((*ge)->isLaserRetro)
          AddKeyValue(_elem, "laser_retro",
              Values2str(1, &(*ge)->laserRetro));
        if ((*ge)->isMaxContacts)
          AddKeyValue(_elem, "max_contacts",
              boost::lexical_cast<std::string>((*ge)->maxContacts));

        contact->LinkEndChild(contactOde);
        surface->LinkEndChild(contact);
        friction->LinkEndChild(frictionOde);
        surface->LinkEndChild(friction);
        _elem->LinkEndChild(surface);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionVisual(TiXmlElement *_elem,
    const std::string &_linkName)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
        ge != sdfIt->second.end(); ++ge)
    {
      if (_linkName.find((*ge)->oldLinkName) != std::string::npos)
      {
        // insert material block
        if (!(*ge)->material.empty())
        {
          // new sdf needs <material><script>...</script></material>
          TiXmlElement *materialElem = new TiXmlElement("material");
          TiXmlElement *scriptElem = new TiXmlElement("script");
          if (scriptElem && materialElem)
          {
            AddKeyValue(scriptElem, "name", (*ge)->material);
            materialElem->LinkEndChild(scriptElem);
            _elem->LinkEndChild(materialElem);
            // AddKeyValue(_elem, "material", (*ge)->material);
          }
          else
          {
            // Memory allocation error
            sdferr << "Memory allocation error while processing <material>.\n";
          }
        }

        // insert any blobs (including visual plugins)
        if (!(*ge)->visual_blobs.empty())
        {
          std::vector<TiXmlElementPtr>::iterator blob;
          for (blob = (*ge)->visual_blobs.begin();
              blob != (*ge)->visual_blobs.end(); ++blob)
          {
            _elem->LinkEndChild((*blob)->Clone());
          }
        }

      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionLink(TiXmlElement *_elem, const std::string &_linkName)
{
  for (StringSDFExtensionPtrMap::iterator
       sdfIt = g_extensions.begin();
       sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _linkName)
    {
      sdfdbg << "inserting extension with reference ["
             << _linkName << "] into link.\n";
      for (std::vector<SDFExtensionPtr>::iterator ge =
          sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        // insert gravity
        if ((*ge)->gravity)
          AddKeyValue(_elem, "gravity", "true");
        else
          AddKeyValue(_elem, "gravity", "false");

        // damping factor
        TiXmlElement *velocityDecay = new TiXmlElement("velocity_decay");
        if ((*ge)->isDampingFactor)
        {
          /// @todo separate linear and angular velocity decay
          AddKeyValue(velocityDecay, "linear",
              Values2str(1, &(*ge)->dampingFactor));
          AddKeyValue(velocityDecay, "angular",
              Values2str(1, &(*ge)->dampingFactor));
        }
        _elem->LinkEndChild(velocityDecay);
        // selfCollide tag
        if ((*ge)->selfCollide)
          AddKeyValue(_elem, "self_collide", "true");
        else
          AddKeyValue(_elem, "self_collide", "false");
        // insert blobs into body
        for (std::vector<TiXmlElementPtr>::iterator
            blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          _elem->LinkEndChild((*blobIt)->Clone());
        }

      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionJoint(TiXmlElement *_elem,
    const std::string &_jointName)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _jointName)
    {
      for (std::vector<SDFExtensionPtr>::iterator
          ge = sdfIt->second.begin();
          ge != sdfIt->second.end(); ++ge)
      {

        TiXmlElement *physics = _elem->FirstChildElement("physics");
        bool newPhysics = false;
        if (physics == NULL)
        {
          physics = new TiXmlElement("physics");
          newPhysics = true;
        }

        TiXmlElement *physicsOde = physics->FirstChildElement("ode");
        bool newPhysicsOde = false;
        if (physicsOde == NULL)
        {
          physicsOde = new TiXmlElement("ode");
          newPhysicsOde = true;
        }

        TiXmlElement *limit = physicsOde->FirstChildElement("limit");
        bool newLimit = false;
        if (limit == NULL)
        {
          limit = new TiXmlElement("limit");
          newLimit = true;
        }

        // insert stopCfm, stopErp, fudgeFactor
        if ((*ge)->isStopCfm)
        {
          AddKeyValue(limit, "cfm", Values2str(1, &(*ge)->stopCfm));
        }
        if ((*ge)->isStopErp)
        {
          AddKeyValue(limit, "erp", Values2str(1, &(*ge)->stopErp));
        }
        /* gone
        if ((*ge)->isInitialJointPosition)
           AddKeyValue(_elem, "initialJointPosition",
             Values2str(1, &(*ge)->initialJointPosition));
         */

        // insert provideFeedback
        if ((*ge)->isProvideFeedback)
        {
          if ((*ge)->provideFeedback)
          {
            AddKeyValue(physics, "provide_feedback", "true");
            AddKeyValue(physicsOde, "provide_feedback", "true");
          }
          else
          {
            AddKeyValue(physics, "provide_feedback", "false");
            AddKeyValue(physicsOde, "provide_feedback", "false");
          }
        }

        // insert implicitSpringDamper
        if ((*ge)->isImplicitSpringDamper)
        {
          if ((*ge)->implicitSpringDamper)
          {
            AddKeyValue(physicsOde, "implicit_spring_damper", "true");
            /// \TODO: deprecating cfm_damping, transitional tag below
            AddKeyValue(physicsOde, "cfm_damping", "true");
          }
          else
          {
            AddKeyValue(physicsOde, "implicit_spring_damper", "false");
            /// \TODO: deprecating cfm_damping, transitional tag below
            AddKeyValue(physicsOde, "cfm_damping", "false");
          }
        }

        // insert fudgeFactor
        if ((*ge)->isFudgeFactor)
          AddKeyValue(physicsOde, "fudge_factor",
              Values2str(1, &(*ge)->fudgeFactor));

        if (newLimit)
          physicsOde->LinkEndChild(limit);
        if (newPhysicsOde)
          physics->LinkEndChild(physicsOde);
        if (newPhysics)
          _elem->LinkEndChild(physics);

        // insert all additional blobs into joint
        for (std::vector<TiXmlElementPtr>::iterator
            blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          _elem->LinkEndChild((*blobIt)->Clone());
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionRobot(TiXmlElement *_elem)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first.empty())
    {
      // no reference specified
      for (std::vector<SDFExtensionPtr>::iterator
          ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        // insert static flag
        if ((*ge)->setStaticFlag)
          AddKeyValue(_elem, "static", "true");
        else
          AddKeyValue(_elem, "static", "false");

        // copy extension containing blobs and without reference
        for (std::vector<TiXmlElementPtr>::iterator
            blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream streamIn;
          streamIn << *(*blobIt);
          _elem->LinkEndChild((*blobIt)->Clone());
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateGeometry(TiXmlElement* _elem,
    boost::shared_ptr<urdf::Geometry> _geom)
{
  TiXmlElement *sdfGeometry = new TiXmlElement("geometry");

  std::string type;
  TiXmlElement *geometryType = NULL;

  switch (_geom->type)
  {
    case urdf::Geometry::BOX:
      type = "box";
      {
        boost::shared_ptr<const urdf::Box> box;
        box = boost::dynamic_pointer_cast< const urdf::Box >(_geom);
        int sizeCount = 3;
        double sizeVals[3];
        sizeVals[0] = box->dim.x;
        sizeVals[1] = box->dim.y;
        sizeVals[2] = box->dim.z;
        geometryType = new TiXmlElement(type);
        AddKeyValue(geometryType, "size",
            Values2str(sizeCount, sizeVals));
      }
      break;
    case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        boost::shared_ptr<const urdf::Cylinder> cylinder;
        cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(_geom);
        geometryType = new TiXmlElement(type);
        AddKeyValue(geometryType, "length",
            Values2str(1, &cylinder->length));
        AddKeyValue(geometryType, "radius",
            Values2str(1, &cylinder->radius));
      }
      break;
    case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        boost::shared_ptr<const urdf::Sphere> sphere;
        sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(_geom);
        geometryType = new TiXmlElement(type);
        AddKeyValue(geometryType, "radius",
            Values2str(1, &sphere->radius));
      }
      break;
    case urdf::Geometry::MESH:
      type = "mesh";
      {
        boost::shared_ptr<const urdf::Mesh> mesh;
        mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(_geom);
        geometryType = new TiXmlElement(type);
        AddKeyValue(geometryType, "scale", Vector32Str(mesh->scale));
        // do something more to meshes
        {
          /* set mesh file */
          if (mesh->filename.empty())
          {
            sdferr << "urdf2sdf: mesh geometry with no filename given.\n";
          }

          // give some warning if file does not exist.
          // disabled while switching to uri
          // @todo: re-enable check
          // std::ifstream fin;
          // fin.open(mesh->filename.c_str(), std::ios::in);
          // fin.close();
          // if (fin.fail())
          //   sdfwarn << "filename referred by mesh ["
          //          << mesh->filename << "] does not appear to exist.\n";

          // Convert package:// to model://,
          // in ROS, this will work if
          // the model package is in ROS_PACKAGE_PATH and has a manifest.xml
          // as a typical ros package does.
          std::string modelFilename = mesh->filename;
          std::string packagePrefix("package://");
          std::string modelPrefix("model://");
          size_t pos1 = modelFilename.find(packagePrefix, 0);
          if (pos1 != std::string::npos)
          {
            size_t repLen = packagePrefix.size();
            modelFilename.replace(pos1, repLen, modelPrefix);
            // sdfwarn << "ros style uri [package://] is"
            //   << "automatically converted: [" << modelFilename
            //   << "], make sure your ros package is in GAZEBO_MODEL_PATH"
            //   << " and switch your manifest to conform to sdf's"
            //   << " model database format.  See ["
            //   << "http://sdfsim.org/wiki/Model_database#Model_Manifest_XML"
            //   << "] for more info.\n";
          }

          // add mesh filename
          AddKeyValue(geometryType, "uri", modelFilename);
        }
      }
      break;
    default:
      sdfwarn << "Unknown body type: [" << static_cast<int>(_geom->type)
        << "] skipped in geometry\n";
      break;
  }

  if (geometryType)
  {
    sdfGeometry->LinkEndChild(geometryType);
    _elem->LinkEndChild(sdfGeometry);
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string GetGeometryBoundingBox(
    boost::shared_ptr<urdf::Geometry> _geom, double *_sizeVals)
{
  std::string type;

  switch (_geom->type)
  {
    case urdf::Geometry::BOX:
      type = "box";
      {
        boost::shared_ptr<const urdf::Box> box;
        box = boost::dynamic_pointer_cast<const urdf::Box >(_geom);
        _sizeVals[0] = box->dim.x;
        _sizeVals[1] = box->dim.y;
        _sizeVals[2] = box->dim.z;
      }
      break;
    case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        boost::shared_ptr<const urdf::Cylinder> cylinder;
        cylinder = boost::dynamic_pointer_cast<const urdf::Cylinder >(_geom);
        _sizeVals[0] = cylinder->radius * 2;
        _sizeVals[1] = cylinder->radius * 2;
        _sizeVals[2] = cylinder->length;
      }
      break;
    case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        boost::shared_ptr<const urdf::Sphere> sphere;
        sphere = boost::dynamic_pointer_cast<const urdf::Sphere >(_geom);
        _sizeVals[0] = _sizeVals[1] = _sizeVals[2] = sphere->radius * 2;
      }
      break;
    case urdf::Geometry::MESH:
      type = "trimesh";
      {
        boost::shared_ptr<const urdf::Mesh> mesh;
        mesh = boost::dynamic_pointer_cast<const urdf::Mesh >(_geom);
        _sizeVals[0] = mesh->scale.x;
        _sizeVals[1] = mesh->scale.y;
        _sizeVals[2] = mesh->scale.z;
      }
      break;
    default:
      _sizeVals[0] = _sizeVals[1] = _sizeVals[2] = 0;
      sdfwarn << "Unknown body type: [" << static_cast<int>(_geom->type)
        << "] skipped in geometry\n";
      break;
  }

  return type;
}

////////////////////////////////////////////////////////////////////////////////
void PrintCollisionGroups(UrdfLinkPtr _link)
{
  sdfdbg << "COLLISION LUMPING: link: [" << _link->name << "] contains ["
    << static_cast<int>(_link->collision_groups.size())
    << "] collisions.\n";
  for (std::map<std::string,
      boost::shared_ptr<std::vector<UrdfCollisionPtr > > >::iterator
      colsIt = _link->collision_groups.begin();
      colsIt != _link->collision_groups.end(); ++colsIt)
  {
    sdfdbg << "    collision_groups: [" << colsIt->first << "] has ["
      << static_cast<int>(colsIt->second->size())
      << "] Collision objects\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
urdf::Pose TransformToParentFrame(
    urdf::Pose _transformInLinkFrame, urdf::Pose _parentToLinkTransform)
{
  // transform to sdf::Pose then call TransformToParentFrame
  sdf::Pose p1 = CopyPose(_transformInLinkFrame);
  sdf::Pose p2 = CopyPose(_parentToLinkTransform);
  return CopyPose(TransformToParentFrame(p1, p2));
}

////////////////////////////////////////////////////////////////////////////////
sdf::Pose TransformToParentFrame(sdf::Pose _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform)
{
  // transform to sdf::Pose then call TransformToParentFrame
  sdf::Pose p2 = CopyPose(_parentToLinkTransform);
  return TransformToParentFrame(_transformInLinkFrame, p2);
}

////////////////////////////////////////////////////////////////////////////////
sdf::Pose TransformToParentFrame(sdf::Pose _transformInLinkFrame,
    sdf::Pose _parentToLinkTransform)
{
  sdf::Pose transformInParentLinkFrame;
  // rotate link pose to parentLink frame
  transformInParentLinkFrame.pos =
    _parentToLinkTransform.rot * _transformInLinkFrame.pos;
  transformInParentLinkFrame.rot =
    _parentToLinkTransform.rot * _transformInLinkFrame.rot;
  // translate link to parentLink frame
  transformInParentLinkFrame.pos =
    _parentToLinkTransform.pos + transformInParentLinkFrame.pos;

  return transformInParentLinkFrame;
}

/////////////////////////////////////////////////
/// reduced fixed joints: transform to parent frame
sdf::Pose inverseTransformToParentFrame(
    sdf::Pose _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform)
{
  sdf::Pose transformInParentLinkFrame;
  //   rotate link pose to parentLink frame
  urdf::Rotation ri = _parentToLinkTransform.rotation.GetInverse();
  sdf::Quaternion q1(ri.w, ri.x, ri.y, ri.z);
  transformInParentLinkFrame.pos = q1 * _transformInLinkFrame.pos;
  urdf::Rotation r2 = _parentToLinkTransform.rotation.GetInverse();
  sdf::Quaternion q3(r2.w, r2.x, r2.y, r2.z);
  transformInParentLinkFrame.rot = q3 * _transformInLinkFrame.rot;
  //   translate link to parentLink frame
  transformInParentLinkFrame.pos.x = transformInParentLinkFrame.pos.x
    - _parentToLinkTransform.position.x;
  transformInParentLinkFrame.pos.y = transformInParentLinkFrame.pos.y
    - _parentToLinkTransform.position.y;
  transformInParentLinkFrame.pos.z = transformInParentLinkFrame.pos.z
    - _parentToLinkTransform.position.z;

  return transformInParentLinkFrame;
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionToParent(UrdfLinkPtr _link)
{
  /// \todo: move to header
  /// Take the link's existing list of gazebo extensions, transfer them
  /// into parent link.  Along the way, update local transforms by adding
  /// the additional transform to parent.  Also, look through all
  /// referenced link names with plugins and update references to current
  /// link to the parent link. (reduceGazeboExtensionFrameReplace())

  /// @todo: this is a very complicated module that updates the plugins
  /// based on fixed joint reduction really wish this could be a lot cleaner

  std::string linkName = _link->name;

  // update extension map with references to linkName
  // this->ListSDFExtensions();
  StringSDFExtensionPtrMap::iterator ext = g_extensions.find(linkName);
  if (ext != g_extensions.end())
  {
    sdfdbg << "  REDUCE EXTENSION: moving reference from ["
           << linkName << "] to [" << _link->getParent()->name << "]\n";

    // update reduction transform (for rays, cameras for now).
    //   FIXME: contact frames too?
    for (std::vector<SDFExtensionPtr>::iterator ge = ext->second.begin();
        ge != ext->second.end(); ++ge)
    {
      (*ge)->reductionTransform = TransformToParentFrame(
          (*ge)->reductionTransform,
          _link->parent_joint->parent_to_joint_origin_transform);
      // for sensor and projector blocks only
      ReduceSDFExtensionsTransform((*ge));
    }

    // find pointer to the existing extension with the new _link reference
    std::string newLinkName = _link->getParent()->name;
    StringSDFExtensionPtrMap::iterator newExt = g_extensions.find(newLinkName);

    // if none exist, create new extension with newLinkName
    if (newExt == g_extensions.end())
    {
      std::vector<SDFExtensionPtr> ge;
      g_extensions.insert(std::make_pair(
            newLinkName, ge));
      newExt = g_extensions.find(newLinkName);
    }

    // move sdf extensions from _link into the parent _link's extensions
    for (std::vector<SDFExtensionPtr>::iterator ge = ext->second.begin();
        ge != ext->second.end(); ++ge)
      newExt->second.push_back(*ge);
    ext->second.clear();
  }

  // for extensions with empty reference, search and replace
  // _link name patterns within the plugin with new _link name
  // and assign the proper reduction transform for the _link name pattern
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    // update reduction transform (for contacts, rays, cameras for now).
    for (std::vector<SDFExtensionPtr>::iterator
        ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      ReduceSDFExtensionFrameReplace(*ge, _link);
  }

  // this->ListSDFExtensions();
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionFrameReplace(SDFExtensionPtr _ge,
    UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  // HACK: need to do this more generally, but we also need to replace
  //       all instances of _link name with new link name
  //       e.g. contact sensor refers to
  //         <collision>base_link_collision</collision>
  //         and it needs to be reparented to
  //         <collision>base_footprint_collision</collision>
  sdfdbg << "  STRING REPLACE: instances of _link name ["
        << linkName << "] with [" << newLinkName << "]\n";
  for (std::vector<TiXmlElementPtr>::iterator blobIt =
         _ge->blobs.begin();
         blobIt != _ge->blobs.end(); ++blobIt)
  {
    std::ostringstream debugStreamIn;
    debugStreamIn << *(*blobIt);
    std::string debugBlob = debugStreamIn.str();
    sdfdbg << "        INITIAL STRING link ["
           << linkName << "]-->[" << newLinkName << "]: ["
           << debugBlob << "]\n";

    ReduceSDFExtensionContactSensorFrameReplace(blobIt, _link);
    ReduceSDFExtensionPluginFrameReplace(blobIt, _link,
        "plugin", "bodyName", _ge->reductionTransform);
    ReduceSDFExtensionPluginFrameReplace(blobIt, _link,
        "plugin", "frameName", _ge->reductionTransform);
    ReduceSDFExtensionProjectorFrameReplace(blobIt, _link);
    ReduceSDFExtensionGripperFrameReplace(blobIt, _link);
    ReduceSDFExtensionJointFrameReplace(blobIt, _link);

    std::ostringstream debugStreamOut;
    debugStreamOut << *(*blobIt);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionsTransform(SDFExtensionPtr _ge)
{
  for (std::vector<TiXmlElementPtr>::iterator blobIt =
         _ge->blobs.begin();
         blobIt != _ge->blobs.end(); ++blobIt)
  {
    /// @todo make sure we are not missing any additional transform reductions
    ReduceSDFExtensionSensorTransformReduction(blobIt,
        _ge->reductionTransform);
    ReduceSDFExtensionProjectorTransformReduction(blobIt,
        _ge->reductionTransform);
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::ListSDFExtensions()
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    int extCount = 0;
    for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
        ge != sdfIt->second.end(); ++ge)
    {
      if (!(*ge)->blobs.empty())
      {
        sdfdbg <<  "  PRINTING [" << static_cast<int>((*ge)->blobs.size())
          << "] BLOBS for extension [" << ++extCount
          << "] referencing [" << sdfIt->first << "]\n";
        for (std::vector<TiXmlElementPtr>::iterator
            blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream streamIn;
          streamIn << *(*blobIt);
          sdfdbg << "    BLOB: [" << streamIn.str() << "]\n";
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::ListSDFExtensions(const std::string &_reference)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _reference)
    {
      sdfdbg <<  "  PRINTING [" << static_cast<int>(sdfIt->second.size())
        << "] extensions referencing [" << _reference << "]\n";
      for (std::vector<SDFExtensionPtr>::iterator
          ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        for (std::vector<TiXmlElementPtr>::iterator
            blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          std::ostringstream streamIn;
          streamIn << *(*blobIt);
          sdfdbg << "    BLOB: [" << streamIn.str() << "]\n";
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateSDF(TiXmlElement *_root,
    ConstUrdfLinkPtr _link,
    const sdf::Pose &_transform)
{
  sdf::Pose _currentTransform = _transform;

  // must have an <inertial> block and cannot have zero mass.
  //  allow det(I) == zero, in the case of point mass geoms.
  // @todo:  keyword "world" should be a constant defined somewhere else
  if (_link->name != "world" &&
      ((!_link->inertial) || sdf::equal(_link->inertial->mass, 0.0)))
  {
    if (!_link->child_links.empty())
      sdfdbg << "urdf2sdf: link[" << _link->name
        << "] has no inertia, ["
        << static_cast<int>(_link->child_links.size())
        << "] children links ignored.\n";

    if (!_link->child_joints.empty())
      sdfdbg << "urdf2sdf: link[" << _link->name
        << "] has no inertia, ["
        << static_cast<int>(_link->child_links.size())
        << "] children joints ignored.\n";

    if (_link->parent_joint)
      sdfdbg << "urdf2sdf: link[" << _link->name
        << "] has no inertia, "
        << "parent joint [" << _link->parent_joint->name
        << "] ignored.\n";

    sdfdbg << "urdf2sdf: link[" << _link->name
      << "] has no inertia, not modeled in sdf\n";
    return;
  }

  /* create <body:...> block for non fixed joint attached bodies */
  if ((_link->getParent() && _link->getParent()->name == "world") ||
      !g_reduceFixedJoints ||
      (!_link->parent_joint ||
       !FixedJointShouldBeReduced(_link->parent_joint)))
    CreateLink(_root, _link, _currentTransform);

  // recurse into children
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    CreateSDF(_root, _link->child_links[i], _currentTransform);
}

////////////////////////////////////////////////////////////////////////////////
sdf::Pose CopyPose(urdf::Pose _pose)
{
  sdf::Pose p;
  p.pos.x = _pose.position.x;
  p.pos.y = _pose.position.y;
  p.pos.z = _pose.position.z;
  p.rot.x = _pose.rotation.x;
  p.rot.y = _pose.rotation.y;
  p.rot.z = _pose.rotation.z;
  p.rot.w = _pose.rotation.w;
  return p;
}

////////////////////////////////////////////////////////////////////////////////
urdf::Pose CopyPose(sdf::Pose _pose)
{
  urdf::Pose p;
  p.position.x = _pose.pos.x;
  p.position.y = _pose.pos.y;
  p.position.z = _pose.pos.z;
  p.rotation.x = _pose.rot.x;
  p.rotation.y = _pose.rot.y;
  p.rotation.z = _pose.rot.z;
  p.rotation.w = _pose.rot.w;
  return p;
}

////////////////////////////////////////////////////////////////////////////////
void CreateLink(TiXmlElement *_root,
    ConstUrdfLinkPtr _link,
    sdf::Pose &_currentTransform)
{
  /* create new body */
  TiXmlElement *elem     = new TiXmlElement("link");

  /* set body name */
  elem->SetAttribute("name", _link->name);

  /* compute global transform */
  sdf::Pose localTransform;
  // this is the transform from parent link to current _link
  // this transform does not exist for the root link
  if (_link->parent_joint)
  {
    localTransform = CopyPose(
        _link->parent_joint->parent_to_joint_origin_transform);
    _currentTransform = localTransform * _currentTransform;
  }
  else
    sdfdbg << "[" << _link->name << "] has no parent joint\n";

  // create origin tag for this element
  AddTransform(elem, _currentTransform);

  /* create new inerial block */
  CreateInertial(elem, _link);

  /* create new collision block */
  CreateCollisions(elem, _link);

  /* create new visual block */
  CreateVisuals(elem, _link);

  /* copy sdf extensions data */
  InsertSDFExtensionLink(elem, _link->name);

  /* add body to document */
  _root->LinkEndChild(elem);

  /* make a <joint:...> block */
  CreateJoint(_root, _link, _currentTransform);
}

////////////////////////////////////////////////////////////////////////////////
void CreateCollisions(TiXmlElement* _elem,
    ConstUrdfLinkPtr _link)
{
  // loop through all collision groups. as well as additional collision from
  //   lumped meshes (fixed joint reduction)
  for (std::map<std::string,
      boost::shared_ptr<std::vector<UrdfCollisionPtr> > >::const_iterator
      collisionsIt = _link->collision_groups.begin();
      collisionsIt != _link->collision_groups.end(); ++collisionsIt)
  {
    unsigned int defaultMeshCount = 0;
    unsigned int groupMeshCount = 0;
    unsigned int lumpMeshCount = 0;
    // loop through collisions in each group
    for (std::vector<UrdfCollisionPtr>::iterator
        collision = collisionsIt->second->begin();
        collision != collisionsIt->second->end();
        ++collision)
    {
      if (collisionsIt->first == "default")
      {
        sdfdbg << "creating default collision for link [" << _link->name
               << "]";

        std::string collisionPrefix = _link->name;

        if (defaultMeshCount > 0)
        {
          // append _[meshCount] to link name for additional collisions
          std::ostringstream collisionNameStream;
          collisionNameStream << collisionPrefix << "_" << defaultMeshCount;
          collisionPrefix = collisionNameStream.str();
        }

        /* make a <collision> block */
        CreateCollision(_elem, _link, *collision, collisionPrefix);

        // only 1 default mesh
        ++defaultMeshCount;
      }
      else if (collisionsIt->first.find(std::string("lump::")) == 0)
      {
        // if collision name starts with "lump::", pass through
        //   original parent link name
        sdfdbg << "creating lump collision [" << collisionsIt->first
               << "] for link [" << _link->name << "].\n";
        /// collisionPrefix is the original name before lumping
        std::string collisionPrefix = collisionsIt->first.substr(6);

        if (lumpMeshCount > 0)
        {
          // append _[meshCount] to link name for additional collisions
          std::ostringstream collisionNameStream;
          collisionNameStream << collisionPrefix << "_" << lumpMeshCount;
          collisionPrefix = collisionNameStream.str();
        }

        CreateCollision(_elem, _link, *collision, collisionPrefix);
        ++lumpMeshCount;
      }
      else
      {
        sdfdbg << "adding collisions from collision group ["
              << collisionsIt->first << "]\n";

        std::string collisionPrefix = _link->name + std::string("_") +
          collisionsIt->first;

        if (groupMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional collisions
          std::ostringstream collisionNameStream;
          collisionNameStream << collisionPrefix << "_" << groupMeshCount;
          collisionPrefix = collisionNameStream.str();
        }

        CreateCollision(_elem, _link, *collision, collisionPrefix);
        ++groupMeshCount;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateVisuals(TiXmlElement* _elem,
    ConstUrdfLinkPtr _link)
{
  // loop through all visual groups. as well as additional visuals from
  //   lumped meshes (fixed joint reduction)
  for (std::map<std::string,
      boost::shared_ptr<std::vector<UrdfVisualPtr> > >::const_iterator
      visualsIt = _link->visual_groups.begin();
      visualsIt != _link->visual_groups.end(); ++visualsIt)
  {
    unsigned int defaultMeshCount = 0;
    unsigned int groupMeshCount = 0;
    unsigned int lumpMeshCount = 0;
    // loop through all visuals in this group
    for (std::vector<UrdfVisualPtr>::iterator
        visual = visualsIt->second->begin();
        visual != visualsIt->second->end();
        ++visual)
    {
      if (visualsIt->first == "default")
      {
        sdfdbg << "creating default visual for link [" << _link->name
               << "]";

        std::string visualPrefix = _link->name;

        if (defaultMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visualNameStream;
          visualNameStream << visualPrefix << "_" << defaultMeshCount;
          visualPrefix = visualNameStream.str();
        }

        // create a <visual> block
        CreateVisual(_elem, _link, *visual, visualPrefix);

        // only 1 default mesh
        ++defaultMeshCount;
      }
      else if (visualsIt->first.find(std::string("lump::")) == 0)
      {
        // if visual name starts with "lump::", pass through
        //   original parent link name
        sdfdbg << "creating lump visual [" << visualsIt->first
               << "] for link [" << _link->name << "].\n";
        /// visualPrefix is the original name before lumping
        std::string visualPrefix = visualsIt->first.substr(6);

        if (lumpMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visualNameStream;
          visualNameStream << visualPrefix << "_" << lumpMeshCount;
          visualPrefix = visualNameStream.str();
        }

        CreateVisual(_elem, _link, *visual, visualPrefix);
        ++lumpMeshCount;
      }
      else
      {
        sdfdbg << "adding visuals from visual group ["
              << visualsIt->first << "]\n";

        std::string visualPrefix = _link->name + std::string("_") +
          visualsIt->first;

        if (groupMeshCount > 0)
        {
          // append _[meshCount] to _link name for additional visuals
          std::ostringstream visualNameStream;
          visualNameStream << visualPrefix << "_" << groupMeshCount;
          visualPrefix = visualNameStream.str();
        }

        CreateVisual(_elem, _link, *visual, visualPrefix);
        ++groupMeshCount;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateInertial(TiXmlElement *_elem,
    ConstUrdfLinkPtr _link)
{
  TiXmlElement *inertial = new TiXmlElement("inertial");

  /* set mass properties */
  // check and print a warning message
  double roll, pitch, yaw;
  _link->inertial->origin.rotation.getRPY(roll, pitch, yaw);

  /// add pose
  sdf::Pose pose = CopyPose(_link->inertial->origin);
  AddTransform(inertial, pose);

  // add mass
  AddKeyValue(inertial, "mass",
      Values2str(1, &_link->inertial->mass));

  // add inertia (ixx, ixy, ixz, iyy, iyz, izz)
  TiXmlElement *inertia = new TiXmlElement("inertia");
  AddKeyValue(inertia, "ixx",
      Values2str(1, &_link->inertial->ixx));
  AddKeyValue(inertia, "ixy",
      Values2str(1, &_link->inertial->ixy));
  AddKeyValue(inertia, "ixz",
      Values2str(1, &_link->inertial->ixz));
  AddKeyValue(inertia, "iyy",
      Values2str(1, &_link->inertial->iyy));
  AddKeyValue(inertia, "iyz",
      Values2str(1, &_link->inertial->iyz));
  AddKeyValue(inertia, "izz",
      Values2str(1, &_link->inertial->izz));
  inertial->LinkEndChild(inertia);

  _elem->LinkEndChild(inertial);
}

////////////////////////////////////////////////////////////////////////////////
void CreateJoint(TiXmlElement *_root,
    ConstUrdfLinkPtr _link,
    sdf::Pose &_currentTransform)
{
  /* compute the joint tag */
  std::string jtype;
  jtype.clear();
  if (_link->parent_joint != NULL)
  {
    switch (_link->parent_joint->type)
    {
      case urdf::Joint::CONTINUOUS:
      case urdf::Joint::REVOLUTE:
        jtype = "revolute";
        break;
      case urdf::Joint::PRISMATIC:
        jtype = "prismatic";
        break;
      case urdf::Joint::FLOATING:
      case urdf::Joint::PLANAR:
        break;
      case urdf::Joint::FIXED:
        jtype = "fixed";
        break;
      default:
        sdfwarn << "Unknown joint type: [" << static_cast<int>(_link->parent_joint->type)
          << "] in link [" << _link->name << "]\n";
        break;
    }
  }

  // skip if joint type is fixed and we are not faking it with a hinge,
  //   skip/return with the exception of root link being world,
  //   because there's no lumping there
  if (_link->getParent() && _link->getParent()->name != "world"
      && FixedJointShouldBeReduced(_link->parent_joint)
      && g_reduceFixedJoints) return;

  if (!jtype.empty())
  {
    TiXmlElement *joint = new TiXmlElement("joint");
    if (jtype == "fixed")
      joint->SetAttribute("type", "revolute");
    else
      joint->SetAttribute("type", jtype);
    joint->SetAttribute("name", _link->parent_joint->name);
    AddKeyValue(joint, "child", _link->name);
    AddKeyValue(joint, "parent", _link->getParent()->name);

    TiXmlElement *jointAxis = new TiXmlElement("axis");
    TiXmlElement *jointAxisLimit = new TiXmlElement("limit");
    TiXmlElement *jointAxisDynamics = new TiXmlElement("dynamics");
    if (jtype == "fixed")
    {
      AddKeyValue(jointAxisLimit, "lower", "0");
      AddKeyValue(jointAxisLimit, "upper", "0");
      AddKeyValue(jointAxisDynamics, "damping", "0");
      AddKeyValue(jointAxisDynamics, "friction", "0");
    }
    else
    {
      sdf::Vector3 rotatedJointAxis =
        _currentTransform.rot.RotateVector(
            sdf::Vector3(_link->parent_joint->axis.x,
              _link->parent_joint->axis.y,
              _link->parent_joint->axis.z));
      double rotatedJointAxisArray[3] =
      { rotatedJointAxis.x, rotatedJointAxis.y, rotatedJointAxis.z };
      AddKeyValue(jointAxis, "xyz",
          Values2str(3, rotatedJointAxisArray));
      if (_link->parent_joint->dynamics)
      {
        AddKeyValue(jointAxisDynamics, "damping",
            Values2str(1, &_link->parent_joint->dynamics->damping));
        AddKeyValue(jointAxisDynamics, "friction",
            Values2str(1, &_link->parent_joint->dynamics->friction));
      }

      if (g_enforceLimits && _link->parent_joint->limits)
      {
        if (jtype == "slider")
        {
          AddKeyValue(jointAxisLimit, "lower",
              Values2str(1, &_link->parent_joint->limits->lower));
          AddKeyValue(jointAxisLimit, "upper",
              Values2str(1, &_link->parent_joint->limits->upper));
          AddKeyValue(jointAxisLimit, "effort",
              Values2str(1, &_link->parent_joint->limits->effort));
          AddKeyValue(jointAxisLimit, "velocity",
              Values2str(1, &_link->parent_joint->limits->velocity));
        }
        else if (_link->parent_joint->type != urdf::Joint::CONTINUOUS)
        {
          double *lowstop  = &_link->parent_joint->limits->lower;
          double *highstop = &_link->parent_joint->limits->upper;
          // enforce ode bounds, this will need to be fixed
          if (*lowstop > *highstop)
          {
            sdfwarn << "urdf2sdf: revolute joint ["
              << _link->parent_joint->name
              << "] with limits: lowStop[" << *lowstop
              << "] > highStop[" << highstop
              << "], switching the two.\n";
            double tmp = *lowstop;
            *lowstop = *highstop;
            *highstop = tmp;
          }
          AddKeyValue(jointAxisLimit, "lower",
              Values2str(1, &_link->parent_joint->limits->lower));
          AddKeyValue(jointAxisLimit, "upper",
              Values2str(1, &_link->parent_joint->limits->upper));
          AddKeyValue(jointAxisLimit, "effort",
              Values2str(1, &_link->parent_joint->limits->effort));
          AddKeyValue(jointAxisLimit, "velocity",
              Values2str(1, &_link->parent_joint->limits->velocity));
        }
      }
    }
    jointAxis->LinkEndChild(jointAxisLimit);
    jointAxis->LinkEndChild(jointAxisDynamics);
    joint->LinkEndChild(jointAxis);

    /* copy sdf extensions data */
    InsertSDFExtensionJoint(joint, _link->parent_joint->name);

    /* add joint to document */
    _root->LinkEndChild(joint);
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateCollision(TiXmlElement* _elem, ConstUrdfLinkPtr _link,
    UrdfCollisionPtr _collision, const std::string &_oldLinkName)
{
  /* begin create geometry node, skip if no collision specified */
  TiXmlElement *sdfCollision = new TiXmlElement("collision");

  /* set its name, if lumped, add original link name */
  if (_oldLinkName == _link->name)
    sdfCollision->SetAttribute("name", _link->name + g_collisionExt);
  else
    sdfCollision->SetAttribute("name", _link->name + g_collisionExt
        + std::string("_") + _oldLinkName);

  /* set transform */
  double pose[6];
  pose[0] = _collision->origin.position.x;
  pose[1] = _collision->origin.position.y;
  pose[2] = _collision->origin.position.z;
  _collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
  AddKeyValue(sdfCollision, "pose", Values2str(6, pose));


  /* add geometry block */
  if (!_collision || !_collision->geometry)
  {
    sdfdbg << "urdf2sdf: collision of link [" << _link->name
           << "] has no <geometry>.\n";
  }
  else
  {
    CreateGeometry(sdfCollision, _collision->geometry);
  }

  /* set additional data from extensions */
  InsertSDFExtensionCollision(sdfCollision, _link->name);

  /* add geometry to body */
  _elem->LinkEndChild(sdfCollision);
}

////////////////////////////////////////////////////////////////////////////////
void CreateVisual(TiXmlElement *_elem, ConstUrdfLinkPtr _link,
    UrdfVisualPtr _visual, const std::string &_oldLinkName)
{
  /* begin create sdf visual node */
  TiXmlElement *sdfVisual = new TiXmlElement("visual");

  /* set its name */
  sdfdbg << "original link name [" << _oldLinkName
         << "] new link name [" << _link->name << "]\n";
  if (_oldLinkName == _link->name)
    sdfVisual->SetAttribute("name", _link->name + g_visualExt);
  else
    sdfVisual->SetAttribute("name", _link->name + g_visualExt
        + std::string("_") + _oldLinkName);

  /* add the visualisation transfrom */
  double pose[6];
  pose[0] = _visual->origin.position.x;
  pose[1] = _visual->origin.position.y;
  pose[2] = _visual->origin.position.z;
  _visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
  AddKeyValue(sdfVisual, "pose", Values2str(6, pose));

  /* insert geometry */
  if (!_visual || !_visual->geometry)
  {
    sdfdbg << "urdf2sdf: visual of link [" << _link->name
           << "] has no <geometry>.\n";
  }
  else
    CreateGeometry(sdfVisual, _visual->geometry);

  /* set additional data from extensions */
  InsertSDFExtensionVisual(sdfVisual, _oldLinkName);

  /* end create _visual node */
  _elem->LinkEndChild(sdfVisual);
}

////////////////////////////////////////////////////////////////////////////////
TiXmlDocument URDF2SDF::InitModelString(const std::string &_urdfStr,
    bool _enforceLimits)
{
  g_enforceLimits = _enforceLimits;

  /* Create a RobotModel from string */
  boost::shared_ptr<urdf::ModelInterface> robotModel =
    urdf::parseURDF(_urdfStr.c_str());

  // an xml object to hold the xml result
  TiXmlDocument sdfXmlOut;

  if (!robotModel)
  {
    sdferr << "Unable to call parseURDF on robot model\n";
    return sdfXmlOut;
  }

  /* create root element and define needed namespaces */
  TiXmlElement *robot = new TiXmlElement("model");

  // set model name to urdf robot name if not specified
  robot->SetAttribute("name", robotModel->getName());

  /* initialize transform for the model, urdf is recursive,
     while sdf defines all links relative to model frame */
  sdf::Pose transform;

  /* parse sdf extension */
  TiXmlDocument urdfXml;
  urdfXml.Parse(_urdfStr.c_str());
  g_extensions.clear();
  this->ParseSDFExtension(urdfXml);

  // Parse robot pose
  ParseRobotOrigin(urdfXml);

  ConstUrdfLinkPtr rootLink = robotModel->getRoot();

  /* Fixed Joint Reduction */
  /* if link connects to parent via fixed joint, lump down and remove link */
  /* set reduceFixedJoints to false will replace fixed joints with
     zero limit revolute joints, otherwise, we reduce it down to its
     parent link recursively */
  /* using the disabledFixedJointLumping option is possible to disable
     fixed joint lumping only for selected joints */
  if (g_reduceFixedJoints)
    ReduceFixedJoints(robot,
        (boost::const_pointer_cast< urdf::Link >(rootLink)));

  if (rootLink->name == "world")
  {
    /* convert all children link */
    for (std::vector<UrdfLinkPtr>::const_iterator
        child = rootLink->child_links.begin();
        child != rootLink->child_links.end(); ++child)
      CreateSDF(robot, (*child), transform);
  }
  else
  {
    /* convert, starting from root link */
    CreateSDF(robot, rootLink, transform);
  }

  /* insert the extensions without reference into <robot> root level */
  InsertSDFExtensionRobot(robot);

  InsertRobotOrigin(robot);

  // add robot to sdfXmlOut
  TiXmlElement *sdf = new TiXmlElement("sdf");

  // URDF is compatible with version 1.4. The automatic conversion script
  // will up-convert URDF to SDF.
  sdf->SetAttribute("version", "1.4");

  sdf->LinkEndChild(robot);
  sdfXmlOut.LinkEndChild(sdf);

  // debug
  // sdfXmlOut.Print();

  return sdfXmlOut;
}

////////////////////////////////////////////////////////////////////////////////
TiXmlDocument URDF2SDF::InitModelDoc(TiXmlDocument* _xmlDoc)
{
  std::ostringstream stream;
  stream << *_xmlDoc;
  std::string urdfStr = stream.str();
  return InitModelString(urdfStr);
}

////////////////////////////////////////////////////////////////////////////////
TiXmlDocument URDF2SDF::InitModelFile(const std::string &_filename)
{
  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(_filename))
  {
    return this->InitModelDoc(&xmlDoc);
  }
  else
    sdferr << "Unable to load file[" << _filename << "].\n";

  return xmlDoc;
}

////////////////////////////////////////////////////////////////////////////////
bool FixedJointShouldBeReduced(boost::shared_ptr<urdf::Joint> _jnt)
{
    // A joint should be lumped only if its type is fixed and
    // the disabledFixedJointLumping joint option is not set
    return (_jnt->type == urdf::Joint::FIXED &&
              (g_fixedJointsNotReduced.find(_jnt->name) ==
                 g_fixedJointsNotReduced.end()) );
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionSensorTransformReduction(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    sdf::Pose _reductionTransform)
{
  // overwrite <xyz> and <rpy> if they exist
  if ((*_blobIt)->ValueStr() == "sensor")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform

    // debug print
    // for (TiXmlNode* elIt = (*_blobIt)->FirstChild();
    //      elIt; elIt = elIt->NextSibling())
    // {
    //   std::ostringstream streamIn;
    //   streamIn << *elIt;
    //   sdfdbg << "    " << streamIn << "\n";
    // }

    {
      TiXmlNode* oldPoseKey = (*_blobIt)->FirstChild("pose");
      /// @todo: FIXME:  we should read xyz, rpy and aggregate it to
      /// reductionTransform instead of just throwing the info away.
      if (oldPoseKey)
        (*_blobIt)->RemoveChild(oldPoseKey);
    }

    // convert reductionTransform to values
    urdf::Vector3 reductionXyz(_reductionTransform.pos.x,
        _reductionTransform.pos.y,
        _reductionTransform.pos.z);
    urdf::Rotation reductionQ(_reductionTransform.rot.x,
        _reductionTransform.rot.y,
        _reductionTransform.rot.z,
        _reductionTransform.rot.w);

    urdf::Vector3 reductionRpy;
    reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);

    // output updated pose to text
    std::ostringstream poseStream;
    poseStream << reductionXyz.x << " " << reductionXyz.y
      << " " << reductionXyz.z << " " << reductionRpy.x
      << " " << reductionRpy.y << " " << reductionRpy.z;
    TiXmlText* poseTxt = new TiXmlText(poseStream.str());

    TiXmlElement* poseKey = new TiXmlElement("pose");
    poseKey->LinkEndChild(poseTxt);

    (*_blobIt)->LinkEndChild(poseKey);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionProjectorTransformReduction(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    sdf::Pose _reductionTransform)
{
  // overwrite <pose> (xyz/rpy) if it exists
  if ((*_blobIt)->ValueStr() == "projector")
  {
    /*
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    for (TiXmlNode* elIt = (*_blobIt)->FirstChild();
    elIt; elIt = elIt->NextSibling())
    {
    std::ostringstream streamIn;
    streamIn << *elIt;
    sdfdbg << "    " << streamIn << "\n";
    }
    */

    /* should read <pose>...</pose> and agregate reductionTransform */
    TiXmlNode* poseKey = (*_blobIt)->FirstChild("pose");
    // read pose and save it

    // remove the tag for now
    if (poseKey) (*_blobIt)->RemoveChild(poseKey);

    // convert reductionTransform to values
    urdf::Vector3 reductionXyz(_reductionTransform.pos.x,
        _reductionTransform.pos.y,
        _reductionTransform.pos.z);
    urdf::Rotation reductionQ(_reductionTransform.rot.x,
        _reductionTransform.rot.y,
        _reductionTransform.rot.z,
        _reductionTransform.rot.w);

    urdf::Vector3 reductionRpy;
    reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);

    // output updated pose to text
    std::ostringstream poseStream;
    poseStream << reductionXyz.x << " " << reductionXyz.y
      << " " << reductionXyz.z << " " << reductionRpy.x
      << " " << reductionRpy.y << " " << reductionRpy.z;
    TiXmlText* poseTxt = new TiXmlText(poseStream.str());

    poseKey = new TiXmlElement("pose");
    poseKey->LinkEndChild(poseTxt);

    (*_blobIt)->LinkEndChild(poseKey);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionContactSensorFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;
  if ((*_blobIt)->ValueStr() == "sensor")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* contact = (*_blobIt)->FirstChild("contact");
    if (contact)
    {
      TiXmlNode* collision = contact->FirstChild("collision");
      if (collision)
      {
        if (GetKeyValueAsString(collision->ToElement()) ==
            linkName + g_collisionExt)
        {
          contact->RemoveChild(collision);
          TiXmlElement* collisionNameKey = new TiXmlElement("collision");
          std::ostringstream collisionNameStream;
          collisionNameStream << newLinkName << g_collisionExt
            << "_" << linkName;
          TiXmlText* collisionNameTxt = new TiXmlText(
              collisionNameStream.str());
          collisionNameKey->LinkEndChild(collisionNameTxt);
          contact->LinkEndChild(collisionNameKey);
        }
        // @todo: FIXME: chagning contact sensor's contact collision
        //   should trigger a update in sensor offset as well.
        //   But first we need to implement offsets in contact sensors
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionPluginFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link,
    const std::string &_pluginName, const std::string &_elementName,
    sdf::Pose _reductionTransform)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;
  if ((*_blobIt)->ValueStr() == _pluginName)
  {
    // replace element containing _link names to parent link names
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* elementNode = (*_blobIt)->FirstChild(_elementName);
    if (elementNode)
    {
      if (GetKeyValueAsString(elementNode->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(elementNode);
        TiXmlElement* bodyNameKey = new TiXmlElement(_elementName);
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
        /// @todo update transforms for this sdf plugin too

        // look for offset transforms, add reduction transform
        TiXmlNode* xyzKey = (*_blobIt)->FirstChild("xyzOffset");
        if (xyzKey)
        {
          urdf::Vector3 v1 = ParseVector3(xyzKey);
          _reductionTransform.pos = sdf::Vector3(v1.x, v1.y, v1.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->RemoveChild(xyzKey);
        }
        TiXmlNode* rpyKey = (*_blobIt)->FirstChild("rpyOffset");
        if (rpyKey)
        {
          urdf::Vector3 rpy = ParseVector3(rpyKey, M_PI/180.0);
          _reductionTransform.rot =
            sdf::Quaternion::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->RemoveChild(rpyKey);
        }

        // pass through the parent transform from fixed joint reduction
        _reductionTransform = inverseTransformToParentFrame(_reductionTransform,
            _link->parent_joint->parent_to_joint_origin_transform);

        // create new offset xml blocks
        xyzKey = new TiXmlElement("xyzOffset");
        rpyKey = new TiXmlElement("rpyOffset");

        // create new offset xml blocks
        urdf::Vector3 reductionXyz(_reductionTransform.pos.x,
            _reductionTransform.pos.y,
            _reductionTransform.pos.z);
        urdf::Rotation reductionQ(_reductionTransform.rot.x,
            _reductionTransform.rot.y, _reductionTransform.rot.z,
            _reductionTransform.rot.w);

        std::ostringstream xyzStream, rpyStream;
        xyzStream << reductionXyz.x << " " << reductionXyz.y << " "
          << reductionXyz.z;
        urdf::Vector3 reductionRpy;
        reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);
        rpyStream << reductionRpy.x << " " << reductionRpy.y << " "
          << reductionRpy.z;

        TiXmlText* xyzTxt = new TiXmlText(xyzStream.str());
        TiXmlText* rpyTxt = new TiXmlText(rpyStream.str());

        xyzKey->LinkEndChild(xyzTxt);
        rpyKey->LinkEndChild(rpyTxt);

        (*_blobIt)->LinkEndChild(xyzKey);
        (*_blobIt)->LinkEndChild(rpyKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionProjectorFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  // updates _link reference for <projector> inside of
  // projector plugins
  // update from <projector>MyLinkName/MyProjectorName</projector>
  // to <projector>NewLinkName/MyProjectorName</projector>
  TiXmlNode* projectorElem = (*_blobIt)->FirstChild("projector");
  {
    if (projectorElem)
    {
      std::string projectorName =  GetKeyValueAsString(
          projectorElem->ToElement());
      // extract projector _link name and projector name
      size_t pos = projectorName.find("/");
      if (pos == std::string::npos)
      {
        sdferr << "no slash in projector reference tag [" << projectorName
          << "], expecting linkName/projector_name.\n";
      }
      else
      {
        std::string projectorLinkName = projectorName.substr(0, pos);

        if (projectorLinkName == linkName)
        {
          // do the replacement
          projectorName = newLinkName + "/" +
            projectorName.substr(pos+1, projectorName.size());

          (*_blobIt)->RemoveChild(projectorElem);
          TiXmlElement *bodyNameKey = new TiXmlElement("projector");
          std::ostringstream bodyNameStream;
          bodyNameStream << projectorName;
          TiXmlText *bodyNameTxt = new TiXmlText(bodyNameStream.str());
          bodyNameKey->LinkEndChild(bodyNameTxt);
          (*_blobIt)->LinkEndChild(bodyNameKey);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionGripperFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  if ((*_blobIt)->ValueStr() == "gripper")
  {
    TiXmlNode* gripperLink = (*_blobIt)->FirstChild("gripper_link");
    if (gripperLink)
    {
      if (GetKeyValueAsString(gripperLink->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(gripperLink);
        TiXmlElement* bodyNameKey = new TiXmlElement("gripper_link");
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
    TiXmlNode* palmLink = (*_blobIt)->FirstChild("palm_link");
    if (palmLink)
    {
      if (GetKeyValueAsString(palmLink->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(palmLink);
        TiXmlElement* bodyNameKey = new TiXmlElement("palm_link");
        std::ostringstream bodyNameStream;
        bodyNameStream << newLinkName;
        TiXmlText* bodyNameTxt = new TiXmlText(bodyNameStream.str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionJointFrameReplace(
    std::vector<TiXmlElementPtr>::iterator _blobIt,
    UrdfLinkPtr _link)
{
  std::string linkName = _link->name;
  std::string newLinkName = _link->getParent()->name;

  if ((*_blobIt)->ValueStr() == "joint")
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    TiXmlNode* parent = (*_blobIt)->FirstChild("parent");
    if (parent)
    {
      if (GetKeyValueAsString(parent->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(parent);
        TiXmlElement* parentNameKey = new TiXmlElement("parent");
        std::ostringstream parentNameStream;
        parentNameStream << newLinkName;
        TiXmlText* parentNameTxt = new TiXmlText(parentNameStream.str());
        parentNameKey->LinkEndChild(parentNameTxt);
        (*_blobIt)->LinkEndChild(parentNameKey);
      }
    }
    TiXmlNode* child = (*_blobIt)->FirstChild("child");
    if (child)
    {
      if (GetKeyValueAsString(child->ToElement()) == linkName)
      {
        (*_blobIt)->RemoveChild(child);
        TiXmlElement* childNameKey = new TiXmlElement("child");
        std::ostringstream childNameStream;
        childNameStream << newLinkName;
        TiXmlText* childNameTxt = new TiXmlText(childNameStream.str());
        childNameKey->LinkEndChild(childNameTxt);
        (*_blobIt)->LinkEndChild(childNameKey);
      }
    }
    /// @todo add anchor offsets if parent link changes location!
  }
}
