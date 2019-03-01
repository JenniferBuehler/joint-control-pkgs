#ifndef GAZEBO_VERSION_HELPERS_H
#define GAZEBO_VERSION_HELPERS_H

#include <gazebo/gazebo.hh>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
inline
unsigned int GetDOF(const gazebo::physics::JointPtr &joint)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return joint->DOF();
#else
  return joint->GetAngleCount();
#endif
}

////////////////////////////////////////////////////////////////////////////////
inline
double GetPosition(const gazebo::physics::JointPtr &joint,
                   unsigned int axis = 0)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return joint->Position(axis);
#else
  return joint->GetAngle(axis).Radian();
#endif
}

////////////////////////////////////////////////////////////////////////////////
inline
double GetUpperLimit(const gazebo::physics::JointPtr &joint,
                     unsigned int axis = 0)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return joint->UpperLimit(axis);
#else
  return joint->GetUpperLimit(axis).Radian();
#endif
}

////////////////////////////////////////////////////////////////////////////////
inline
double GetLowerLimit(const gazebo::physics::JointPtr &joint,
                     unsigned int axis = 0)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return joint->LowerLimit(axis);
#else
  return joint->GetLowerLimit(axis).Radian();
#endif
}
////////////////////////////////////////////////////////////////////////////////
inline
common::Time GetSimTime(const gazebo::physics::WorldPtr &world)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return world->SimTime();
#else
  return world->GetSimTime();
#endif
}


}  // namespace

#endif  // GAZEBO_VERSION_HELPERS_H

