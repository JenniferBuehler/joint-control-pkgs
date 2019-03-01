#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif

#include <gazebo_joint_control/GazeboJointControl.h>
#include <gazebo_joint_control/GazeboVersionHelpers.h>
#include <convenience_math_functions/MathFunctions.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <map>
#include <string>
#include <algorithm>
#include <vector>

#define DEFAULT_MAX_FORCE 1

// maximum velocity for arm joints (rads/sec)
#define DEFAULT_MAX_VEL 0.1 

#define KP_POS 200
#define KI_POS 10
#define KD_POS 1

#define KP_VEL 10000
#define KI_VEL 100
#define KD_VEL 0

#define INIT_WITH_POS false

#define DISABLE_GRAVITY false

#define USE_FORCE true

// don't use home pose as initial pose, but 0 angles instead
// #define USE_0_INIT_POSE

#define UPDATE_RATE 1000

// the major version of Gazebo from which on 
// SetVelocityLimits() works
#define GAZEBO_MAJOR_MAXVALS_WORKING 3

using convenience_math_functions::MathFunctions;
using arm_components_name_manager::ArmComponentsNameManager;

namespace gazebo
{

using physics::JointControllerThreadsafe;

GazeboJointControl::GazeboJointControl():
    nh(""),
    loadedVelocityControllers(false)
{
    ROS_INFO("Creating GazeboJointControl plugin");
    //gazebo::common::Console::SetQuiet(false);
}

GazeboJointControl::~GazeboJointControl()
{
}

void GazeboJointControl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO("Loading GazeboJointControl");
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    std::string armNamespace = _parent->GetName();
    if (_sdf->HasElement("robot_components_namespace"))
    {
        sdf::ElementPtr armParamElem = _sdf->GetElement("robot_components_namespace");
        armNamespace = armParamElem->Get<std::string>();
    }
    else
    {
        ROS_WARN("SDF Element 'robot_components_namespace' not defined, so using robot name as namespace for components.");
    }


    ROS_INFO_STREAM("GazeboJointControl: Loading arm component parameters from "<< armNamespace);
    joints = ArmComponentsNameManagerPtr(new ArmComponentsNameManager(armNamespace,false));
    if (!joints->waitToLoadParameters(1, 3, 0.5))
    {
        ROS_FATAL_STREAM("Cannot load arm components for robot "<<_parent->GetName()<<" from namespace "<<armNamespace);
        return;
    }
    ROS_INFO_STREAM("GazeboJointControl: Arm component parameters loaded.");

    bool loadVelocityControllers = true;
    nh.param<bool>("gazebo/load_velocity_controllers",loadVelocityControllers,loadVelocityControllers);
    ROS_INFO_STREAM("Set to load velocity controlers: "<<loadVelocityControllers);

    // Instantiate the threadsafe Joint Controller of parent model
    // -------------
    
    // First, check if there is already a thread-safe joint controller for the parent model
    physics::BasePtr jcChild = _parent->GetChild(physics::JointControllerThreadsafe::UniqueName());
    if (!jcChild.get())
    {   // no joint controller loaded yet, so create a new one
        physics::JointControllerPtr _modelJointController = _parent->GetJointController();
        if (!_modelJointController.get())
        {
            ROS_ERROR("Cannot load GazeboJointControl if no default JointController is set for the model");
            throw std::string("Cannot load GazeboJointControl if no default JointController is set for the model");
        }

        JointControllerThreadsafe::JointControllerImplPtr _newControllerPtr(new JointControllerThreadsafe::JointControllerImpl(_parent));
        jointController = physics::JointControllerThreadsafePtr(
                              new JointControllerThreadsafe(_parent, _newControllerPtr));
        //jointController = physics::JointControllerThreadsafePtr(
        //                      new physics::JointControllerThreadsafe(_parent, _modelJointController));
        _parent->AddChild(jointController);
    }
    else
    {   // use the existing joint controller
        physics::JointControllerThreadsafePtr ptr =
            boost::dynamic_pointer_cast<physics::JointControllerThreadsafe>(jcChild);
        if (!ptr.get())
        {
            ROS_ERROR_STREAM("Cannot load GazeboJointStateClient if child '"
                             << physics::JointControllerThreadsafe::UniqueName()
                             << "' is not of class JointControllerThreadsafe");
            throw std::string("Cannot load GazeboJointStateClient if child '"
                              + physics::JointControllerThreadsafe::UniqueName()
                              + "' is not of class JointControllerThreadsafe");
        }
        jointController = ptr;
    }



    // get joint names from parameters
    std::vector<std::string> joint_names;
    joints->getJointNames(joint_names, true);
    const std::vector<float>& arm_init = joints->getArmJointsInitPose();
    const std::vector<float>& gripper_init = joints->getGripperJointsInitPose();

#if GAZEBO_MAJOR_VERSION >= GAZEBO_ADVANCED_JOINTCONTROLLER_VERSION
    // Print the joint names to help debugging
    if (jointController)
    {
      std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();
      for (std::map< std::string, physics::JointPtr>::iterator it = jntMap.begin(); it != jntMap.end(); ++it)
      {
          physics::JointPtr j = it->second;
          ROS_INFO_STREAM("Gazebo joint: '" << j->GetName() << "' is registered as '" << it->first << "'");
      }
    }
#endif

    // check if the joint names maintained in 'joints' match the names in gazebo,
    // that the joints can be used by this class, and if yes, load PID controllers.
    int i = 0;
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {
        // ROS_INFO_STREAM("Local joint name: '"<<*it<<"'");

        physics::JointPtr joint = _parent->GetJoint(*it);
        if (!joint.get())
        {
            ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
            throw std::string("Joint not found");
        }

        if (GetDOF(joint) != 1)
        {
            ROS_FATAL("GazeboJointControl: joints must have exactly one axis");
            throw std::string("GazeboJointControl: joints must have exactly one axis");
        }

        jointController->AddJoint(joint);
        
        bool isGripper = i > 5;

        float max_force, max_velocity;
        GetMaxVals(*it, max_force, max_velocity);

        // ROS_INFO_STREAM("Setting max vals for joint " << *it << ": f=" << max_force << ", v=" << max_velocity);
        joint->SetEffortLimit(0, max_force);

#if GAZEBO_MAJOR_VERSION >= GAZEBO_JADE_VERSION
        joint->SetVelocityLimit(0, max_velocity);
#endif
            
    // XXX TODO March 29th this causes Gazebo warnings (only visible when gazebo::common::Console::SetQuiet(false)), but I think this is
    // what actually made it work in the end.. have to test later again and find the proper way to make this work!

#if GAZEBO_MAJOR_VERSION > 2
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
        // XXX TODO investigate: This was deprecated in ODE gazebo 5? Also works without it..
        if (!UseForce()) {
            ROS_INFO("Setting fmax prints a warning/error but it somehow still is required...");
            joint->SetParam("fmax", 0, max_force);
        }
#else
        joint->SetMaxForce(0,max_force);
#endif
        
        if (DisableGravity())
        {
            physics::LinkPtr link = joint->GetChild();
            // physics::LinkPtr link = joint->GetJointLink(0);
            if (link.get())
            {
                ROS_INFO_STREAM("Disable gravity for link " << link->GetName());
                link->SetGravityMode(false);
            }
        }

        float max_val = UseForce() ? max_force : max_velocity;
        
        std::string scopedName = joint->GetScopedName();

        float imin = 0;
        float imax = 0;
        float kp, ki, kd;
        GetPosGains(*it, kp, ki, kd);
        jointController->SetPositionPID(scopedName, gazebo::common::PID(kp, ki, kd, imin, imax, max_val, -max_val));

        if (loadVelocityControllers)
        {
            GetVelGains(*it, kp, ki, kd);
            //imax = 0.2;
            jointController->SetVelocityPID(scopedName, gazebo::common::PID(kp, ki, kd, imin, imax, max_val, -max_val));
        }

        double initVal = isGripper ? gripper_init[i - joints->numArmJoints()] : arm_init[i];
        initVal = MathFunctions::capToPI(initVal);

        // ROS_INFO_STREAM("Setting initial position for " << joint_names[i] << ": " << initVal);

        double lowLimit = GetLowerLimit(joint);
        double upLimit = GetUpperLimit(joint);
        // ROS_INFO("Joint limits: %f, %f",lowLimit,upLimit);
        if (initVal > upLimit) initVal = upLimit;
        if (initVal < lowLimit) initVal = lowLimit;

#ifdef USE_0_INIT_POSE
        initVal = 0;
#endif

        jointController->SetJointPosition(scopedName, initVal);

        if (!jointController->SetPositionTarget(scopedName, initVal))
        {
            ROS_ERROR_STREAM("Could not set position target for " << joint_names[i]);
        }
        // ROS_INFO_STREAM("Set position target for " << joint_names[i]);
        ++i;
    }

    model = _parent;

    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJointControl::WorldUpdate, this));
    // ros::NodeHandle nh;
    // update_connection = nh.createTimer(ros::Duration(1.0 / UPDATE_RATE), &GazeboJointControl::WorldUpdate, this);
   
    loadedVelocityControllers = loadVelocityControllers;
    ROS_INFO("GazeboJointControl loaded.");
}

double GazeboJointControl::capTargetVel(const physics::JointPtr joint, const float targetVel,
        const bool useEndTargetPos, const float endTargetPos, const float tolerance) const
{
    if (joint.get() == NULL)
    {
        ROS_FATAL("Passing NULL joint!");
        return targetVel;
    }

    float velocity = targetVel;
    int axis = 0;

#if GAZEBO_MAJOR_VERSION > GAZEBO_MAJOR_MAXVALS_WORKING
    float maxVel = fabs(joint->GetVelocityLimit(axis));  
    float maxForce = fabs(joint->GetEffortLimit(axis));
#else
    float maxForce, maxVel;
    GetMaxVals(joint->GetName(), maxForce, maxVel);
  // double maxForce=fabs(joint->GetMaxForce(axis));
#endif
  // ROS_INFO_STREAM("Max vel "<<joint->GetName()<<": "<<maxVel);
  // ROS_INFO_STREAM("Max f "<<joint->GetName()<<": "<<maxForce);

    double distToTargetPos = 0;
    bool closeToTargetPos = false;

    double currAngle = GetPosition(joint, axis);

    if (useEndTargetPos)
    {
        float target = MathFunctions::capToPI(endTargetPos);

        double _currAngle = MathFunctions::capToPI(currAngle);
        distToTargetPos = (target - _currAngle);
        distToTargetPos = MathFunctions::capToPI(distToTargetPos);

        closeToTargetPos = fabs(distToTargetPos) < tolerance;
    }

    if (closeToTargetPos)
    {
//        ROS_INFO_STREAM("Close to target: "<<joint->GetName()<<" low, pos "<<currAngle);
        velocity = 0;
    }

    velocity = std::min(velocity, maxVel);
    velocity = std::max(velocity, -maxVel);

    // Now, prevent the joints to get close to any possible limits, because
    // Gazebo then has weird issues.

    double lowLimit = GetLowerLimit(joint, axis);
    double upLimit = GetUpperLimit(joint, axis);
    // ROS_INFO_STREAM("Low limit "<<joint->GetName()<<": "<<lowLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("High limit "<<joint->GetName()<<": "<<upLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("Current vel: "<<velocity);

    // XXX TODO. This is not a safe hack to prevent getting too close, because it does not consider
    // the update rate. Setting a high velocity when update rate is low
    // can very well exceed the joint limits.
    if ((fabs(currAngle - lowLimit) < JOINTLIMIT_PADDING) && (velocity < 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" low, pos "<<currAngle);
        velocity = 0;
    }
    if ((fabs(upLimit - currAngle) < JOINTLIMIT_PADDING) && (velocity > 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" high, pos "<<currAngle);
        velocity = 0;
    }

    return velocity;
}


double GazeboJointControl::capTargetForce(const physics::JointPtr joint, const float targetForce, const bool considerJointLimits) const
{
    if (joint.get() == NULL)
    {
        ROS_FATAL("Passing NULL joint!");
        return targetForce;
    }

    float force = targetForce;
    int axis = 0;
  
#if GAZEBO_MAJOR_VERSION > GAZEBO_MAJOR_MAXVALS_WORKING
    float maxVel = fabs(joint->GetVelocityLimit(axis));  
    float maxForce = fabs(joint->GetEffortLimit(axis));
#else
    float maxForce, maxVel;
    GetMaxVals(joint->GetName(), maxForce, maxVel);
    // double maxForce=fabs(joint->GetMaxForce(axis));
#endif
    // ROS_INFO_STREAM("Max vel "<<joint->GetName()<<": "<<maxVel);
    // ROS_INFO_STREAM("Max f "<<joint->GetName()<<": "<<maxForce);

    force = std::min(force, maxForce);
    force = std::max(force, -maxForce);
    
    double currAngle = GetPosition(joint, axis);

    // Now, prevent the joints to get close to any possible limits, because
    // Gazebo then has weird issues.

    double lowLimit = GetLowerLimit(joint);
    double upLimit = GetUpperLimit(joint);
    // ROS_INFO_STREAM("Low limit "<<joint->GetName()<<": "<<lowLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("High limit "<<joint->GetName()<<": "<<upLimit<<" curr: "<<currAngle);
    // ROS_INFO_STREAM("Current vel: "<<force);

    // XXX TODO. This is not a safe hack to prevent getting too close, because actually
    // an opposite force may have to be applied in order for the joint not to "fall down" beyond
    // its limits. But this would need to be handled by PID controller...
    if ((fabs(currAngle - lowLimit) < JOINTLIMIT_PADDING) && (force < 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" low, pos "<<currAngle);
        force = 0;
    }
    if ((fabs(upLimit - currAngle) < JOINTLIMIT_PADDING) && (force > 0))
    {
        // ROS_INFO_STREAM("Skipping "<<joint->GetName()<<" high, pos "<<currAngle);
        force = 0;
    }

    return force;
}

bool GazeboJointControl::UpdateJoints()
{
    if (!UseForce())
    {
        ROS_FATAL_STREAM("Cannot use the base class GazeboJointControl in velocity mode. Always force is used.");
        return false;
    }

    if (!model.get())
    {
        ROS_ERROR("Cannot update GazeboJointControl if no model is set");
        return false;
    }
    if (!jointController.get())
    {
        ROS_ERROR("Cannot load GazeboJointControl if no default JointController is set for the model");
        return false;
    }
    jointController->Update();
    return true;
}


void GazeboJointControl::WorldUpdate()
//void GazeboJointControl::WorldUpdate(const ros::TimerEvent& t)
{
    if (!UpdateJoints())
    {
        throw std::string("Cannot update GazeboJointControl");
    }
}


void GazeboJointControl::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}


bool GazeboJointControl::isGripper(const physics::JointPtr& joint) const
{
    return joints->isGripper(joint->GetName()) || joints->isGripper(joint->GetScopedName());
}

bool GazeboJointControl::DisableGravity() const
{
    return DISABLE_GRAVITY;
}

bool GazeboJointControl::UseForce() const
{
    return USE_FORCE;
}

void GazeboJointControl::GetDefaultPosGains(float& kp, float& ki, float& kd) const
{
    kp = KP_POS;
    ki = KI_POS;
    kd = KD_POS;
}

void GazeboJointControl::GetDefaultVelGains(float& kp, float& ki, float& kd) const
{
    kp = KP_VEL;
    ki = KI_VEL;
    kd = KD_VEL;
}
    
void GazeboJointControl::GetDefaultMaxVals(float& force, float& velocity) const
{
    force=DEFAULT_MAX_FORCE;
    velocity=DEFAULT_MAX_VEL;
}


void GazeboJointControl::GetPosGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    GetDefaultPosGains(kp, ki, kd);
    if (!joints->GetPosGains(jointName, kp, ki, kd))
    {
        ROS_ERROR_STREAM("GazeboJointControl::GetPosGains: Joint " << jointName << " not maintained by the joint manager");
    }
    // ROS_INFO_STREAM("Using position PID values for joint " << jointName << ": p=" << kp << ", i=" << ki << ", d=" << kd);
}


void GazeboJointControl::GetVelGains(const std::string& jointName, float& kp, float& ki, float& kd) const
{
    GetDefaultVelGains(kp, ki, kd);
    if (!joints->GetVelGains(jointName, kp, ki, kd))
    {
        ROS_ERROR_STREAM("GazeboJointControl::GetVelGains: Joint " << jointName << " not maintained by the joint manager");
    }

    // ROS_INFO_STREAM("Using velocity PID values for joint " << jointName << ": p=" << kp << ", i=" << ki << ", d=" << kd);
}

void GazeboJointControl::GetMaxVals(const std::string& jointName, float& force, float& velocity) const
{
    GetDefaultMaxVals(force, velocity);
    if (!joints->GetMaxVals(jointName, force, velocity))
    {
        ROS_ERROR_STREAM("GazeboJointControl::GetMaxVals: Joint " << jointName << " not maintained by the joint manager");
    }
}

}  // namespace gazebo
