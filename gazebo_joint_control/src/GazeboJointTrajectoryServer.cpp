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


#include <gazebo_joint_control/GazeboJointTrajectoryServer.h>
#include <gazebo_joint_control/GazeboVersionHelpers.h>
#include <convenience_math_functions/MathFunctions.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <map>
#include <string>
#include <vector>

// if defined, then instead of reading gazebo joint
// velocities with Joint::GetVelocity(), the
// JointVelocityTracker is used.
// #define USE_VELOCITY_TRACKER

using convenience_math_functions::MathFunctions;
using joint_trajectory_execution::TrajectoryActionServer;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboJointTrajectoryServer);

GazeboJointTrajectoryServer::GazeboJointTrajectoryServer():
    finalize(false)
{
    ROS_INFO("Creating GazeboJointTrajectoryServer plugin");
}

GazeboJointTrajectoryServer::~GazeboJointTrajectoryServer()
{
}

////////////////////////////////////////////////////////////////////////////////
void GazeboJointTrajectoryServer::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO("Loading GazeboJointTrajectoryServer");

    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    physics::BasePtr jcChild = _parent->GetChild(physics::JointControllerThreadsafe::UniqueName());
    if (!jcChild.get())
    {
        ROS_ERROR("Cannot load GazeboJointTrajectoryServer if no default JointControllerThreadsafe is set for the model");
        throw std::string("Cannot load GazeboJointTrajectoryServer if no default JointController is set for the model");
    }
    physics::JointControllerThreadsafePtr ptr = boost::dynamic_pointer_cast<physics::JointControllerThreadsafe>(jcChild);
    if (!ptr.get())
    {
        ROS_ERROR_STREAM("Cannot load GazeboJointTrajectoryServer if child '"
                         << physics::JointControllerThreadsafe::UniqueName()
                         << "' is not of class JointControllerThreadsafe");
        throw std::string("Cannot load GazeboJointTrajectoryServer if child '"
                          + physics::JointControllerThreadsafe::UniqueName()
                          + "' is not of class JointControllerThreadsafe");
    }
    jointController = ptr;

    // see which namespace to read arm components and trajectory parameters from
    std::string trajNamespace = _parent->GetName();
    std::string armNamespace = _parent->GetName();
    if (_sdf->HasElement("robot_components_namespace"))
    {
        sdf::ElementPtr armParamElem = _sdf->GetElement("robot_components_namespace");
        armNamespace = armParamElem->Get<std::string>();
    }
    else
    {
        ROS_WARN("GazeboJointTrajectoryServer: SDF Element 'robot_components_namespace' not defined, so using robot name as namespace for component names.");
    }
    if (_sdf->HasElement("trajectory_param_namespace"))
    {
        sdf::ElementPtr trajParamElem = _sdf->GetElement("trajectory_param_namespace");
        trajNamespace = trajParamElem->Get<std::string>();
    }
    else
    {
        ROS_WARN_STREAM("GazeboJointTrajectoryServer: SDF Element 'trajectory_param_namespace' not defined, so using robot components namespace '"
               <<armNamespace<<"' for trajectory parameters as well.");
        trajNamespace = armNamespace;
    }

    ROS_INFO_STREAM("GazeboJointTrajectoryServer: Creating trajectory action server, reading ROS parameters from namespaces "<<trajNamespace<<" and "<<armNamespace);
    trajectory_action_server = TrajectoryActionServer::CreateFromParameters(trajNamespace, armNamespace,
            trajectoryPos, trajectoryVel, currentAngles, currentVels, data_lock);


    // Check joints and add velocity trackers
    std::vector<std::string> joint_names;
    std::string prepend = "";
    trajectory_action_server->getArmNamesMgr().getJointNames(joint_names, true, prepend);

    // check if the joint names maintained in 'joints' match the names in gazebo,
    // that the joints can be used by this class.
    // If yes, do any initializations that may be necessary as well.
    int i = 0;
    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {

        physics::JointPtr joint = _parent->GetJoint(*it);
        if (!joint.get())
        {
            ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
            throw std::string("Joint not found");
        }


        std::string scopedName = joint->GetScopedName();
        std::map<std::string, physics::JointPtr >::iterator jit = jntMap.find(scopedName);
        if (jit == jntMap.end())
        {
            ROS_ERROR_STREAM("Joint name " << *it << " not found in joint controller joints.");
            throw std::string("Joint not found");
        }
       
        // ROS_INFO_STREAM("GazeboJointTrajectoryServer: joint name: '"<<*it<<"' (scoped name '"<<scopedName<<"')");

#ifdef USE_VELOCITY_TRACKER
        int axis=0; 
        velTracker.add(*it,&joint);
#endif
        ++i;
    }

    model = _parent;

    // update_connection =
    //  event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJointControl::WorldUpdate, this));
    update_connection =
        nh.createTimer(ros::Duration(1.0 / 1000.0), &GazeboJointTrajectoryServer::WorldUpdate, this);
}


////////////////////////////////////////////////////////////////////////////////
// void GazeboJointControl::WorldUpdate()
void GazeboJointTrajectoryServer::WorldUpdate(const ros::TimerEvent& t)
{
    std::vector<std::string> joint_names;
    std::string prepend = "";
    trajectory_action_server->getArmNamesMgr().getJointNames(joint_names, true, prepend);
   
#ifdef USE_VELOCITY_TRACKER
    // update velocity tracker so that we can read the actual physical
    // joint velocity averaged over a brief time. 
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {
        int axis=0;
        gazebo::physics::JointPtr joint;
        if (!velTracker.getJoint(*it,joint) || !joint.get())
        {
            ROS_ERROR_STREAM("Inconsistency: Joint "<<*it<<" should have been added to velocity tracker.");
            throw std::string("Inconsistency: Joint should have been added to velocity tracker.");
        } 
        velTracker.update(*it, joint->GetAngle(axis).Radian(), ros::Time::now());
    }
#endif

     // read the current joint values and store in vector
    data_lock.lock();
    GazeboJointTrajectoryServer::readJointStates(currentAngles, currentVels);
    data_lock.unlock();


    // set current joint positions in trajectory

    // ROS_INFO_STREAM("Executing trajectory goal!");

    if (!jointController.get())
    {
        ROS_ERROR("Cannot load GazeboJointTrajectoryServer if no JointController is set for the model");
        return;
    }


    if (!trajectory_action_server->goalActive())
    {
        // no trajectory is being executed, so there is no need to update

        // however it may be necessary to ensure the last trajectory point positions
        // are finalised, because now all velocities will be zero. Must at least
        // once ensure current target positions are set in the controller.
        if (finalize) {
            // ROS_INFO_STREAM("Finalizing trajectory");

            boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();
            // clear out all possible current targets. Necessary e.g. to clear out
            // exitsing velocity targets, when only positions are set now.
            // This is safe to use here because we are setting new targets for *all*
            // joints of the arm.
            jointController->Reset();
            int i = 0;
            for (std::vector<std::string>::iterator it = joint_names.begin();
                    it != joint_names.end(); ++it)
            {
                // ROS_INFO_STREAM("Local joint name: '"<<*it<<"'");
                physics::JointPtr joint = model->GetJoint(*it);
                if (!joint.get())
                {
                    ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
                    throw std::string("Joint not found");
                }

                std::string scopedName = joint->GetScopedName();
                data_lock.lock();
                // ROS_INFO_STREAM("GazeboJointTrajectoryServer: Setting final position target "<<scopedName<<": "<<trajectoryPos[i]);
                if (!jointController->SetPositionTarget(scopedName, MathFunctions::capToPI(trajectoryPos[i])))
                {
                    ROS_ERROR_STREAM_ONCE("Could not set position target for "<<joint->GetName()<<
                           ". Can't control joint. This message is only printed once.");
                }
                data_lock.unlock();
                ++i;
            }
            finalize=false;
        } 
        return;
    }

 
    finalize=true;

    boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();


    // clear out all possible current targets. Necessary e.g. to clear out
    // exitsing velocity targets, when only positions are set now...
    // or because we moved on to next trajectory point and the positions
    // are not valid any more. It's ok to use here because we are setting new
    // targets for every joint anyway.
    jointController->Reset();

    // check if the joint names maintained in 'joints' match the names in gazebo,
    // that the joints can be used by this class, and if yes, load PID controllers.
    int i = 0;
    for (std::vector<std::string>::iterator it = joint_names.begin();
            it != joint_names.end(); ++it)
    {
        // ROS_INFO_STREAM("Local joint name: '"<<*it<<"'");

        physics::JointPtr joint = model->GetJoint(*it);
        if (!joint.get())
        {
            ROS_FATAL_STREAM("Joint name " << *it << " not found as robot joint");
            throw std::string("Joint not found");
        }

        std::string scopedName = joint->GetScopedName();
        // ROS_INFO("Executing %f %f",trajectoryPos[0],trajectoryPos[1]);
        bool usePosition = trajectory_action_server->usePositionMode();
        data_lock.lock();
        if (usePosition)
        {
            // if (fabs(trajectoryPos[i])>0.01) ROS_INFO_STREAM("GazeboJointTrajectoryServer: Setting position target "<<scopedName<<": "<<trajectoryPos[i]);
            if (!jointController->SetPositionTarget(scopedName, MathFunctions::capToPI(trajectoryPos[i])))
            {
                ROS_ERROR_STREAM_ONCE("Could not set position target for "<<joint->GetName()<<
                       ". Can't control joint. This message is only printed once.");
            }
        }
        else
        {
            // if (fabs(trajectoryVel[i])>0.2) ROS_INFO_STREAM("GazeboJointTrajectoryServer: Setting velocity target "<<scopedName<<": "<<trajectoryVel[i]);
            if (!jointController->SetVelocityTarget(scopedName, trajectoryVel[i]))
            {
                ROS_ERROR_STREAM_ONCE("Could not set velocity target for "<<joint->GetName()<<
                       ". Can't control joint. This message is only printed once.");
            }
            // with 0 velocities, also have to maintain the correct arm pose:
            if (fabs(trajectoryVel[i]<1e-04))
            {
                // ROS_INFO_STREAM("GazeboJointTrajectoryServer: Setting position target "<<scopedName<<": "<<currentAngles[i]);
                
                // if the velocity is 0, we can assume that we either are at the next trajectory point already,
                // or this joint didn't move in between trajectory points. Therefore, we can use the next trajectory
                // pose as joint position value. 
                // NOTE: If we use currentAngles[i] here, the arm will slowly collapse. This is because small fluctuations in
                // the current angle values. This is random as the current angles can very well be equivalent
                // to the target position, but more often than not, they are a tiny bit off, caused by the gravity pulling down
                // the arm just a tiny bit. Then setting these wrong new targets will amplify over multiple iterations.
                // jointController->SetPositionTarget(scopedName,MathFunctions::capToPI(currentAngles[i]));
                if (!jointController->SetPositionTarget(scopedName,MathFunctions::capToPI(trajectoryPos[i])))
                {
                    ROS_ERROR_STREAM_ONCE("Could not set position target for "<<joint->GetName()<<
                           ". Can't control joint. This message is only printed once.");
                }
            }
        }
        data_lock.unlock();
        ++i;
    }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboJointTrajectoryServer::readJointStates(std::vector<float>& currAngles, std::vector<float>& currVels)
{
    const arm_components_name_manager::ArmComponentsNameManager& joints = trajectory_action_server->getArmNamesMgr();
    currAngles.resize(joints.numTotalJoints(), 0);
    currVels.resize(joints.numTotalJoints(), 0);

    gazebo::physics::Joint_V::const_iterator it;
    for (it = model->GetJoints().begin(); it != model->GetJoints().end(); ++it)
    {
        physics::JointPtr joint = *it;
        std::string jointName = joint->GetName();
        unsigned int dof = GetDOF(joint);

        // ROS_INFO("Getting %s",jointName.c_str());
        int armJointNumber = trajectory_action_server->getArmNamesMgr().armJointNumber(jointName);
        int gripperJointNumber = trajectory_action_server->getArmNamesMgr().gripperJointNumber(jointName);

        bool robotJoint = (armJointNumber >= 0) || (gripperJointNumber >= 0);
        if (!robotJoint) continue;
       
        if (dof == 0)
        {
          // skip fixed joints
          continue;
        }

        unsigned int axis = 0;
        if (dof > 1)
        {
            ROS_WARN_STREAM("GazeboJointTrajectoryServer: Only support 1 axis, "
              << "got " << dof << ". Wil only read first value for joint " << jointName);
        }

        double currAngle = GetPosition(joint, axis);
        currAngle = MathFunctions::capToPI(currAngle);

        // double currEff=joint->GetForce(axis);
        double currVel = joint->GetVelocity(axis);

#ifdef USE_VELOCITY_TRACKER
        double currGzbVel = currVel;
        if (velTracker.getJointVelocity(jointName, currVel) < 0)
        {
            ROS_ERROR_STREAM("Error obtaining joint velocity for joint "<<jointName);
            // continue;
        }
        // if (fabs(currGzbVel)>0.1) ROS_INFO_STREAM("Velocity compare "<<jointName<<": g="<<currGzbVel<<", t="<<currVel);
#endif
        
        // if (fabs(currVel>0.1)) ROS_INFO_STREAM("Velocity "<<jointName<<": "<<currVel);

        if (gripperJointNumber >= 0)
        {
            // Save gripper angles in radians
            currAngles[joints.numArmJoints() + gripperJointNumber] = currAngle;
            currVels[joints.numArmJoints() + gripperJointNumber] = currVel;
        }
        else if (armJointNumber >= 0)
        {
            // Save arm angles in radians
            currAngles[armJointNumber] = currAngle;
            currVels[armJointNumber] = currVel;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboJointTrajectoryServer::isGripper(const physics::JointPtr& joint) const
{
    return trajectory_action_server->getArmNamesMgr().isGripper(joint->GetName()) 
        || trajectory_action_server->getArmNamesMgr().isGripper(joint->GetScopedName());
}

////////////////////////////////////////////////////////////////////////////////
void GazeboJointTrajectoryServer::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}

}  // namespace gazebo
