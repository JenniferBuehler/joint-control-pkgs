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


#include <gazebo_joint_control/GazeboJointStateClient.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include <map>
#include <string>
#include <algorithm>
#include <vector>

#define DEFAULT_JOINT_STATE_TOPIC "/joint_control"
using arm_components_name_manager::ArmComponentsNameManager;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboJointStateClient);

GazeboJointStateClient::GazeboJointStateClient():
    jointStateTopic(DEFAULT_JOINT_STATE_TOPIC)
{
    ROS_INFO("Creating GazeboJointStateClient plugin");
    nh.param("joint_state_control_topic", jointStateTopic, jointStateTopic);
    ROS_INFO_STREAM("GazeboJointStateClient subscription topic: " << jointStateTopic);
}

GazeboJointStateClient::~GazeboJointStateClient()
{
}

void GazeboJointStateClient::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                         << "in the gazebo_ros package)");
        return;
    }

    // get joint names from parameters
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

    // ROS_INFO_STREAM("GazeboJointStateClient loading joints from components namespace '"<<armNamespace<<"'");

    ROS_INFO_STREAM("GazeboJointStateClient: Loading arm component parameters from "<< armNamespace);
    joints = ArmComponentsNameManagerPtr(new ArmComponentsNameManager(armNamespace,false));
    if (!joints->waitToLoadParameters(1, 3, 0.5))
    {
        ROS_FATAL_STREAM("Cannot load arm components for robot "<<_parent->GetName()<<" from namespace "<<armNamespace);
        return;
    }

    physics::BasePtr jcChild = _parent->GetChild(physics::JointControllerThreadsafe::UniqueName());
    if (!jcChild.get())
    {
        ROS_ERROR("Cannot load GazeboJointStateClient if no default JointControllerThreadsafe is set for the model");
        throw std::string("Cannot load GazeboJointStateClient if no default JointController is set for the model");
    }
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

    // get joint names from parameters
    std::vector<std::string> joint_names;
    joints->getJointNames(joint_names, true);
    const std::vector<float>& arm_init = joints->getArmJointsInitPose();
    const std::vector<float>& gripper_init = joints->getGripperJointsInitPose();

    // Print the joint names to help debugging
    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();
    for (std::map<std::string, physics::JointPtr>::iterator it = jntMap.begin(); it != jntMap.end(); ++it)
    {
        physics::JointPtr j = it->second;
        ROS_INFO_STREAM("Gazebo joint: '"<<j->GetName()<<"' is registered as '"<<it->first<<"'");
    }


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

        std::string scopedName = joint->GetScopedName();
        std::map<std::string, physics::JointPtr >::iterator jit = jntMap.find(scopedName);
        if (jit == jntMap.end())
        {
            ROS_ERROR_STREAM("Joint name " << *it << " not found in joint controller joints");
            throw std::string("Joint not found");
        }

        ++i;
    }

    model = _parent;

    jsSub = nh.subscribe(jointStateTopic, 1, &GazeboJointStateClient::JointStateCallback, this);
}


void GazeboJointStateClient::JointStateCallback(sensor_msgs::JointStateConstPtr _joints)
{
    // ROS_INFO_STREAM("Joint state callback: "<<*_joints);
    if (!jointController.get())
    {
        ROS_ERROR("Cannot load GazeboJointStateCallback if no default JointController is set for the model");
        return;
    }

    // number of joints we are going to process is the larger of velocity or position
    size_t jointCount = std::min(_joints->name.size(), std::max(_joints->velocity.size(), _joints->position.size()));
    if (jointCount == 0)
    {
        ROS_ERROR("GazeboJointStateClient: Must at least provide one position or velocity");
        return;
    }

    boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();

    std::map<std::string, double> forces = jointController->GetForces();
    std::map<std::string, double> positions = jointController->GetPositions();
    std::map<std::string, double> velocities = jointController->GetVelocities();
    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();

    // XXX old implementation:
    // clear out all possible current targets. Necessary e.g. to clear out
    // exitsing velocity targets, when only positions are set now...
    // XXX this is not good because if the joint state is a subset, all other
    // targets are cleared out and the arm collapses.
    // jointController->Reset();

    for (size_t i = 0; i < jointCount; ++i)
    {
        bool isGripper = false;
        std::string jointName = _joints->name[i];

        // ROS_INFO_STREAM("Processing joint '"<<jointName<<"'");
        physics::JointPtr joint = model->GetJoint(jointName);
        if (!joint.get())
        {
            ROS_ERROR_STREAM("Joint name " << jointName << " not found as robot joint");
            continue;
        }

        std::string scopedName = joint->GetScopedName();
        std::map<std::string, physics::JointPtr >::iterator jit = jntMap.find(scopedName);
        if (jit == jntMap.end())
        {
            // ROS_WARN_STREAM("Joint name "<<scopedName<<" not found in joint controller joints");
            continue;
        }

        if (i < _joints->position.size())
        {
            double currTargetPosVal = positions[scopedName];
            // set current target position of joint i, if it is not already achieved:
            double targetPosVal = _joints->position[i];
            static double eps = 1e-06;

            if (fabs(currTargetPosVal - targetPosVal) > eps)
            {
                // ROS_INFO_STREAM("Setting position target "<<scopedName<<": "<<target);
                jointController->SetPositionTarget(scopedName, targetPosVal);
            }
            else
            {
                // ROS_INFO_STREAM("Leaving position target "<<scopedName<<": "<<currTargetPosVal);
                jointController->SetPositionTarget(scopedName, currTargetPosVal);
            }
            // XXX new implementation:
            // set velocity to 0 for now, it will be overwritten if a velocity is specified for the
            // same joint.
            jointController->SetVelocityTarget(scopedName, 0);
        }
        if (i < _joints->velocity.size())
        {
            double targetVelVal = _joints->velocity[i];

            static double eps = 1e-03;
            bool zeroTargetVel = (fabs(targetVelVal) < eps);

            double targetPosVal = 0;
            if (i < _joints->position.size())
            {
                targetPosVal = _joints->position[i];
            }
            else if (zeroTargetVel)
            {
                ROS_ERROR_STREAM("If you specify 0 velocity for joint " << scopedName
                                 << ", you also have to specify a position for the joint to remain at");
                continue;
            }

            // ROS_INFO_STREAM("GazeboJointStateClient: Setting 'fallback' position of " << scopedName << " to " << targetPosVal);
            // set the position target to the current value to start with. The velocity
            // should overwrite this, but in case the velocity is set to 0, this position
            // should be held.
            jointController->SetPositionTarget(scopedName, targetPosVal);

            // set current target position of joint i, if it is not already achieved:
            double currTargetVelVal = velocities[scopedName];

            if (fabs(currTargetVelVal - targetVelVal) > eps)
            {
                // ROS_INFO_STREAM("GazeboJointStateClient: Setting velocity target "<<scopedName<<": "<<targetVelVal);
                jointController->SetVelocityTarget(scopedName, targetVelVal);
            }
            else
            {
                // ROS_INFO_STREAM("GazeboJointStateClient: Leaving velocity target "<<scopedName<<": "<<targetVelVal);
                jointController->SetVelocityTarget(scopedName, currTargetVelVal);
            }
        }
    }
}



bool GazeboJointStateClient::isGripper(const physics::JointPtr& joint) const
{
    return joints->isGripper(joint->GetName()) || joints->isGripper(joint->GetScopedName());
}



void GazeboJointStateClient::UpdateChild()
{
    ROS_INFO("UpdateChild()");
}

}  // namespace gazebo
