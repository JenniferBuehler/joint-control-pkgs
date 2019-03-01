#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Copyright (C) 2019 Jennifer Buehler
   See also LICENSE file in this repository.
*/
#endif

#include <gazebo_joint_control/GazeboJointStatePublisher.h>
#include <gazebo_joint_control/GazeboVersionHelpers.h>
#include <convenience_math_functions/MathFunctions.h>

#include <ros/ros.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>

#include<boost/tokenizer.hpp>

#include <string>
#include <vector>

#define DEFAULT_JOINT_STATE_TOPIC "/joints_in_gazebo"


// set to true if all joints are to be published.
// If set to false, only the arm joints are published, not including gripper joints.
#define PUBLISH_ALL_JOINT_STATES true

using convenience_math_functions::MathFunctions;
using arm_components_name_manager::ArmComponentsNameManager;

///////////////////////////////////////////////////////////////////////////////
void separateTokens(const std::string& str, std::set<std::string>& result)
{
  if (str.empty()) return;
  boost::char_separator<char> sep(" ,;");
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  tokenizer tok(str, sep);
  
  int i = 0;
  for(tokenizer::iterator beg=tok.begin(); beg!=tok.end();++beg)
  {
    // ROS_WARN_STREAM("TEST TOKENIZE: '"<<beg->c_str()<<"'");
    result.insert(beg->c_str());
  }
}


namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboJointStatePublisher);

///////////////////////////////////////////////////////////////////////////////
GazeboJointStatePublisher::GazeboJointStatePublisher():
  joints(),  // use default constructor to read from parameters
  jointStateTopic(DEFAULT_JOINT_STATE_TOPIC),
  publishAllJoints(PUBLISH_ALL_JOINT_STATES)
{
  ROS_INFO("Creating GazeboJointStatePublisher plugin");
  nh.param("publish_joint_states_topic", jointStateTopic, jointStateTopic);
  ROS_INFO_STREAM("Joint state publish topic: " << jointStateTopic);
  std::string _preserveAngles;
  nh.param("preserve_original_angles", _preserveAngles, _preserveAngles);
  separateTokens(_preserveAngles, preserveAngles);

  if (!preserveAngles.empty())
    ROS_INFO("Joints to *not* cap to [-PI,PI] in GazeboJointStatePublisher:"); 
  std::set<std::string>::iterator it;
  for (it=preserveAngles.begin(); it!=preserveAngles.end(); ++it)
  {
    ROS_INFO_STREAM(*it);
  }
}

///////////////////////////////////////////////////////////////////////////////
GazeboJointStatePublisher::~GazeboJointStatePublisher()
{
}

///////////////////////////////////////////////////////////////////////////////
void GazeboJointStatePublisher::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
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
  
  ROS_INFO_STREAM("GazeboJointStatePublisher: Loading arm component parameters from "<< armNamespace);
  joints = ArmComponentsNameManagerPtr(new ArmComponentsNameManager(armNamespace,false));
  if (!joints->waitToLoadParameters(1, 3, 0.5))
  {
    ROS_FATAL_STREAM("Cannot load arm components for robot "<<_parent->GetName()<<" from namespace "<<armNamespace);
    return;
  }

  std::vector<std::string> joint_names;
  joints->getJointNames(joint_names, true);
  const std::vector<float>& arm_init = joints->getArmJointsInitPose();
  const std::vector<float>& gripper_init = joints->getGripperJointsInitPose();

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

    ++i;
  }

  model = _parent;
  jsPub = nh.advertise<sensor_msgs::JointState>(jointStateTopic, 1000);
  update_connection =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJointStatePublisher::WorldUpdate, this));
}

///////////////////////////////////////////////////////////////////////////////
void GazeboJointStatePublisher::WorldUpdate()
{
  sensor_msgs::JointState js;
  readJointStates(js);
  jsPub.publish(js);
}

///////////////////////////////////////////////////////////////////////////////
void GazeboJointStatePublisher::readJointStates(sensor_msgs::JointState& js)
{
  // Add timestamp to message (so robot_state_publisher does not ignore it)
  js.header.stamp = ros::Time::now();
  js.header.frame_id = "world";

  gazebo::physics::Joint_V::const_iterator it;
  for (it = model->GetJoints().begin(); it != model->GetJoints().end(); ++it)
  {
    physics::JointPtr joint = *it;
    std::string _jointName = joint->GetName();

    // ROS_INFO("Getting %s",_jointName.c_str());

    int armJointNumber = joints->armJointNumber(_jointName);
    int gripperJointNumber = joints->gripperJointNumber(_jointName);
    
    if (GetDOF(joint) == 0)
    {
      // a 0 degree joint is a fixed joint, we will skip it.
      continue;
    }

    if (GetDOF(joint) > 1)
    {
      ROS_WARN_STREAM_ONCE("GazeboJointStatePublisher: Only support 1 axis, "
      << "got " << GetDOF(joint)
      << ", will only write 1st angle for joint '" << _jointName << "'.");
    }

    unsigned int axis = 0;
    double currAngle = GetPosition(joint, axis);
    currAngle = MathFunctions::capToPI(currAngle);

    if (preserveAngles.find(_jointName) != preserveAngles.end())
    {
      // simply overwrite the "capToPi" correction from above
      currAngle = GetPosition(joint, axis);
    }

    double currEff = joint->GetForce(axis);
    double currVel = joint->GetVelocity(axis);

    // ROS_INFO("Joint %s (%u) %f %f %f", _jointName.c_str(), i, currAngle, currEff, currVel);

    bool isRobotJoint = (gripperJointNumber >= 0) || (armJointNumber >= 0);

    if (publishAllJoints || isRobotJoint)
    {
      js.name.push_back(_jointName);
      js.position.push_back(currAngle);
      js.velocity.push_back(currVel);
      js.effort.push_back(currEff);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboJointStatePublisher::isGripper(const physics::JointPtr& joint) const
{
  return joints->isGripper(joint->GetName()) || joints->isGripper(joint->GetScopedName());
}


///////////////////////////////////////////////////////////////////////////////
void GazeboJointStatePublisher::UpdateChild()
{
  ROS_INFO("UpdateChild()");
}

}  // namespace gazebo
