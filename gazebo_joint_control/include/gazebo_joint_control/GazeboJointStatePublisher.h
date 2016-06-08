#ifndef GAZEBO_JOINT_CONTROL_GAZEBOJOINTSTATEPUBLISHER_H
#define GAZEBO_JOINT_CONTROL_GAZEBOJOINTSTATEPUBLISHER_H

/**
   Gazebo plugin which publishes sensor_msgs/JointState messages with the current values in Gazebo.

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

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <arm_components_name_manager/ArmComponentsNameManager.h>
#include <gazebo_joint_control/JointController.h>

namespace gazebo
{


/**
 * \brief Gazebo plugin which publishes sensor_msgs/JointState messages with the current values in Gazebo.
 *
 * **ROS parameters**
 *
 * - Reads topic to publish to from ROS parameter "publish_joint_states_topic".
 * - From parameter "preserve_original_angles", reads a comma-separated list of joint names for
 *   which the original gazebo angles are to be published, instead of capping values to [-PI,PI].
 *   See also note below.
 * - This class also reads ROS parameters specifiying the arm components
 *    (see also arm_components_name_manager::ArmComponentsNameManager).
 *    The default namespace onto which such parameters have to be loaded correspond to the
 *    robot's name (as specified in the robot URDF/SDF). For example, if the robot's name is
 *    pr2, the parameters are read from ROS parameter namespace /pr2/...
 *    Alternatively, you can specify another namespace in the plugin's URDF/SDF
 *    tag ``<robot_components_namespace> your-namespace </robot_components_namespace>``.
 *
 *
 *  **Note**
 *  It may be necessary to publish values uncapped ( *not* in range [-PI,PI])
 *  when the lower limit is smaller than -PI.
 *  When using Gazebo, currently it is not possible to use a lower limit value greater than
 *  the higher limit in the URDF, e.g. [2.5..0.73].    
 *  While this generally does not present a problem, it does in conjunction with MoveIt!.
 *  MoveIt will detect an angle of eg. 3.0 as invalid if we have limits such as [-3.9..0.73].
 *  So we need to publish angles in this range as well. This has to be done for all *revolute* joints
 *  where lower limit value is greater than higher limit.
 *  Use ROS paramter ``preserve_original_angles`` to specify comma-separated list of joint names
 *  for which the original gazebo angles are to be published.
 *
 * \author Jennifer Buehler
 * \date January 2016
 */
class GazeboJointStatePublisher : public ModelPlugin
{
public:
    GazeboJointStatePublisher();

    virtual ~GazeboJointStatePublisher();

    /**
     *
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();

    bool isGripper(const physics::JointPtr& joint) const;

    physics::ModelPtr model;
    typedef boost::shared_ptr<arm_components_name_manager::ArmComponentsNameManager> ArmComponentsNameManagerPtr;
    ArmComponentsNameManagerPtr joints;

private:
    void WorldUpdate();
    void readJointStates(sensor_msgs::JointState& js);

    // This is a set of joint names for which the original output of gazebo
    // should be published *instead* capping the value to [-PI,PI].
    std::set<std::string> preserveAngles;

    event::ConnectionPtr update_connection;

    std::string jointStateTopic;
    ros::Publisher jsPub;
    ros::NodeHandle nh;

    bool publishAllJoints;
};
}  // namespace gazebo

#endif  // GAZEBO_JOINT_CONTROL_GAZEBOJOINTSTATEPUBLISHER_H
