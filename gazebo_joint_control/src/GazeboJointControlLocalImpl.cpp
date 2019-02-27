#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Copyright (C) 2015 Jennifer Buehler
   See also LICENSE file in this repository.
*/
#endif


#include <gazebo_joint_control/GazeboJointControlLocalImpl.h>
#include <gazebo_joint_control/GazeboVersionHelpers.h>
#include <convenience_math_functions/MathFunctions.h>
#include <map>
#include <string>

#define DISABLE_GRAVITY false 

#define INIT_WITH_POS false

// Default PID gains if SetVelocity() returns true
#define KP_POS_SETVEL 15
#define KI_POS_SETVEL 2
#define KD_POS_SETVEL 0.1

#define KP_VEL_SETVEL 0.1
#define KI_VEL_SETVEL 0.01
#define KD_VEL_SETVEL 0

// Default PID gains if SetVelocity() returns false 
#define KP_POS_SETF 100
#define KI_POS_SETF 0.01
#define KD_POS_SETF 5

#define KP_VEL_SETF 80
#define KI_VEL_SETF 0
#define KD_VEL_SETF 0.001

// do additional fallback for ensuring safe use:
// If velocity 0 is specified as target, but *no*
// alternative position value has been specified,
// calculate that position value. This will slow
// things down a little (even if position *was* specified)
// but it should make things
// work still if the user does not set the target
// commands consistenly. Warnings will be printed
// if this fix has been applied.
#define _FIX_POSITION_FALLBACK_                            

// the major version of Gazebo when call to Joint::SetForce()
// does *not* need to be repeated artificially in order for
// it to work...
#define GAZEBO_IMPROVED_SETFORCE 3

// for older Gazebo versions, it seems that successive calls to SetForce()
// are required (because it's accumulative), otherwise not enough force
// gets applied. SetForce() will always be called once, and the additional number
// specified here.
#if GAZEBO_MAJOR_VERSION < GAZEBO_IMPROVED_SETFORCE
#define GAZEBO_SETFORCE_REPEAT 1
#else
#define GAZEBO_SETFORCE_REPEAT 0
#endif

using convenience_math_functions::MathFunctions;

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
GazeboJointControlLocalImpl::GazeboJointControlLocalImpl():
    GazeboJointControl()
{
#if GAZEBO_MAJOR_VERSION > 2
    gazebo::common::Console::SetQuiet(false);
#endif
    ROS_INFO("Creating GazeboJointControlLocalImpl plugin");
    ros::NodeHandle n("");
    n.param<bool>("gazebo_use_set_velocity",useSetVelocity,true);
    ROS_INFO_STREAM("Gazebo using SetVelocity(): "<<useSetVelocity);
}

////////////////////////////////////////////////////////////////////////////////
GazeboJointControlLocalImpl::~GazeboJointControlLocalImpl()
{
}


////////////////////////////////////////////////////////////////////////////////
bool GazeboJointControlLocalImpl::DisableGravity() const
{
    return DISABLE_GRAVITY;
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboJointControlLocalImpl::UseForce() const
{
    return !SetVelocity();
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboJointControlLocalImpl::SetVelocity() const
{
    return useSetVelocity;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboJointControlLocalImpl::GetDefaultPosGains(float& kp, float& ki, float& kd) const
{
    if (SetVelocity())
    {
        kp = KP_POS_SETVEL;
        ki = KI_POS_SETVEL;
        kd = KD_POS_SETVEL;
    } else {
        kp = KP_POS_SETF;
        ki = KI_POS_SETF;
        kd = KD_POS_SETF;
    }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboJointControlLocalImpl::GetDefaultVelGains(float& kp, float& ki, float& kd) const
{
    if (SetVelocity())
    {
        kp = KP_VEL_SETVEL;
        ki = KI_VEL_SETVEL;
        kd = KD_VEL_SETVEL;
    } else {
        kp = KP_VEL_SETF;
        ki = KI_VEL_SETF;
        kd = KD_VEL_SETF;
    }
}

////////////////////////////////////////////////////////////////////////////////
double GazeboJointControlLocalImpl::DistToPosition(const physics::JointPtr& joint,
        double targetPosition) const
{
    const int axis = 0;
    double currPosition = GetPosition(joint, axis);

    double lowLimit=GetLowerLimit(joint, axis);
    double highLimit=GetUpperLimit(joint, axis);
//    ROS_INFO_STREAM("Limits: "<<lowLimit<<", "<<highLimit);

    double _targetPosition=MathFunctions::limitsToTwoPI(targetPosition, lowLimit, highLimit);
    // ... currPosition (read from Gazebo) is always in the right format already.

    double dist=currPosition - _targetPosition;

    if ((fabs(highLimit) > 2*M_PI) || (fabs(lowLimit) > 2*M_PI)) {
        // the joint has no limits, so it can turn either direction. 
        // Use the shortest angle distance as goal.
        // TODO: current limitation is the assumption that if there is
        // either a high or a low limit, the arm has both limits.
        dist=MathFunctions::angleDistance(_targetPosition,currPosition);
    }
    return dist;
}

/*
#define BREAK_LOOKAHEAD 0.03
double GazeboJointControlLocalImpl::AdjustForCloseTarget(const physics::JointPtr& joint,
        const double distToTarget, const gazebo::common::Time& stepTime, const double lookaheadSecs, const double commandValue) const
{
    const int axis=0;
    double currVel = joint->GetVelocity(axis); 
    double nextStepSize = currVel * lookaheadSecs;
    double retVal = commandValue;
    if (((distToTarget < 0) && (nextStepSize < distToTarget)) ||
        ((distToTarget > 0) && (nextStepSize > distToTarget)))
    {
        // at this velocity, in <lookaheadSecs> would arrive at target exactly.
        double targetVel = distToTarget / lookaheadSecs;
        // how much smaller proportionally the target velocity is to the current velocity
        double factor = targetVel / currVel;
        if (factor < 0) {
            ROS_ERROR_STREAM("Factor should always be positive! "<<factor);
            throw std::string("Factor should always be positive!");
        }

        if (factor > 1.01) {
            ROS_INFO_STREAM("Overshoot "<<joint->GetName()<<" for goal "<<distToTarget
                <<", at currVel="<<currVel<<" and nextSTepSize "<<nextStepSize
                <<": target "<<targetVel<<", adjust "<<commandValue<<" by "<<factor);
            ROS_ERROR("DEBUG ME: Factor should never be higher");
            throw std::string("DEBUG ME: Factor should never be higher");
        } 
        retVal = commandValue * factor;
    }
    return retVal;
}
*/

////////////////////////////////////////////////////////////////////////////////
double GazeboJointControlLocalImpl::UpdateVelocityToPosition(const physics::JointPtr& joint,
        double targetPosition, common::PID& pid, const gazebo::common::Time& stepTime) const
{

    double dist = DistToPosition(joint,targetPosition);
    double cmd = pid.Update(dist, stepTime);
    // double targetVel = capTargetVel(joint, cmd, true, targetAngle,1e-04);
    return capTargetVel(joint, cmd, false);
}

////////////////////////////////////////////////////////////////////////////////
double GazeboJointControlLocalImpl::UpdateForceToPosition(const physics::JointPtr& joint,
        double targetPosition, common::PID& pid, const gazebo::common::Time& stepTime) const
{
    double dist = DistToPosition(joint,targetPosition);
    double cmd = pid.Update(dist, stepTime);
    // ROS_INFO_STREAM("PID val "<<joint->GetName()<<": "<<cmd);
    // return cmd;
    // cmd = AdjustForCloseTarget(joint, dist, stepTime, BREAK_LOOKAHEAD, cmd);
    return capTargetForce(joint, cmd, false);
}

    
////////////////////////////////////////////////////////////////////////////////
// removes from \e anyMap all joint names (map key) which are not maintained by \e joints
template<class Any>
void FilterMaintanedJointsMap(std::map<std::string, Any>& anyMap, const std::vector<std::string>& joint_names, const physics::ModelPtr& model)
{  
    // this implementation just builds a new map
    std::map<std::string, Any> newMap;
    for (std::vector<std::string>::const_iterator it=joint_names.begin(); it!=joint_names.end(); ++it)
    {
        // need to get the joint to retrieve the scoped name
        physics::JointPtr joint = model->GetJoint(*it);
        if (!joint)
        {
            ROS_ERROR_STREAM("GazeboJointControlLocalImpl: FilterMaintanedJointsMap can't find joint "<<*it);
            continue;
        }
        typename std::map<std::string, Any>::iterator mEntry = anyMap.find(joint->GetScopedName());
        if (mEntry!=anyMap.end())
        {
            newMap.insert(std::make_pair(mEntry->first, mEntry->second));
        }
    }
    anyMap = newMap;
    /*
    // this implementation removes entries from the map which can't be found in joint_names
    for (typename std::map<std::string, Any>::iterator it=anyMap.begin(); it!=anyMap.end(); ++it)
    {
        if (std::find(joint_names.begin(),joint_names.end(), it->first) == joint_names.end())
        {
        }
    }*/
} 


////////////////////////////////////////////////////////////////////////////////
bool GazeboJointControlLocalImpl::UpdateJoints()
{
    // XXX TODO remove this test:
    static boost::mutex updateMtx;
    if (!updateMtx.try_lock()) {
        ROS_WARN("Other thread is just updating the joints, your GazeboJointControlLocalImpl::UpdateJoints() may be too slow");
        return false;
    }
    boost::adopt_lock_t adl;
    boost::lock_guard<boost::mutex> updLck(updateMtx, adl);
    // XXX end test

    if (!model.get())
    {
        ROS_ERROR("Cannot update GazeboJointControlLocalImpl if no model is set");
        return false;
    }

    if (!jointController.get())
    {
        ROS_ERROR("Cannot load GazeboJointControlLocalImpl if no default JointController is set for the model");
        return false;
    }

    //ROS_INFO("Start");
    common::Time currTime = GetSimTime(model->GetWorld());
    common::Time stepTime = currTime - prevUpdateTime;

    if (stepTime <= 0)
    {
        prevUpdateTime = currTime; 
        return true;
    }
//    ROS_INFO_STREAM("Step time: "<<stepTime.Double());

    boost::unique_lock<boost::recursive_mutex> lck = jointController->GetLock();

    std::map<std::string, double> forces = jointController->GetForces();
    std::map<std::string, double> positions = jointController->GetPositions();
    std::map<std::string, double> velocities = jointController->GetVelocities();

    std::map<std::string, physics::JointPtr > jntMap = jointController->GetJoints();
    std::vector<std::string> joint_names; // all joint names maintained by the joint manager. This should be a global field at some point.
    joints->getJointNames(joint_names, true); 
        
    FilterMaintanedJointsMap(forces, joint_names, model);
    FilterMaintanedJointsMap(positions, joint_names, model);
    FilterMaintanedJointsMap(velocities, joint_names, model);

    const int axis = 0;

    // It is necessary to first collect the final target values, including the overwriting of
    // lower-priority values, and then set the joint velocities in one batch with
    // joint::SetVelocity().This is necessary because when calling joint::SetVelocity()
    // first and then a little bit after again to do the overwriting, the arm wiggles.
    // Therefore a map is pre-computed with all Joints to be updated, to run the
    // actual updating at the end.

    // The map contains hoints (with the key name Joint::GetName()) to be updated
    // along with the velocity values.
    // All joints which are not to be updated here will remain at their current pose.
    std::map<std::string, double> finalJointUpdates;

    if (!positions.empty())
    {
        std::map<std::string, common::PID> posPIDs = jointController->GetPositionPIDs();
        std::map<std::string, double>::iterator it;
        for (it = positions.begin(); it != positions.end(); ++it)
        {
            std::map<std::string, double>::iterator velIt = velocities.find(it->first);
            static float eps = 1e-05;
            std::map<std::string, physics::JointPtr >::iterator jntIt;
            jntIt = jntMap.find(it->first);
            if (jntIt == jntMap.end())
            {
                ROS_ERROR_STREAM("Could not find joint " << it->first << " in joint controllers map");
                return false;
            }
            physics::JointPtr joint = jntIt->second;

            std::map<std::string, common::PID>::iterator pidIt=posPIDs.find(it->first);
            if (pidIt==posPIDs.end())
            {
                ROS_ERROR_STREAM_ONCE("No position PID controller found for "<<it->first<<
                       ". Can't control joint. This message is only printed once.");
                return false;
            }
            common::PID& pid = pidIt->second;
            // ROS_INFO_STREAM("Updating position PID for "<<it->first);

            double targetVal=it->second;
            if (SetVelocity()) {
                targetVal = UpdateVelocityToPosition(joint, it->second, pid, stepTime);
           } else {
                targetVal = UpdateForceToPosition(joint, it->second, pid, stepTime);
            }
            
            finalJointUpdates.insert(std::make_pair(joint->GetName(), targetVal));

            // Because unfortunately JointController does not give us back a reference,
            // we have to manually update the PID again
            jointController->SetPositionPID(it->first, pid);
        }
    }

    if (!velocities.empty())
    {
        if (!velocityControllersLoaded())
        {
            ROS_ERROR_STREAM_ONCE("No velocity controllers loaded. "<<
                   ". Can't control joints. This message is only printed once.");
            return false;
        }
        std::map<std::string, common::PID> velPIDs = jointController->GetVelocityPIDs();
        std::map<std::string, double>::iterator it;
        for (it = velocities.begin(); it != velocities.end(); ++it)
        {
            std::map<std::string, physics::JointPtr >::iterator jntIt;
            jntIt = jntMap.find(it->first);
            if (jntIt == jntMap.end())
            {
                ROS_ERROR_STREAM("Could not find joint " << it->first << " in joint controllers map");
                return false;
            }
            physics::JointPtr joint = jntIt->second;

            double targetVel = it->second;
            targetVel = capTargetVel(joint, targetVel, false);

            double currVel = joint->GetVelocity(axis);


            std::map<std::string, common::PID>::iterator pidIt=velPIDs.find(it->first);
            if (pidIt==velPIDs.end())
            {
                ROS_ERROR_STREAM_ONCE("No velocity PID controller found for "<<it->first<<
                       ". Can't control joint. This message is only printed once.");
                return false;
            }
            
            // ROS_INFO_STREAM("Updating velocity PID for "<<it->first<<" (curr vel "<<currVel<<")");
            
            common::PID& pid = pidIt->second;

            double cmd = pid.Update(currVel - targetVel, stepTime);
            double finalTargetVal = targetVel;
            // true when we want to insert the final value into the map
            // so that the joint is updated. If false, already existing
            // values (e.g. from positions) will be kept in the map instead,
            // and not overwritten.
            bool insertToMap = true;
            if (DisableGravity())
            {
                if (!SetVelocity()) {
                    ROS_ERROR_STREAM_ONCE("GazeboJointControlLocalImpl does not support disabled "
                        <<"gravity and use of Joint::SetForce() yet. This message is printed only once.");
                    return false;
                }

                // Because there is no gravity anyway, set velocity directly.
                finalTargetVal = targetVel;
                if (fabs(targetVel) > 1e-04) 
                    ROS_INFO_STREAM("DisableGrav: Velocity for "<<joint->GetName()<<": curr="<<currVel<<", target="<<targetVel<<" in orig map "<<it->second);
            }
            else
            {
                if (SetVelocity())
                {
                    double pidTargetVel = targetVel + cmd;
                    pidTargetVel = capTargetVel(joint, pidTargetVel, false);
                    finalTargetVal = pidTargetVel;
                } else {
                    finalTargetVal = cmd;
                }
                static const float eps = 1e-04;
                if (fabs(targetVel) <= eps)
                // if velocity is 0, rely on the position command instead
                {
                    // Check whether a replacement position was actually specified as well, and print an error if not.
                    std::map<std::string, double>::iterator posIt = positions.find(it->first);
                    bool hasPositionUpdate = (posIt != positions.end());
#ifdef _FIX_POSITION_FALLBACK_                            
                    if (!hasPositionUpdate)
                    {
                        ROS_WARN_STREAM_ONCE("Velocity specified as 0 (" << targetVel << ") for joint " << it->first
                                             << ". If you specify a zero velocity, you should also specify a"
                                             << " position target at which to keep the arm. "
                                             << " Will now use the joint's current position to keep."
                                             << " This should work, but it is inefficient. "
                                             << " This message is printed only once.");
                        std::map<std::string, common::PID> posPIDs = jointController->GetPositionPIDs();
                        common::PID posPID = posPIDs[it->first];
                        double currPosition = GetPosition(joint, axis);  // if capped to PI, it won't work for joint limits
                        
                        if (SetVelocity()) finalTargetVal = UpdateVelocityToPosition(joint, currPosition, posPID, stepTime);
                        else finalTargetVal = UpdateForceToPosition(joint, currPosition, posPID, stepTime);
                        
                        jointController->SetPositionPID(it->first, posPID);
                    }
#endif  //_FIX_POSITION_FALLBACK_                   
                    if (hasPositionUpdate) 
                    {
                        /*std::map<std::string, double>::iterator fju = finalJointUpdates.find(joint->GetName());
                        if (fju==finalJointUpdates.end()) {
                            ROS_FATAL("Target position velocity value should have been specified");
                            return false;
                        }
                        ROS_INFO_STREAM("Not overwriting "<<joint->GetName()<<" velocity "<<fju->second);*/
                        insertToMap = false;
                    }
                }
                else  
                {
                    // double p,i,d;
                    // pid.GetErrors(p,i,d);
                    // ROS_INFO_STREAM("Velocity for "<<joint->GetName()<<": curr="<<currVel<<", target="<<targetVel<<". Apply cmd="<<finalTargetVal); //<<", i="<<i);
                }
            }

            if (insertToMap)
            {
                std::map<std::string, double>::iterator fju =
                        finalJointUpdates.insert(std::make_pair(joint->GetName(), finalTargetVal)).first;
                if (fju == finalJointUpdates.end())
                {
                    ROS_WARN_STREAM("Joint " << joint->GetName() << " was not in position map");
                }
                fju->second = finalTargetVal;
            }

            // Because unfortunately JointController does not give us back a reference,
            // we have to manually update the PID again
            jointController->SetVelocityPID(it->first, pid);
        }
    }

    if (!forces.empty())
    {
        ROS_ERROR_ONCE("Setting forces is not implemented yet. Ignoring force settings. This message will only be printed once.");
    }

    std::map<std::string, double>::iterator fju;
    for (fju = finalJointUpdates.begin(); fju != finalJointUpdates.end(); ++fju)
    {
        physics::JointPtr joint = model->GetJoint(fju->first);
        if (!joint.get())
        {
            ROS_ERROR_STREAM("Could not find joint " << fju->first << " in model joints");
            return false;
        }
        // possible HACK: If velocity is 0, we could here fix the current position with SetJointPosition(), and NOT
        // do a  velocity 0 update. This can prevent the arm from slightly wiggling around 0 velocity positions.
        // ROS_INFO_STREAM("Setting velocity "<<joint->GetName()<<": "<<fju->second<<" (measured: "<<joint->GetVelocity(0)<<")");

        if (SetVelocity())
        {
            // double currV=joint->GetVelocity(axis);
            // ROS_INFO_STREAM("Setting velocity of "<<joint->GetName()<<": "<<fju->second<<" - curr "<<currV<<", stepTime: "<<stepTime.Double());
#if GAZEBO_MAJOR_VERSION > 2
            // XXX TODO setting the velocity like this generates gazebo errors, but in the end it was required
            // to make it work anyway? Switching off the error message here for now.
            gazebo::common::Console::SetQuiet(true);
            joint->SetParam("vel", axis, fju->second);
            gazebo::common::Console::SetQuiet(false);
            // ROS_INFO_STREAM("Setting velocity "<<joint->GetName()<<": "<<fju->second<<" (measured: "<<joint->GetVelocity(0)<<"), "<<joint->GetParam("vel", axis));
            // for some reason, SetVelocity() still has to be called. But if called
            // without the previous SetParam(), it doesn't work as effectively, set
            // velocities are mostly not met if moving against direction of gravity.
            joint->SetVelocity(axis, fju->second);
#else
            joint->SetVelocity(axis, fju->second);
#endif
        } else {
            //double currF=joint->GetForce(axis);
            //ROS_INFO_STREAM("Setting force of "<<joint->GetName()<<": "<<fju->second<<" - curr "<<currF<<", stepTime: "<<stepTime.Double());
            joint->SetForce(axis, fju->second);
            // sometimes, successive calls to SetForce
            // are required (because it's accumulative),
            // otherwise not enough force gets applied.
            for (int i=0; i < GAZEBO_SETFORCE_REPEAT; ++i) 
                joint->SetForce(axis, fju->second);
        }
    }
    prevUpdateTime = GetSimTime(model->GetWorld());
   // ROS_INFO_STREAM("end: "<<stepTime.Double());
    return true;
}

}  // namespace gazebo
