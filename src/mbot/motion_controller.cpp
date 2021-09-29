#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include "maneuver_controller.h"

#define PI 3.1415

/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers. 
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////   



class StraightManeuverController : public ManeuverControllerBase
{
public:
    
    StraightManeuverController() = default;   
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        
        this->t_now = utime_now();
        
        

        this->prevDev = this->dev;
        this->dev = sqrt(pow(target.x - pose.x, 2) + pow(target.y - pose.y, 2));
          

        v = Kp * (this->dev); //+Ki*(this->Dt)*this->dev + Kd * (this->xDeviation - this->prevDev) / (this->Dt);
                
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float angleDeviation = angle_diff(pose.theta, target_heading);
        
        float w = Komega * angleDeviation;
        
        this->Dt = (t_now - t_prev);
        this->t_prev = this->t_now;
        this->t_next = this->t_now + 1000;
        
        if (v > 1) v = 1;
        if (w > 6) w = 6;
        if (v < 0.07) v = 0.07;
       
        
        return {0, v, w};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        return ((fabs(pose.x - target.x) < 0.05) && (fabs(pose.y - target.y)  < 0.05));
    }

private:
    float Kp = 1.5;
    //float Ki = 0.0000002;
    //float Kd = 40000;
    float Komega = 1;
    uint64_t t_prev = 0.0;
    uint64_t t_now = 0.0;
    float dev = 0.0;
    float prevDev = 0.0;
    float Dt = 0.0;
    float v = 0.0;
    float w = 0.0;
    uint64_t t_next = 0.0;

};


class TurnManeuverController : public ManeuverControllerBase
{
public:
    TurnManeuverController() = default;   
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float wError = angle_diff(target_heading, pose.theta);

        float w = Kp*wError;// PI / 4;
        /*if (wError < 0) {
            w = -2.5;//-PI/4;
        }*/
        return { 0, 0, w };
       
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        return (fabs(angle_diff(pose.theta, target_heading)) < 0.03);
    }
private: 
    float Kp = 0.5;
};

class OrientManeuverController : public ManeuverControllerBase
{
public:
    OrientManeuverController() = default;
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        
        float wError = angle_diff(target.theta, pose.theta);
        float w = PI/4;
        if (wError < 0) {
            w = -PI/4;
        }
        return { 0, 0, w };

    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        return (fabs(angle_diff(pose.theta, target.theta)) < 0.03);
    }
private:
    float Kp = 30;
};


class MotionController
{ 
public: 
    
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
    :
        lcmInstance(instance),
        odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

	    time_offset = 0;
	    timesync_initialized_ = false;
    } 
    
    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void) 
    {
        mbot_motor_command_t cmd {now(), 0.0, 0.0};
        
        if(!targets_.empty() && !odomTrace_.empty()) 
        {
            pose_xyt_t target = targets_.back();
            pose_xyt_t pose = currentPose();

            ///////  TODO: Add different states when adding maneuver controls /////// 
            if(state_ == TURN)
            {
                std::cout << "\rTURNING";
                
                if(turn_controller.target_reached(pose, target))
                {
                    state_ = DRIVE;
                    std::cout << "\n";
                } 
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                    
                }
            }
            else if(state_ == DRIVE) 
            {
                std::cout << "\rDRIVING";
                if(straight_controller.target_reached(pose, target))
                {
                    if (!assignNextTarget())
                    {
                        std::cout << "\nTarget Reached!\n";
                    }
                    std::cout << "\n";
                }
                else
                { 
                    cmd = straight_controller.get_command(pose, target);
                }
		    }
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
		} 
        return cmd; 
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync)
    {
	    timesync_initialized_ = true;
	    time_offset = timesync->utime-utime_now();
    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->utime << "\n"; 
    	for(auto pose : targets_)
        {
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}
        std::cout << std::endl;

        assignNextTarget();

        //confirm that the path was received
        message_received_t confirm {now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        pose_xyt_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        computeOdometryOffset(*pose);
    }
    
private:
    
    enum State
    {
        TURN,
        DRIVE,
        ORIENT
    };
    
    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;
 
    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;
    OrientManeuverController orient_controller;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }
    
    bool assignNextTarget(void)
    {
        if(!targets_.empty()) { 
            targets_.pop_back(); 
            std::cout << "\nCurrent target: " << "(" << targets_.back().x << "," << targets_.back().y << "," << targets_.back().theta << ")\n";
        }
        state_ = TURN;
        return !targets_.empty();
    }

    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
         
        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated; 
        odomToGlobalFrame_.theta = deltaTheta;
    }
    
    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
       
        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    signal(SIGINT, exit);
    std::cout << "\n";
    
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            	mbot_motor_command_t cmd = controller.updateCommand();
                lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
                //std::cout << "\rv: " << cmd.trans_v << "    w: " << cmd.angular_v;
    	}
    }
    
    return 0;
}
