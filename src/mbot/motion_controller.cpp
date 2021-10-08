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
using namespace std;
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

#define DEBUG_STATE




#define DRIVE_CRITERIA 0.026 // m (2cm)
#define TURN_CRITERIA 0.04 //  radian 5 degree
#define TURN2_CRITERIA 0.04 // radian
// const float Kp = 1;
// const float Ktheta = 0.5;
// const float Kdrive_w = 0.5;
float Kp = 1.5;
float Ktheta = 0.5;
float Kdrive_w = 1;


// const float MAX_FWD_VEL = 0.8;
// const float MAX_TURN_VEL = 2.5;

// Default value
float MAX_FWD_VEL = 0.2;
float MAX_TURN_VEL = M_PI/4;


class StraightManeuverController : public ManeuverControllerBase
{
public:
    StraightManeuverController() = default;
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        return {0, 0.1, 0};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        return ((fabs(pose.x - target.x) < 0.1) && (fabs(pose.y - target.y)  < DRIVE_CRITERIA));
    }
    virtual bool target_reached2(const pose_xyt_t& pose, const float& target_theta)  override
    {
        return (fabs(angle_diff(pose.theta, target_theta)) < TURN2_CRITERIA); // radian (~15 degree)
    }

};

class TurnManeuverController : public ManeuverControllerBase
{
public:
    TurnManeuverController() = default;
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        return {0, 0, 0.5};
    }
    // For 'heading' alignment
    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        return (fabs(angle_diff(pose.theta, target_heading)) < TURN_CRITERIA); // radian
    }
    virtual bool target_reached2(const pose_xyt_t& pose, const float& target_theta)  override
    {
        return (fabs(angle_diff(pose.theta, target_theta)) < TURN2_CRITERIA); // radian (~15 degree)
    }


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
            pose_xyt_t target = targets_.back(); //
            pose_xyt_t pose = currentPose();

            // Print State, current (x,y,theta) and current goal
            #ifdef DEBUG_STATE
                // std::cout<<"state: "<<state_<<"(x,y,t): "<<pose.x<<","<<pose.y<<","<<pose.theta<<", goal: "<<target.x<<", "<<target.y<<", "<<target.theta<<std::endl;
                 std::cout<<"state: "<<state_<<"(x,y): "<<pose.x<<","<<pose.y<<", goal: "<<target.x<<", "<<target.y<<std::endl;
            #endif


            ///////  TODO: Add different states when adding maneuver controls ///////
            if(state_ == TURN)
            {
                if(turn_controller.target_reached(pose, target))
                {
                    printf("\rTURN: Reached target theta %.2f\n",target.theta);
		            state_ = DRIVE;
                }
                else
                {
                    // TODO: calculate cmd by error
                    float dx = target.x - pose.x;
                    float dy = target.y - pose.y;
                    float target_heading = atan2(dy, dx);
                    float angle_err = angle_diff(target_heading, pose.theta);
                    // printf("\rTURN: Goal: cur %.3f, goal %.3f Angle Error: %.4f\n",pose.theta, target_heading, angle_err);
                    cmd.trans_v = 0;
                    cmd.angular_v = Ktheta * angle_err;
                    // cmd.angular_v = Ktheta * angle_err;
                    // cmd = turn_controller.get_command(pose, target);
                }
            }
            else if(state_ == DRIVE)
            {
                if(straight_controller.target_reached(pose, target))
                {
                    cmd.trans_v = 0;
                    cmd.angular_v = 0;
                    printf("\rDRIVE: Reached target (x,y) %.2f, %.2f\n",target.x, target.y);
                    if(!assignNextTarget())
                    {
                        std::cout << "\rTarget Reached!\n";
                    }
                    // state_ = TURN2;
                }
                else
                {
                    float distance_err = sqrt( pow((target.x - pose.x),2) + pow((target.y-pose.y),2) );
                    // float x_err = fabs(pose.x - target.x);
                    // float y_err = fabs(pose.y - target.y);
                    // printf("\rDRIVE: cur x %.3f, goal x %.3f x_err %.4f y_err %4f\n",pose.x, target.x, x_err, y_err);
                    cmd.trans_v = Kp * distance_err ;

                    float dx = target.x - pose.x;
                    float dy = target.y - pose.y;
                    float target_heading = atan2(dy, dx);
                    float angle_err = angle_diff(target_heading, pose.theta);
                    cmd.angular_v = Kdrive_w * angle_err;

                    // cmd = straight_controller.get_command(pose, target);
                }
		    }
            else if(state_ == TURN2)  // pose.theta => target.theta
            {
                if(turn_controller2.target_reached2(pose, target.theta))
                {
                    if(!assignNextTarget())
                    {
                        std::cout << "\rEnd Path !!!\n";
                    }
                    // printf("\rTarget Reached!!!!!!!!! Reached target theta %.2f, now to TURN \n",target.theta);
		            state_ = TURN;
                }
                else
                {
                    float angle_err = angle_diff(target.theta, pose.theta);
                    // printf("\rTURN2: Goal: cur %.3f, goal %.3f Angle Error: %.4f\n",pose.theta, target.theta, angle_err);
                    cmd.trans_v = 0;
                    cmd.angular_v = Ktheta * angle_err;
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
    	std::cout << "received new path at time: " << path->utime << "\n";
    	for(auto pose : targets_)
        {
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}
        std::cout << std::endl;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()


        // assignNextTarget();
        state_ = TURN;

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
        TURN2
    };
    int index = 1; // target
    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;

    TurnManeuverController turn_controller;  // First heading controller
    StraightManeuverController straight_controller;
    TurnManeuverController turn_controller2; // Final Orientation controller

    int64_t now()
    {
	    return utime_now() + time_offset;
    }

    bool assignNextTarget(void)
    {
        if(!targets_.empty()) {
            cout<<"Target:"<<index++<<", "<<targets_.back().x<<", "<<targets_.back().y<<", "<<targets_.back().theta<<endl;
            targets_.pop_back();
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

    // if(argc < 2){
    //     printf("Wrong input format. ./motion_controller <speed_mode> (0: Fast, 1: Slow)\n");
    //     return 0;
    // }
    int speed_mode;
    cout<<"Enter speed mode: 0 => Slow, 1 => Fast: \n";
    cin>>speed_mode;
    string mode = (speed_mode==0)? "Slow" : "Fast";
    Ktheta = (speed_mode==0)?0.5 : 2.2;

    cout<<"Speed mode: "<< mode<<endl;
    MAX_FWD_VEL = (speed_mode==0)? 0.2 : 0.8;
    MAX_TURN_VEL = (speed_mode==0)? M_PI/4 : M_PI;

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            	mbot_motor_command_t cmd = controller.updateCommand();
                cmd.trans_v = std::max( std::min(MAX_FWD_VEL, cmd.trans_v), float(0));
                cmd.angular_v = std::min(MAX_TURN_VEL, cmd.angular_v);
                // printf("%f, %f\n",cmd.trans_v, cmd.angular_v);
            	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}
    }

    return 0;
}
