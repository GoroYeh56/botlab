#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>


// Goal : 1. (1, 0)
//        2. (0, 0)
 
#define SCALAR 3
int main(int argc, char** argv)
{std::cout << "here" <<std::endl;
    int numTimes = 1;
    // int numTimes = 1;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
   
    robot_path_t path;
    path.path.resize(2);

    pose_xyt_t nextPose;
    nextPose.x = 1.0f*SCALAR;
    nextPose.y = 0.0f*SCALAR;
    nextPose.theta = 0;
    path.path[0] = nextPose;
    
    nextPose.x = 0.0f*SCALAR;
    nextPose.y = 0.0f*SCALAR;
    nextPose.theta = -M_PI;
    path.path[1] = nextPose;

    // path.path.resize(numTimes * 4); // utime, x, y, theta
    
    // pose_xyt_t nextPose;
    
    // nextPose.x = 1.0f*SCALAR;
    // nextPose.y = 0.0f*SCALAR;
    // nextPose.theta = M_PI_2;
    
    // for(int n = 0; n < numTimes; ++n)
    // {
    //     path.path[4*n] = nextPose;
    // }
    
    // nextPose.x = 1.0f*SCALAR;
    // nextPose.y = 1.0f*SCALAR;
    // nextPose.theta = M_PI;
    // for(int n = 0; n < numTimes; ++n)
    // {
    //     path.path[4*n + 1] = nextPose;
    // }
    
    // nextPose.x = 0.0f*SCALAR;
    // nextPose.y = 1.0f*SCALAR;
    // nextPose.theta = -M_PI/2;
    // for(int n = 0; n < numTimes; ++n)
    // {
    //     path.path[4*n + 2] = nextPose;
    // }
    
    // nextPose.x = 0.0f*SCALAR;
    // nextPose.y = 0.0f*SCALAR;
    // nextPose.theta = 0;
    // for(int n = 0; n < numTimes; ++n)
    // {
    //     path.path[4*n + 3] = nextPose;
    // }
    
    // Return to original heading after completing all circuits
    
    // nextPose.x = 0.0f*SCALAR;
    // nextPose.y = 0.0f*SCALAR;
    // nextPose.theta = 0; 
    // path.path.insert(path.path.end(), nextPose);
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
