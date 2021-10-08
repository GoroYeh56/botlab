#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <vector>
using namespace std;

#define SCALE 0.61
#define PI 3.14159

// Speed mode target FAST, SLOW
#define FAST

#ifdef SLOW
    vector<vector<float>> targets= {
        {1.1, 0, 0},          //
        {1.1, -1, -M_PI_2},   //    y more
        {2.3, -1, 0},        // x more
        {2.3, 0.9, M_PI_2},     //
        {3.3, 0.9, 0},          // x more This point too short
        {3.3, -1.2, -M_PI_2},  //
        {4.2, -1.2, 0},      //
        {4.2, -0.1, M_PI_2},      //
        {5.2, -0.1, 0}
    };
#else
    #ifdef FAST
        vector<vector<float>> targets= {
            {1.05, 0, 0},          //
            {1.1, -1, -M_PI_2},   //    y more
            {2.21, -1, 0},        // x more
            {2.21, 0.9, M_PI_2},     //
            {3.23, 0.9, 0},          // x more This point too short
            {3.23, -1.2, -M_PI_2},  //
            {4.2, -1.2, 0},      //
            {4.2, -0.1, M_PI_2},      //
            {5.2, -0.1, 0}
        };
    #else
        vector<vector<float>> targets = {
            {0.55, 0, 0},
            {0.72, -0.625,-1.48},
            {1.354,-0.642,-0.052},
            {1.459,0.513,1.666},
            {1.959, 0.668,0.110},
            {2.296,-0.450,-1.482},
            {2.934,-0.604,1.513},
            {3.011,0.086,-0.052},
            {3.632,0.025,0}
        };
    #endif
#endif


int main(int argc, char** argv)
{std::cout << "here" <<std::endl;
    // int numTimes = 4;
    int numTimes = 1;

    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }

    std::cout << "Commanding robot to drive checkpoint1 " << numTimes << " times.\n";


    robot_path_t path;
    path.path.resize(targets.size()); // utime, x, y, theta

    for(int i=0; i< int(targets.size()); i++){
        pose_xyt_t nextPose;
        nextPose.x = targets[i][0]*SCALE;
        nextPose.y = targets[i][1]*SCALE;
        nextPose.theta = targets[i][2];
        path.path[i] = nextPose;
    }


    path.path_length = path.path.size();

    int index=0;
    for(auto pose : path.path){
        cout<<index++<<": "<<pose.x<<", "<<pose.y<<", "<<pose.theta<<endl;
    }

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
