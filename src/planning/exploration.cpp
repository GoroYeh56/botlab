#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>

const float kReachedPositionThreshold = 0.05f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}


Exploration::Exploration(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    params.robotRadius = 0.2; // TODO : Larger (original 0.2)
    planner_.setParams(params);
}


bool Exploration::exploreEnvironment()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration(); // copyDataforUpdate to update Map & Pose
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}
// if lcm pass new map and pose channel
bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();  // update currentMap_ & currentPose_
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;
    
    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing(); // nextState = exploring_map
                break;
            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                if(nextState == exploration_status_t::STATE_FAILED_EXPLORATION)
                    std::cout<<"Fail exploration from STATE_EXPLORING_MAP\n";
                break;
                
            case exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
    case exploration_status_t::STATE_INITIALIZING:
        nextState = executeInitializing();
        break;
    case exploration_status_t::STATE_EXPLORING_MAP:
        nextState = executeExploringMap(stateChanged);
        break;

    case exploration_status_t::STATE_RETURNING_HOME:
        nextState = executeReturningHome(stateChanged);
        break;

    case exploration_status_t::STATE_COMPLETED_EXPLORATION:
        nextState = executeCompleted(stateChanged);
        break;

    case exploration_status_t::STATE_FAILED_EXPLORATION:
        nextState = executeFailed(stateChanged);
        break;
    }

    stateChanged = nextState != state_;
    state_ = nextState;

} while (stateChanged);

//if path confirmation was not received, resend path
if (!pathReceived_)
{
    std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

    std::cout << "timestamp: " << currentPath_.utime << "\n";

    for (auto pose : currentPath_.path) {
        std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    }std::cout << "\n";

    lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
}

//if path changed, send current path
if (previousPath.path != currentPath_.path)
{

    std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

    std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

    for (auto pose : currentPath_.path) {
        std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    }std::cout << "\n";

    lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

    pathReceived_ = false;
    most_recent_path_time = currentPath_.utime;
}

}


int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;

    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);

    return exploration_status_t::STATE_EXPLORING_MAP;
}


int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    *
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */
    double minFrontierLength = 0.15; // 0.05 * 3 = 

    // When to select a new frontier (of cells) to explore
    // 1. front 
    // from currentPose, run BFS expand (4 connect ) to find frontier_cell and grow_frontier cells
    
    //////////////// Robert Lai !!! ////////////////
    float delay_time = 6000000; // 6 sec
    frontiers_ = find_map_frontiers(currentMap_, currentPose_, minFrontierLength );
    if (currentPath_.path.size()< 2 || utime_now() - most_recent_path_time > delay_time) { //Too close to frontier Or 1000us update
        if(currentPath_.path_length < 2)
            std::cout<<"cur path length <2. "<<currentPath_.path_length<<"\n";
        else
            std::cout<<"Too long. delta time: "<<utime_now() - most_recent_path_time<<" us\n";
        planner_.setMap(currentMap_);

        if(currentPath_.path.size() >1)
            planner_.setPrevGoal(currentPath_.path[currentPath_.path_length-1]);

        
        usleep(1000000); // 1s
        currentPath_ = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
        //robotpath_t
        // pose_xyt_t path[path_length];
        
        //Reduce waypoints.
        if(currentPath_.path.size() > 5){

            std::vector<pose_xyt_t> tmpPath;

            for(int i=0; i<currentPath_.path_length; ++i){
                if(i%2==1){
                    tmpPath.push_back(currentPath_.path[i]);
                }
            }
            this->currentPath_.path = tmpPath;
            this->currentPath_.path_length = tmpPath.size();
            std::cout<<"set currentPath_ : length: "<<currentPath_.path_length<<"\n";
        }

        if(currentPath_.path.size()>0){
            if(currentPath_.path.back().x == planner_.getPrevGoal().x && currentPath_.path.back().y == planner_.getPrevGoal().y)
            {
                std::cout<<"current goal == prev_goal , "<<currentPath_.path.back().x<<", "<<currentPath_.path.back().y<< "\n";
                // Manually set a path ahead of the robot current cell
                // current_cell = global_position_to_grid_cell(currentPose_, map); 
                float dx, dy;
                const float PI = 3.14159;
                if(currentPose_.theta>= 0.0 &&  currentPose_.theta < PI/2 ){ // GO right up
                    dx = +currentMap_.metersPerCell();
                    dy = +currentMap_.metersPerCell();
                }
                else if( currentPose_.theta>= PI/2 &&  currentPose_.theta < PI ){ // GO left up
                    dx = -currentMap_.metersPerCell();
                    dy = +currentMap_.metersPerCell();

                }
                else if(currentPose_.theta <= -PI/2 &&  currentPose_.theta > -PI){ // Go left down
                    dx = -currentMap_.metersPerCell();
                    dy = -currentMap_.metersPerCell();

                }
                else{ // currentPose_.theta <0  &&  currentPose_.theta > -PI/2 : Go right down
                    dx = +currentMap_.metersPerCell();
                    dy = -currentMap_.metersPerCell();
                }
                std::vector<pose_xyt_t> forward_path {
                    pose_xyt_t{currentPose_.utime, currentPose_.x + dx, currentPose_.y + dy, 0 },
                    pose_xyt_t{currentPose_.utime, currentPose_.x + 2*dx, currentPose_.y + 2*dy, 0 }
                    // ,pose_xyt_t{currentPose_.utime, currentPose_.x+3*currentMap_.metersPerCell(), currentPose_.y+3*currentMap_.metersPerCell(), 0 }
                };
                currentPath_.path = forward_path;
            }
        }
        // this->currentPath_ = tmpPath2;
    }
    ///////////////////////////////////////////////

    // Exploration Thread:  A* gets the wrong map
    // The map planner use is not the same as botgui sees.

    // bool stateChanged = initialize;
    // // If change state: re find new map_frontiers

    // // if there is still frontier points: we should keep exploreing
    // if(stateChanged){
    //     std::cout<<"frontiers:\n";
    // // Use a flag(goal threshold) to determine whether we're still heading to the current exploring goal (frontier point)
    //     // Use a priority_queue (priority: distance from robot to frontier)
        // planner_.setMap(currentMap_);
        // this->frontiers_ = find_map_frontiers(this->currentMap_, 
        //                                     this->currentPose_,
        //                                     minFrontierLength);
        // if(this->frontiers_.empty()){
        //     std::cout<<"Empty frontiers!\n";

        // }
        // else{
        //     std::cout<<"Frontiers:\n";
        //     for(int i=0; i<this->frontiers_.size(); i++){
        //         std::cout<<frontiers_[i].cells[0];
        //     }
        //     std::cout<<std::endl;            
                                                
        //     this->currentPath_ = plan_path_to_frontier(this->frontiers_, 
        //                                 this->currentPose_,
        //                                 currentMap_,
        //                                 planner_);
        // }
    // }

    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    
    // If no frontiers remain, then exploration is complete
    if(frontiers_.empty())
    {
        std::cout<<"No frontiers. STATUS_COMPLETE_EXPLORATION. =>RETURNING_HOME...\n";
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        // std::cout<<"currentPath.path.size()>1. len: "<<currentPath_.path.size()<<std::endl;
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {

        // PROBLEM HERE!!!10/26
        std::cout<<"Have valid goal(target_cell) but A* can't find path (return empty path)\n";
        std::cout<<"We have frontiers but NO valid path\n";
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            std::cout<<"executeExploringMap: we still has a path to follow...\n";
            return exploration_status_t::STATE_EXPLORING_MAP;
            
        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            return exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose 0.05
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */
    std::cout<<"executeReturningHome: homePose_: "<<homePose_.x<<", "<<homePose_.y<<std::endl;
    float delay_time = 15000000; // 6 sec
    
    if ( utime_now() - most_recent_path_time > delay_time) { //Too close to frontier Or 1000us update
        if(planner_.isValidGoal(pose_xyt_t{currentPose_.utime, (float)homePose_.x, (float)homePose_.y, 0})){
            
            
            this->currentPath_ = this->planner_.planPath(this->currentPose_, this->homePose_);
            //Reduce waypoints.
            
            // if(currentPath_.path_length > 5){

            //     std::vector<pose_xyt_t> tmpPath;

            //     for(int i=0; i<currentPath_.path_length; ++i){
            //         if(i%2==1){
            //             tmpPath.push_back(currentPath_.path[i]);
            //         }
            //     }
            //     if(currentPath_.path.size()%2==0)
            //         tmpPath.push_back(currentPath_.path[currentPath_.path.size()-1]);

            //     this->currentPath_.path = tmpPath;
            //     this->currentPath_.path_length = tmpPath.size();
            //     std::cout<<"set currentPath_ : length: "<<currentPath_.path_length<<"\n";
            // }

        }
        else{
            std::cout<<"homePose NOT valid goal.\n ";
        }
    }
    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;
    
    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y), 
                                                Point<float>(currentPose_.x, currentPose_.y));

    // distToHome =  0.0422591;
    std::cout << "Distance to home: " << distToHome << std::endl;
    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_RETURNING_HOME;
    }
    else if(status.status == exploration_status_t::STATUS_COMPLETE){
        return exploration_status_t::STATE_COMPLETED_EXPLORATION;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}