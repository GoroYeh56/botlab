#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <stack>
#include <set>
#include <cassert>
// #include <bits/stdc+.h>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);


pose_xyt_t nearest_navigable_cell(pose_xyt_t pose, Point<float> desiredPosition, const OccupancyGrid& map, const MotionPlanner& planner) {

    Point<double> gridPos = global_position_to_grid_position(Point<double>((double)desiredPosition.x, (double)desiredPosition.y), map );
    pose_xyt_t currentPose;
    currentPose.x = gridPos.x;
    currentPose.y = gridPos.y;

    bool foundFreeSpace = false;

    int w = 1;
    int h = 1;
    int x = w;
    int y = 0;
    int dir = 0;

    while (!foundFreeSpace) {
        currentPose.x = x;
        currentPose.y = y;
        if (planner.planPath(pose, currentPose).path.size() > 1) {
            foundFreeSpace = true;
        }
        else {
            switch (dir) {
            case 0: //up
                if (y >= h) {
                    dir = 1;
                    x--;
                }
                else {
                    y++;
                }
                break;
            case 1: //left
                if (x <= 0) {
                    dir = 2;
                    y--;
                }
                else {
                    x--;
                }
                break;
            case 2: //down
                if (y <= 0) {
                    dir = 3;
                    x++;
                }
                else {
                    y--;
                }
                break;
            case 3: //right
                if (x >= w) {
                    dir = 0;
                    x++;
                    y--;
                    w += 2;
                    h += 2;
                }
                else {
                    x++;
                }
                break;
            }
        }

    }

    Point<double> globalPos = grid_position_to_global_position(Point<double>((double)x, (double)y),map);
    currentPose.x = globalPos.x;
    currentPose.y = globalPos.y;
    return currentPose;
}


pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner) {
    
    Point<double> gridPos = global_position_to_grid_position(Point<double>((double)position.x, (double)position.y),map);
    pose_xyt_t currentPose;
    currentPose.x = gridPos.x;
    currentPose.y = gridPos.y;

    bool foundFreeSpace = false;

    int w = 1;
    int h = 1;
    int x = w;
    int y = 0;
    int dir = 0;
    
    while (!foundFreeSpace) {
        
        if (map(x, y) < 0) {
            foundFreeSpace = true;
        }
        else {
            switch (dir) {
            case 0: //up
                if (y >= h) {
                    dir = 1;
                    x--;
                }
                else {
                    y++;
                }
                break;
            case 1: //left
                if (x <= 0) {
                    dir = 2;
                    y--;
                }
                else {
                    x--;
                }
                break;
            case 2: //down
                if (y <= 0) {
                    dir = 3;
                    x++;
                }
                else {
                    y--;
                }
                break;
            case 3: //right
                if (x >= w) {
                    dir = 0;
                    x++;
                    y--;
                    w += 2;
                    h += 2;
                }
                else {
                    x++;
                }
                break;
            }
        }

    }

    Point<double> globalPos = grid_position_to_global_position(Point<double>((double)x, (double)y),map);
    currentPose.x = globalPos.x;
    currentPose.y = globalPos.y;
    return currentPose;
}

double path_length(const robot_path_t& path) {
    double length = 0.0;
    pose_xyt_t prevPose = path.path.at(0);
    pose_xyt_t currPose = path.path.at(0);
    for (int i = 0; i < path.path.size() - 1; i++) {
        currPose = path.path.at(i);
        double dx = currPose.x - prevPose.x;
        double dy = currPose.y - prevPose.y;
        length += std::sqrt(dx * dx + dy * dy);
        prevPose = currPose;
    }
    return length;
}

robot_path_t path_to_frontier(const frontier_t& frontier, const pose_xyt_t& pose, const OccupancyGrid& map, const MotionPlanner& planner) {
    Point<float> goalPoint = frontier.cells.at((int)(frontier.cells.size() / 2));
    pose_xyt_t goal = search_to_nearest_free_space(goalPoint, map, planner);
    Point<float> freeGoalPoint(goal.x, goal.y);
    goal = nearest_navigable_cell(pose, freeGoalPoint, map, planner);
    return planner.planPath(pose, goal);
}

std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    // Start from the robot current position(cell)
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontiers;
}

pose_xyt_t nearest_navigable_cell(pose_xyt_t pose, 
                                  Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner)
{
    pose_xyt_t navigable_cell;

    return navigable_cell;

}

pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner)
{
    /*The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
           be able to drive straight to a frontier cell, but will need to drive somewhere close*/

    pose_xyt_t free_space;

    // current frontier.cells[mid] : position (float x, float y)
    // map  ( isCellinMap)
    // planner: minD
    // float radius = planner.Searchparams_.minDistanceToObstacle; // 0.2
    float radius = 0.2;

    // expand 4 directions:
    int dx[4] = {0, 0, 1, -1};
    int dy[4] = {1, -1, 0, 0};
    for(int i=0; i<4; i++){

        int newx = position.x + dx[i];
        int newy = position.y + dy[i];
        if(map.isCellInGrid(newx, newy)){
            
            free_space.x = newx;
            free_space.y = newy;
            break;
        }
    }




    return free_space;

}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */
    robot_path_t Path;
    // pose_xyt_t start = robotPose;
    // pose_xyt_t goal;

    // TODO
    // self-define compare function for priority_queue
    auto eucDistance = [robotPose](Point<double> a, Point<double> b) {
        return sqrt((a.x - robotPose.x)*(a.x-robotPose.x) + (a.y-robotPose.y)*(a.y-robotPose.y))
                > sqrt((b.x - robotPose.x)*(b.x-robotPose.x) + (b.y-robotPose.y)*(b.y-robotPose.y));
    };
    // Priority queue : push all frontier's cells and search for a nearest free cell 
    std::priority_queue< Point<double>, std::vector<Point<double>>, decltype(eucDistance)> pq(eucDistance);

    // Push all cells into pq
    for(int i=0; i< frontiers.size(); ++i){
        for(int j=0; j<frontiers[i].cells.size(); ++j){
            pq.push(frontiers[i].cells[j]);
        }
    }

    // update ObstacleDistanceGrid (setDistance)
    // planner.setMap(map);
    // planner.setMap(map);

    // Use pq to pop one cell at a time, then
    // use DFS to explore 4 direction from that cell to find a FREE cell 
    // that is the nearest to the robot current position
    static bool testOnce = false;

    bool foundPath = false;
    Point<int> target_cell; 

    int debug_cnt = 10;
    int it = 0;
    std::cout<<"\n Call plan_path_to_frontier again:...\n";
    std::cout<<"PQ size: # frontiers cells: "<<pq.size()<<std::endl; // 5217
    // std::cout<<"logOdds(108,107) "<<(int)map.logOdds(108,107)<<std::endl;
    // std::cout<<"logOdds(115,104) "<<(int)map.logOdds(115,104)<<std::endl;
      
    while(!pq.empty() && !foundPath ){
        it++;
        std::cout<<"PQ: "<<pq.size()<<" cells left\n";
        // if(it>=debug_cnt){
        //     std::cout<<"Iterate through "<<debug_cnt<<" frontier cells. Break\n";
        //     break;
        // }

        Point<int> cur_cell = global_position_to_grid_cell(pq.top(), map);
        std::cout<<"Current frontier cell: "<<cur_cell.x<<", "<<cur_cell.y<<std::endl;
        // a stack for cells
        // std::stack<Point<int>> stk;
        std::queue<Point<int>> stk;
        stk.push(cur_cell); // Point<int> (x,y)

        std::set<Point<int>> visited;
        bool foundFreeCell = false; // if found : break and return A* path
        
        while(!stk.empty() && !foundFreeCell){

            // Point<int> cur = stk.top();
            Point<int> cur = stk.front();
            
            // std::cout<<"cur free cell in stack: "<<cur.x<<", "<<cur.y;

            // if(planner.obstacleDistances().isCellInGrid(cur.x, cur.y))
            //     std::cout<<"distance:"<<planner.obstacleDistances()(cur.x, cur.y)<<"\n";
            // else
            //     std::cout<<"\n";

            Point<double> cur_global = grid_position_to_global_position(cur, map);
            stk.pop();


            // TODO: If goal is "too close to our self , Cannot!!" 
            
            if(/*abs(cur.x-robotPose.x)>=3 && abs(cur.y-robotPose.y)>=3 &&*/ (int)map.logOdds(cur.x, cur.y) <0  && planner.isValidGoal(pose_xyt_t{robotPose.utime, (float)cur_global.x, (float)cur_global.y, 0})){
                foundFreeCell = true;
                target_cell = cur; // Point<int>
            }
            else{
                int step_size = 2; // PARAMETERS TO BE TUNED
                // DFS 4 directions to push cells into stack
                // int dx[8] = {0,  0, 1, 1,  1, -1, -1, -1};
                // int dy[8] = {1, -1, 0, 1, -1, -1,  0,  1};
                int dx[4] = {0,  0, 1, -1};
                int dy[4] = {1, -1, 0, 0};                
                for(int k=0; k< 4; ++k){
                    int next_x = cur.x + dx[k]*step_size;
                    int next_y = cur.y + dy[k]*step_size;
                    // if((int)map.isCellInGrid(next_x, next_y))
                    //     std::cout<<"next x "<<next_x<<", y "<<next_y<<", odds: "<<(int)map.logOdds(next_x, next_y)<<"\n";
                    // else{
                    //     std::cout<<"next x "<<next_x<<", y "<<next_y<<", NOT in map. Out of boundary.\n";
                    // }

                    // if( visited.find(Point<int>(next_x, next_y) ) !=visited.end()){
                    //     std::cout<<"Already visit "<<next_x<<", "<<next_y<<"\n";
                    //     continue;
                    // }

                    // if(!map.isCellInGrid(next_x, next_y)){
                    //     std::cout<<"Not in map "<<next_x<<", "<<next_y<<"\n";
                    //     continue;
                    // }
                    if( visited.find(Point<int>(next_x, next_y) )==visited.end() && map.isCellInGrid(next_x, next_y) && (int)map.logOdds(next_x, next_y) < 0){
                        // std::cout<<"Push left free cell: "<<cur.x-step_size<<", "<<cur.y<<" dis: "<<planner.obstacleDistances()(next_x, next_y)<<std::endl;
                        stk.push(Point<int>(next_x, next_y));
                        visited.insert(Point<int>(next_x, next_y));
                    }
                    // if( visited.find(Point<int>(cur.x, cur.y-step_size))==visited.end() && planner.obstacleDistances().isCellInGrid(cur.x, cur.y-step_size) && map.logOdds(cur.x, cur.y-step_size) < 0){                   
                    //     std::cout<<"Push down free cell: "<<cur.x<<", "<<cur.y-step_size<<" dis: "<<planner.obstacleDistances()(cur.x,cur.y-step_size)<<std::endl;
                    //     stk.push(Point<int>(cur.x, cur.y-step_size));
                    //     visited.insert(Point<int>(cur.x, cur.y-step_size));
                    // }
                    // if( visited.find(Point<int>(cur.x+step_size, cur.y))==visited.end() && planner.obstacleDistances().isCellInGrid(cur.x+step_size, cur.y) && map.logOdds(cur.x+step_size, cur.y) < 0){                  
                    //     std::cout<<"Push right free cell: "<<cur.x+step_size<<", "<<cur.y<<" dis: "<<planner.obstacleDistances()(cur.x+step_size,cur.y)<<std::endl;
                    //     stk.push(Point<int>(cur.x+step_size, cur.y));
                    //     visited.insert(Point<int>(cur.x+step_size, cur.y));
                    // }
                    // if( visited.find(Point<int>(cur.x, cur.y+step_size))==visited.end() && planner.obstacleDistances().isCellInGrid(cur.x, cur.y+step_size) && map.logOdds(cur.x, cur.y+step_size) < 0){                
                    //     std::cout<<"Push up free cell: "<<cur.x<<", "<<cur.y+step_size<<" dis: "<<planner.obstacleDistances()(cur.x,cur.y+step_size)<<std::endl;
                    //     stk.push(Point<int>(cur.x, cur.y+step_size));
                    //     visited.insert(Point<int>(cur.x, cur.y+step_size));
                    // }
                }
            }             
        }
        // delete &stk;
        // Found chosen target_cell
        if(foundFreeCell)
            std::cout<<"Found target cell: "<<target_cell.x<<", "<<target_cell.y<<std::endl;
        else
            std::cout<<"Stack empty. DFS failed, no target cell at this frontier\n";
        // while(1);
        pq.pop();
        Point<double> target_cell_global = grid_position_to_global_position(target_cell, map);
        Path = planner.planPath(robotPose, pose_xyt_t{robotPose.utime, (float)target_cell_global.x, (float)target_cell_global.y, 0});
        if(Path.path_length >1 ) foundPath = true;
    }

    
    if(foundPath) return Path;
    else{
        std::cout<<"Fail to find a path to target_cell: "<<target_cell.x<<", "<<target_cell.y<<"\n";
    }


    // delete &pq;

    return Path;
    // TODO : If all empty: means we've explored the whole environment

}

// 1. It is in the map 2. its log odds is zero 3. its neighbor is free
bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0 (its neighbor is a free cell)
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}