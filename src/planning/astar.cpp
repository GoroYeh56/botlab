#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>

// TODO : Implement these functions below:
double h_cost(Node* from, Node* goal){
    double dx = goal->cell.x - from->cell.x;
    double dy = goal->cell.y - from->cell.y;
    // double h = sqrt(pow(dx,2) + pow(dy, 2)); // straight line distance
    double h = abs(dx) + abs(dy);
    return h;
}

double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params){

    // TODO: use params and distances?
    double dx = to->cell.x - from->cell.x;
    double dy = to->cell.y - from->cell.y;
    double delta_g = sqrt(pow(abs(dx),2) + pow(abs(dy), 2));

    double cellDistance = distances(to->cell.x, to->cell.y);
    double grid_to_obs_cost = pow(abs(params.maxDistanceWithCost - cellDistance), params.distanceCostExponent);
    // if(cellDistance > params.maxDistanceWithCost){
    if(true){
        return from->g_cost + delta_g;
    }
    // else{
        // return from->g_cost + delta_g + grid_to_obs_cost;
    // }
}

// a fxn to expand a node => Find all the children
std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params){

    // TODO : 4-connect or 8-connect
    std::vector<Node*> neighbors;
    int dirxs[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dirys[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    for(int i=0; i<8; ++i){
        double next_x = node->cell.x + dirxs[i];
        double next_y = node->cell.y + dirys[i];
        // if not obstacles
        if( distances.isCellInGrid(next_x, next_y) && distances(next_x, next_y) >= params.minDistanceToObstacle ){
            // std::cout<<"dis to obs: "<<distances(next_x, next_y)<<std::endl;
            Node* next_node = new Node(next_x, next_y);
            neighbors.push_back(next_node); //      push neighbor Node* into the neighbors vector
        }   
    }
    return neighbors;

}
std::vector<Node* > extract_path(Node* node)          // Backtracing from goal to start (see each node's parent)
{
    std::vector<Node*> path;

    while(node->parent != NULL){
        path.insert(path.begin(), node);
        node = node->parent;
    }
    // std::vector<Node*> ans(path.rbegin(), path.rend())
    
    printf("======= Path ==========\n");
    for(auto n: path){
        std::cout<< "( "<< n->cell.x<<", "<<n->cell.y<<" )"<<std::endl;
    }
    return path;

}

std::vector<pose_xyt_t> extract_path_pose(std::vector<Node*> path, const ObstacleDistanceGrid& distances) // Publish this vector of pose_xyt_t as our final Waypoints!
{

    // TODO: Why need distances? 
    std::vector<pose_xyt_t> path_poses;
    // for(auto node: path){
    for(int i=0; i<path.size()-1; ++i){
        pose_xyt_t pose;
        pose.x = path[i]->cell.x;
        pose.y = path[i]->cell.y;
        float dx = path[i+1]->cell.x - pose.x;
        float dy = path[i+1]->cell.y - pose.y;
        float target_heading = atan2(dy, dx);
        pose.theta = target_heading;
        path_poses.push_back(pose);
    }

    // the last pose
    pose_xyt_t lastpose;
    lastpose.x = path.back()->cell.x;
    lastpose.y = path.back()->cell.y;
    lastpose.theta =path_poses.back().theta;
    path_poses.push_back(lastpose);

    return path_poses;

}




///////////////// A* here ///////////////////
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t astar_path; // pose_xyt_t path[path_length]
    astar_path.utime = start.utime;
    // path.path.push_back(start);
    // path.path[0] = start;    
    // path.path_length = path.path.size();

    // /* A* here */

    // Edge case: if grid_filled : return empty path


    PriorityQueue Openlist;
    PriorityQueue Closedlist;
    
    // cell_t
    Point<double> startp;
    startp.x = start.x;
    startp.y = start.y;

    Point<double> goalp;
    goalp.x = goal.x;
    goalp.y = goal.y;
    // return Point<int> : cell_t here
    // Point<int> start, goal
    auto grid_start = global_position_to_grid_cell(startp, distances);
    auto grid_goal = global_position_to_grid_cell(goalp, distances);

    Node* start_node = new Node(grid_start.x, grid_start.y);
    start_node->g_cost = 0;
    Openlist.push(start_node);
    Node* goal_node = new Node(grid_goal.x, grid_goal.y);

    std::vector<Node*> node_path;
    bool success = false;

    std::cout<<"Start : "<<start_node->cell.x<<", "<<start_node->cell.y<<std::endl; // (-5, 0)
    std::cout<<"Goal : "<<goal_node->cell.x<<", "<<goal_node->cell.y<<std::endl;    // (5, 0)

    long int id = 0;
    long int BREAKING_ITERS = 1000000;
    // int BREAKING_ITERS = 100000;

    // for(int i=0; i<1; i++){
    //     std::cout<<"distance(151,y): "<<distances(150,52)<<"\n";
    // }
    std::vector< Node*> nodes_to_be_deleted;

    while(!Openlist.empty()){
        id++;

        if(id >= BREAKING_ITERS) break;

        // #ifdef DEBUG
        // if(id% 100000 ==0){
        //     std::cout<<"\nIteration: "<<id<<std::endl;
        // }
        // #endif
        // std::cout<<"Openlist:\n";
        // Openlist.print();
        Node* cur = Openlist.pop();
        // std::priority_queue<Node*, std::vector<Node*>, CompareNode> Q2 = Openlist.Q;
        // while(!Q2.empty()){
        //     Node* node = Q2.top();
        //     Q2.pop();
        //     std::cout<<"\tdistance: "<<distances(node->cell.x, node->cell.y)<<" f: "<<node->f_cost() <<", h: "<<node->h_cost <<", g: "<<node->g_cost<<" (" <<node->cell.x<<", "<<node->cell.y<<")\n";
        // }

        // Closedlist.push(cur);
        // #ifdef DEBUG
            // std::cout<<"Current node: " << cur->cell.x <<", "<<cur->cell.y<< " f: "<< cur->f_cost()<<" g: "<< cur->g_cost<<" h: "<< cur->h_cost<< std::endl;
        // #endif
        // std::cout<<"Closedlist: "<<std::endl;
        // Closedlist.print();
   
        if(cur->cell.x == goal_node->cell.x && cur->cell.y == goal_node->cell.y){
            std::cout<<"A* success! Find path and break while loop.\n";
            node_path =  extract_path(cur); // extract path from this node
            success = true;
            break; 
            
        }
        // 4-direction
        std::vector<Node*> next_nodes = expand_node(cur, distances, params);
        for(auto n: next_nodes){
            nodes_to_be_deleted.push_back(n);
        }
        // std::cout<<"    neighbots: \n";
        // for(auto n: next_nodes)
        //     std::cout<<"( "<<n->cell.x<<", "<<n->cell.y<<")"<<std::endl;
        // std::cout<<std::endl;

        for(auto node : next_nodes){

            int newx = node->cell.x;
            int newy = node->cell.y;
            Node* next_node = node;
            next_node->g_cost = g_cost(cur, next_node, distances, params); 
            next_node->h_cost = h_cost(next_node, goal_node);

            // Out of boundaries OR less than minDist OR in closed_list
            // std::cout<<"Neighbor: "<<newx<<", "<<newy<<std::endl;
            if(Closedlist.is_member(node)){
                // Problem here: didn't push (50, 150) into Closedlist!
                // std::cout<<"Already explored node "<<newx<<", "<<newy<<std::endl;
                continue;
            }
            // std::cout<<params.minDistanceToObstacle<<std::endl; // 0.1 from astar_test.cpp   // TODO : +0.05
            if( !distances.isCellInGrid(newx, newy) || distances(newx, newy) < params.minDistanceToObstacle){
                // std::cout<<"Out of boundary or too close \n";
                continue;
            }

            // // Whether to change parent?
            // Calculate g for this next_node
            double gt = g_cost(cur, next_node, distances, params);               
            bool change_parent = false;

            // // check whether to change y's parent -> x.
            // Case 1: First appear
            if(!Openlist.is_member(next_node)){
                // std::cout<<"First visit this node\n";
                // std::cout<<"Push "<<next_node->cell.x<<", "<<next_node->cell.y<<" f: "<<next_node->f_cost()<<std::endl;
                Openlist.push(next_node);
                change_parent = true;
            }
            // Case 2: from cur to neighbor is closer than its original neighbor (relaxation here!!!!)
            else if( gt < Openlist.get_member(next_node)->g_cost){
                // std::cout<<"Find a closer node\n";
                change_parent = true;
            }
            // This node might have been added to OpenList from other node
            // Suppose we now find a 'new parent' that can make this node's g to be smaller
            // Need to change its parent to this node
            if(change_parent){
                // std::cout<<"change parent!\n";
                Openlist.get_member(next_node)->parent = cur;
                Openlist.get_member(next_node)->g_cost = gt;
            }            
        }

    }



    // TODO: problem: Can't find goal (10000 iterations)
    if(success){
        std::vector<pose_xyt_t> pose_path = extract_path_pose(node_path, distances);
        astar_path.path_length = pose_path.size();
        astar_path.path.resize(astar_path.path_length);
        // std::cout<<"size of path: "<<node_path.size()<<"\n";
        #ifdef DEBUG
        for(auto p : node_path){
            std::cout<<"("<<p->cell.x<<","<<p->cell.y<<"): distance: "<< distances(p->cell.x, p->cell.y)<<"\n";
        }
        #endif
        // std::cout<<"path length: "<<astar_path.path_length<<std::endl;
        // while(1);
        // robot_path_t path;
        // pose_xyt_t path[path_length];
        int i=0; // start index
        // std::cout<<"size of path: "<<pose_path.size()<<"\n";
        for(auto pose : pose_path){ // pose_xyt_t;
            // std::cout<<"i: "<<i<<std::endl;
            Point<double> posep;
            posep.x = pose.x;
            posep.y = pose.y;
            
            Point<double> global_pose =  grid_position_to_global_position(posep, distances);
            // pose_xyt_t p;
            astar_path.path[i].utime = start.utime; //TODO  now
            astar_path.path[i].x = float(global_pose.x);
            astar_path.path[i].y = float(global_pose.y);
            astar_path.path[i].theta = 0.0;
            i++;
        }
    }
    else{
        if(Openlist.empty()){
            std::cout<<"Openlist empty. Explore all free cell but NOT reach goal\n";
        }
    }
    // for(auto p : astar_path.path){
    //     std::cout<<"("<<p.x<<","<<p.y<<"): distance: "<< distances(p.x, p.y)<<"\n";
    // }


    // delete Openlist, Closedlist;
    // free(Openlist);
    for(auto n: nodes_to_be_deleted){
        delete n;
    }

    return astar_path;
}

// shortest distance to target
double h_cost(Node* from, Node* goal) {
    double dx = goal->cell.x - from->cell.x;
    double dy = goal->cell.y - from->cell.y;

    return std::sqrt((dx * dx) + (dy * dy));
}

// cost of the path up from one node to another
double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    
    double cost = 0.0;
    Node* currentNode = from;
    double dx = to->cell.x - from->cell.x;
    double dy = to->cell.y - from->cell.y;
    double distanceCost = std::sqrt((dx * dx) + (dy * dy));
    
    
    //cost += from->g_cost;
    //cost += distanceCost;
 
    float cellDistance = distances(currentNode->cell.x, currentNode->cell.y);
    if (cellDistance > params.minDistanceToObstacle && cellDistance < params.maxDistanceWithCost) {
        cost += std::pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
    }
    currentNode = currentNode->parent;

    return cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    std::vector<Node*> kiddos;
    
    // This was super clever I can't believe I never thought of doing this before in this way
    // const int xDeltas[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };
    // const int yDeltas[8] = { 0, -1, -1, -1, 1, 1, -1, 0 };
    const int xDeltas[8] = { 1, 1, 1, 0, -1, -1, -1, 0};
    const int yDeltas[8] = {-1, 0, 1, 1, 1, 0, -1, -1};

    //add checking for params later!
    for (int i = 0; i < 8; i++) {
        Node* currentKiddo = new Node(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        currentKiddo->parent = node;
        
        if (distances.isCellInGrid(currentKiddo->cell.x,currentKiddo->cell.y)) {
            
            if (distances(currentKiddo->cell.x, currentKiddo->cell.y) > params.minDistanceToObstacle + 0.05) {
                
                kiddos.push_back(currentKiddo);
            }
        }
       
    }

    return kiddos;

}

std::vector<Node*> extract_node_path(Node* node) {
    std::vector<Node*> path;
    Node* currentNode = node;
    while (node != NULL) {
        path.push_back(node);
        node = node->parent;
    }
    return path;
}


std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*> nodePath, const ObstacleDistanceGrid& distances) {
    std::vector<pose_xyt_t> posePath;
    //std::cout << "\nPath:";
    for (int i = 0; i < nodePath.size(); i++) {
        auto globalPos = grid_position_to_global_position(nodePath.at(i)->cell, distances);
        pose_xyt_t currentPose;
        currentPose.x = globalPos.x;
        currentPose.y = globalPos.y;
        posePath.push_back(currentPose);
        //std::cout << "\n" << currentPose.x << "," << currentPose.y;
        // deal with currentPose.theta later not really used rn 
    }
    return posePath;
}

