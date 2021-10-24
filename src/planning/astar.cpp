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
    double delta_g = sqrt(pow(dx,2) + pow(dy, 2));

    double cellDistance = distances(to->cell.x, to->cell.y);
    double grid_to_obs_cost = pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
    // if(cellDistance > params.maxDistanceWithCost){
    // if(true){
        // return from->g_cost + delta_g;
    // }
    // else{
        return from->g_cost + delta_g + grid_to_obs_cost;
    // }
}

// a fxn to expand a node => Find all the children
std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params){

    // TODO : 4-connect or 8-connect
    std::vector<Node*> neighbors;
    int dirs[5] = {0, 1, 0, -1, 0};
    for(int i=0; i<4; ++i){
        double next_x = node->cell.x + dirs[i];
        double next_y = node->cell.y + dirs[i+1];
        // if not obstacles
        if( distances.isCellInGrid(next_x, next_y) && distances(next_x, next_y) > params.minDistanceToObstacle ){
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
    for(auto node: path){
        pose_xyt_t pose;
        pose.x = node->cell.x;
        pose.y = node->cell.y;
        path_poses.push_back(pose);
    }
    return path_poses;

}


// void push(PriorityQueue Openlist, Node* node){
//     Openlist.Q.push(node);
//     Openlist.elements.push_back(node);
// }

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

    int id = 0;
    while(!Openlist.empty()){
        id++;

        if(id >= 500) break;

        #ifdef DEBUG
            std::cout<<"\nIteration: "<<id<<std::endl;
        #endif
        // std::cout<<"Openlist:\n";
        // Openlist.print();
        Node* cur = Openlist.pop();
        // Closedlist.push(cur);
        #ifdef DEBUG
            std::cout<<"Current node: " << cur->cell.x <<", "<<cur->cell.y<< " f: "<< cur->f_cost()<<" g: "<< cur->g_cost<<" h: "<< cur->h_cost<< std::endl;
        #endif
        // std::cout<<"Closedlist: "<<std::endl;
        // Closedlist.print();
   
        if(cur->cell.x == goal_node->cell.x && cur->cell.y == goal_node->cell.y){
            std::cout<<"A start success! Find path and break while loop.\n";
            node_path =  extract_path(cur); // extract path from this node
            success = true;
            break; 
            
        }
        // 4-direction
        std::vector<Node*> next_nodes = expand_node(cur, distances, params);

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

            if( !distances.isCellInGrid(newx, newy)|| distances(newx, newy)< params.minDistanceToObstacle){
                std::cout<<"Out of boundary or too close \n";
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
        // std::cout<<"path length: "<<astar_path.path_length<<std::endl;
        // while(1);
        // robot_path_t path;
        // pose_xyt_t path[path_length];
        int i=0; // start index
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
    
    // for(auto p : astar_path.path){
    //     std::cout<<"("<<p.x<<","<<p.y<<")\n";
    // }

    return astar_path;
}
