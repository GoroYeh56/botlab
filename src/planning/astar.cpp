#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


// TODO : Implement these functions below:
double h_cost(Node* from, Node* goal){
    double dx = goal->cell.x - from->cell.x;
    double dy = goal->cell.y - from->cell.y;
    double h = sqrt(pow(dx,2) + pow(dy, 2)); // straight line distance
    return h;
}

double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params){

    // TODO: use params and distances?
    double dx = to->cell.x - from->cell.x;
    double dy = to->cell.y - from->cell.y;
    double delta_g = sqrt(pow(dx,2) + pow(dy, 2));

    double cellDistance = distances(to->cell.x, to->cell.y);
    double grid_to_obs_cost = pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
    if(cellDistance > params.maxDistanceWithCost){
        return from->g_cost + delta_g;
    }
    else{
        return from->g_cost + delta_g + grid_to_obs_cost;
    }
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
        std::cout<< "( "<< n->cell.x<<", "<<n->cell.y<<std::endl;
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


///////////////// A* here ///////////////////
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path; // pose_xyt_t path[path_length]
    path.utime = start.utime;
    path.path.push_back(start);
    // path.path[0] = start;    
    path.path_length = path.path.size();

    // /* A* here */
    PriorityQueue Openlist;
    PriorityQueue Closedlist;
 
    Node* start_node = new Node(start.x, start.y);
    Openlist.Q.push(start_node);

    Node* goal_node = new Node(goal.x, goal.y);

    while(!Openlist.empty()){
        Node* cur = Openlist.pop();
        Closedlist.Q.push(cur);
        
        pose_xyt_t cur_pose;
        cur_pose.x = cur->cell.x;
        cur_pose.y = cur->cell.y;
   
        if(cur == goal_node){
            std::cout<<"A start success! Find path and break while loop.\n";
            extract_path(cur); // extract path from this node
            break;
        }
        // 4-direction
        std::vector<Node*> next_nodes = expand_node(cur, distances, params);

        for(auto node : next_nodes){
            // int newx = node->cell.x;
            // int newy = node->cell.y;
            Node* next_node = node;
            int newx = node->cell.x;
            int newy = node->cell.y;
            // if(!Openlist.is_member(next_node) && distances(newx, newy)>= params.minDistanceToObstacle){
            //     Openlist.push(next_node);   
            // }

            // Out of boundaries OR less than minDist OR in closed_list
            if( !distances.isCellInGrid(newx, newy) || distances(newx, newy)< params.minDistanceToObstacle \
            || Closedlist.is_member(next_node)){
                continue;
            }

            // // Whether to change parent?
            // float gt = g_cost[neighbor.id] + dist(chosen_loc , neighbor);    
            double gt = next_node->g_cost + g_cost(cur, next_node, distances, params);               
            bool change_parent = false;
            // // check whether to change y's parent -> x.
            // Case 1: First appear
            if(!Openlist.is_member(next_node)){
                Openlist.push(next_node);
                change_parent = true;
            }
            // Case 2: from cur to neighbor is closer than its original neighbor (relaxation here!!!!)
            else if( gt < next_node->g_cost){
                change_parent = true;
            }

            // This node might have been added to OpenList from other node
            // Suppose we now find a 'new parent' that can make this node's g to be smaller
            // Need to change its parent to this node
            if(change_parent){
                next_node->parent = cur;
                next_node->g_cost = gt;
            }            
        }

    }

    return path;
}
