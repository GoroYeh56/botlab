#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    //std::cout << "\n\ntesting if I can ever even print anything out :/\n\n";
    PriorityQueue openList;
    PriorityQueue closedList;

    auto gridGoal = global_position_to_grid_position({ goal.x, goal.y }, distances);
    Node goalNode(gridGoal.x, gridGoal.y);

    auto gridStart = global_position_to_grid_position({ start.x, start.y }, distances);
    Node startNode(gridStart.x, gridStart.y);
    startNode.parent = NULL;
    openList.push(&startNode);

   // std::cout << "\n-1\n";
    while (!openList.empty()) {
        //std::cout << "\n0\n";
        Node* q = openList.pop();
        //std::cout << "\nq: " << q;
        //std::cout << "\n q: " << q->cell.x << "," << q->cell.y << "\n";
        //std::cout << "\n1\n";
        std::vector<Node*> kiddos = expand_node(q, distances, params);
        
        //std::cout << "\nkiddos size: " << kiddos.size();

        for (int i = 0; i < kiddos.size(); i++) {
            //std::cout << "\nkiddos.at(i): " << kiddos.at(i);
            //std::cout << "\n kiddos.at(i): " << kiddos.at(i)->cell.x << "," << kiddos.at(i)->cell.y;
            
            if ((kiddos.at(i)->cell.x == goalNode.cell.x) && (kiddos.at(i)->cell.y == goalNode.cell.y)) {
                robot_path_t path;
                path.utime = start.utime;
                std::vector<pose_xyt_t> pathVec = extract_pose_path(extract_node_path(kiddos.at(i)), distances);
                std::reverse(pathVec.begin(), pathVec.end());
                path.path = pathVec;
                path.path_length = path.path.size();
                //std::cout << "\nFOUND PATH!!\n";
                return path;
            }
            else {
                //std::cout << "\n2\n";
                //std::cout << "\n kiddos.at(i)->g_cost: " << kiddos.at(i)->g_cost;
                //std::cout << "\n q->g_cost: " << q->g_cost;
                //std::cout << "\n g_cost(): " << g_cost(q, kiddos.at(i), distances, params);
                kiddos.at(i)->g_cost = q->g_cost + g_cost(q, kiddos.at(i), distances, params);
                //std::cout << "\n3\n";
                kiddos.at(i)->h_cost = h_cost(kiddos.at(i), &goalNode);
                //std::cout << "\n4\n";
                bool skip = false;
                if ((openList.is_member(kiddos.at(i)))) {
                    //std::cout << "\n4.5\n";
                    //std::cout << "kiddos.at(i)->f_cost: " << kiddos.at(i)->f_cost();
                    //std::cout << "\n4.6\n";
                    if (!(openList.get_member(kiddos.at(i))->f_cost() > kiddos.at(i)->f_cost())) { // problem child
                        skip = true;
                        //std::cout << "\n6\n";
                        
                    } 
                }
                if ((closedList.is_member(kiddos.at(i)))) {
                    //std::cout << "\n7\n";
                    if (!(closedList.get_member(kiddos.at(i))->f_cost() > kiddos.at(i)->f_cost())) {
                        skip = true;
                    }
                }
                if (distances(kiddos.at(i)->cell.x, kiddos.at(i)->cell.y) <= params.minDistanceToObstacle) {
                    skip = true;
                }
                if (!skip) {
                   // std::cout << "\n8\n";
                    openList.push(kiddos.at(i));
                }
               
            }
        }
       // std::cout << "\n9\n"; //
        closedList.push(q);
    }

    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
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
    Node* currentNode = to;
    double dx = to->cell.x - from->cell.x;
    double dy = to->cell.y - from->cell.y;
    double distanceCost = std::sqrt((dx * dx) + (dy * dy));
    //add checking for params later!
    while (currentNode != from) {
        cost += from->g_cost;
 
        float cellDistance = distances(currentNode->cell.x, currentNode->cell.y);
        if (cellDistance > params.minDistanceToObstacle && cellDistance < params.maxDistanceWithCost) {
            cost += std::pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
        }
        currentNode = currentNode->parent;

    }

    return cost + distanceCost;
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
            if (distances(currentKiddo->cell.x, currentKiddo->cell.y) > params.minDistanceToObstacle) {
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

