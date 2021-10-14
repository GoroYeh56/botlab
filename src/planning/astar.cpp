#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    PriorityQueue openList;
    PriorityQueue closedList;
    openList.push(start);
    Node* goalNode;
    goalNode->cell.x = goal.x;
    goalNode->cell.y = goal.y;

    while (!openList.empty()) {
        Node* q = openList.pop();
        std::vector<Node*> kiddos = expand_node(q, distances, params);
        for (int i = 0; i < kiddos.size(); i++) {
            if (kiddos.at(i) == goalNode) {
                robot_path_t path;
                path.utime = start.utime;
                path.path = extract_pose_path(extract_node_path(kiddos.at(i)),distances);
                path.path_length = path.path.size();
                return path;
            }
            else {
                kiddos.at(i)->g_cost = q->g_cost + g_cost(q, kiddos.at(i), distances, params);
                kiddos.at(i)->h_cost = h_cost(kiddos.at(i), goalNode);
                if (!openList.is_member(kiddos.at(i)) && !closedList.is_member(kiddos.at(i) )){
                    openList.push(kiddos.at(i));
                }
            }
        }
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
    return std::sqrt((dx * dx) - (dy * dy));
}

// cost of the path up from one node to another
double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params) {
    
    double cost = 0.0;
    Node* currentNode = to;
    //add checking for params later!
    while (currentNode != from) {
        cost += ObstacleDistanceGrid(currentNode->cell.x, curentNode->cell.y);
        
        currentNode = currentNode->parent;
    }

}

std::vector<Node*> expand_node(Node* node, ObstacleDistanceGrid& distances, const SearchParams& params) {
    std::vector<Node*> kiddos;
    Node* currentKiddo = node;
    // This was super clever I can't believe I never thought of doing this before in this way
    const int xDeltas[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };
    const int yDeltas[8] = { 0, -1, -1, -1, 1, 1, -1, 0 };

    //add checking for params later!
    for (int i = 0; i < 8; i++) {
        currentKiddo->cell.x += xDeltas[i];
        currentKiddo->cell.y += yDeltas[i];
        currentKiddo->parent = node;
        kiddos.push_back(currentKiddo);
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
    pose_xyt_t currentPose;
    for (int i = 0; i < nodePath.size(); i++) {
        currentPose.x = nodePath.at(i)->x;
        currentPose.y = nodePath.at(i)->y;
        posePath.push_back(currentPose);
        // deal with currentPose.theta later not really used rn 
    }
    return posePath;
}

