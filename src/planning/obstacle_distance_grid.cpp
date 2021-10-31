#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map) {
    int width = map.widthInCells();
    int height = map.heightInCells();
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (map.logOdds(x, y) < 0) {
                distance(x, y) = -1;
            }
            else {
                distance(x, y) = 0;
            }
        }
    }

}

// Driver function: setDistance()

/* TODO: Implement an algorithm to mark 
    'the distance to the nearest obstacle' for every cell in the map. */
void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    initializedDistances(map);


    std::priority_queue<DistanceNode> searchQueue;

    // Calculate 'free grid's' distance to nearest obs
    // and Push free grids to searchQueue (later we can search)    

    // First step : expand 8-directions from obstacle grids
    enqueue_obstacle_cells(*this, searchQueue); 
    // Second step : This step is to expand from 'free' grids


    if(searchQueue.empty()){

        bool all_negative_one=true;
        int width = map.widthInCells(); // number of cells for "width" (how many cells in horizontal direction of the map)
        int height =map.heightInCells();
        // Note: row major (0, 1, 2, ...) => (0,0) (1,0) (2,0), ...
        cell_t cell;
        for(int y=0; y<height; y++){
            for(int x=0; x<width; ++x){
                if(distance(x,y) != -1)
                    all_negative_one = false;
            }
        }

        // all zero : test_filled 
        // all -1 : empty_path to (invalid goal)
        if(all_negative_one){
            // all -1 : free => overwrite to 10
            int width = map.widthInCells(); // number of cells for "width" (how many cells in horizontal direction of the map)
            int height =map.heightInCells();
            // Note: row major (0, 1, 2, ...) => (0,0) (1,0) (2,0), ...
            cell_t cell;
            for(int y=0; y<height; y++){
                for(int x=0; x<width; ++x){
                    // std::cout<< distance(x,y)<<" ";
                    distance(x,y) = 10;
                }
                // std::cout<<std::endl;
            }       
        }

    }
    else{

        while(!searchQueue.empty()){
            DistanceNode nextNode = searchQueue.top();
            searchQueue.pop();
            expand_node(nextNode, *this, searchQueue); 
        }

    }

}

/* 
    This function go through each grid in the map.
    If this grid is free => set its distance(x,y) to -1 (Later we need to calculate its distance to its nearest obstacle)
    If this grid is occupied(map.logOdds > 0) => Set its distance(x,y) to 0 
*/
void ObstacleDistanceGrid::initializedDistances(const OccupancyGrid& map){
    int width = map.widthInCells(); // number of cells for "width" (how many cells in horizontal direction of the map)
    int height =map.heightInCells();
    // Note: row major (0, 1, 2, ...) => (0,0) (1,0) (2,0), ...
    cell_t cell;
    for(int y=0; y<height; y++){
        for(int x=0; x<width; ++x){
            if(map.logOdds(x,y) < 0 ){
                // this cell is free => initialize cell(x,y) to -1
                distance(x,y) = -1;
            }
            else{
                distance(x,y) = 0; 
            }
        }
    }
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);

}

void ObstacleDistanceGrid::enqueue_obstacle_cells(ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue) {
    int width = grid.widthInCells();
    int height = grid.heightInCells();
    cell_t cell;
    for (cell.y = 0; cell.y < height; cell.y++) {
        for (cell.x = 0; cell.x < width; cell.x++) {
            if (distance(cell.x, cell.y) == 0) {
                expand_node(DistanceNode(cell, 0), grid, searchQueue);
            }
        }
    }
}


void ObstacleDistanceGrid::expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue) {
    
    /*
    const int xDeltas[8] = { 1, 1, 1, 0, 0, -1, -1, -1 };
    const int yDeltas[8] = { 0, -1, -1, -1, 1, 1, -1, 0 };
    const float distances[8] = { 1, std::sqrt(2), 1, std::sqrt(2), 1, std::sqrt(2), 1, std::sqrt(2) };
    */

    const int xDeltas[8] = { 1, 1, 1, 0, -1, -1, -1, 0 };
    const int yDeltas[8] = { -1, 0, 1, 1, 1, 0, -1, -1 };
    const float distances[8] = {std::sqrt(2), 1, std::sqrt(2), 1, std::sqrt(2), 1, std::sqrt(2), 1 };
    
    for (int i = 0; i < 8; i++) {
        cell_t adjacentCell(node.cell.x + xDeltas[i], node.cell.y + yDeltas[i]);
        if (grid.isCellInGrid(adjacentCell.x, adjacentCell.y)) {
            if (grid(adjacentCell.x, adjacentCell.y) == -1) {
                DistanceNode adjacentNode(adjacentCell, node.distance + distances[i]); //for diagonals needs to add sqrt(2)
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                searchQueue.push(adjacentNode);
            }
        }
    }
}


void ObstacleDistanceGrid::enqueue_obstacle_cells( ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue)
{
    // if the cell in the grid is an obstacle: add to searchQueue
    int width = grid.widthInCells();
    int height = grid.heightInCells();
    cell_t cell;

    for(cell.y=0; cell.y< height; cell.y++){
        for(cell.x= 0; cell.x<width; cell.x++){
            if(distance(cell.x, cell.y)==0){ 
                // If this is an 'obstacle cel' : expand from it and 
                            // set :cell(cell), :its.distance = 0
                expand_node( DistanceNode(cell, 0), grid , searchQueue);
            }
        }
    }

}


void ObstacleDistanceGrid::expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue){
    const int dx[8] = {0, 0, 1, -1,  1, 1, -1, -1};
    const int dy[8] = {1, -1, 0, 0, -1, 1, 1, -1};
    for(int i=0; i<8; i++){
        cell_t adjacentCell(node.cell.x+dx[i], node.cell.y + dy[i]);
        if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y)){
            if( grid(adjacentCell.x, adjacentCell.y) == -1 ){  
                // if the grid is free => Calculate its distance to the nearest obstsacle
                // Only free grid has a 'distance' to that obstacle. 
                // If grid(x,y)==0 : it IS AN OBSTACLE GRID and thus its distance == 0! 
                // 1.4 for diagonal
                float increment_d = 1;
                // float increment_d = (i>=4)?1.414 : 1;   // assume it has original distance. we 'expand' it so + increment_d
                DistanceNode adjacentNode(adjacentCell, node.distance + increment_d); //diagoonal way
                // Update grid(adj.x, adj.y) to adjNode.distance * grid.meters/cell => convert to realworld scale
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                searchQueue.push(adjacentNode); // push this free grid to 'searchQueue'
            }
        }
    }

}
