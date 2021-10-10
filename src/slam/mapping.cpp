#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_){
        previousPose_ = pose;
        initialized_ = true;
    }
    // a vector<adjusted_ray_t>;
    //                         scan, beginPose,   endPose,  rayStride = 1;[number of ray skip in the original scan]
    MovingLaserScan movingscan(scan, previousPose_, pose);
    // each ray has its own origin

    // go through each ray
    for(auto ray : movingscan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    previousPose_ = pose;

}


// score the endpoint and add the log odds
void Mapping:: scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range <= this->kMaxLaserDistance_ ){
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map); // src/common/grid_utils.hpp
        Point<int> rayCell;

        rayCell.x = static_cast<int>( (ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>( (ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);


        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}


// // score the whole ray
// void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){

//     // Decrease CellOdds for each rays

// }
void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map) {
  
    Point <float> rayStart = global_position_to_grid_cell(ray.origin, map);
    int x0 = static_cast<int>(rayStart.x);
    int y0 = static_cast<int>(rayStart.y);

    int x1 = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    int y1 = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = x0 < x1 ? 1 : -1; //for direction (go left / right)
    int sy = y0 < y1 ? 1 : -1; // go up / down

    int err = dx - dy; // >0

    int x = x0;
    int y = y0;

    while (x != x1 || y != y1) {
        if (map.isCellInGrid(x, y)) {
            decreaseCellOdds(x, y, map); // since free space: decrease odd. (increase: means occupied(gridvalue=1))
        }
        // dx - dy
        // e2 = 2dx - 2 dy
        // 2dx - 2dy >= -dy => 2dx - dy >= 0:
        //              dx-dy-dy => dx - 2dy
        //              
        int e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx; // x direction forward one unit
        }

        // 2dx - 2dy <= dx => dx - 2dy <= 0?
        // dx - dy +dx => 2dx - dy
        if (e2 <= dx) {
            err += dx;
            y += sy; // y direction forward one unit
        }
    }
}



void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){
    if( map(x,y) - std::numeric_limits<CellOdds>::min() > this->kHitOdds_ ){
         map(x,y) -= this->kHitOdds_;
    }
    else{
        // hit the max value of map
        map(x,y) = std::numeric_limits<CellOdds>::min();
    }   
    // map(x,y) -= this->kHitOdds_;
} // write output in map
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
   
    if(std::numeric_limits<CellOdds>::max() - map(x,y) > this->kHitOdds_ ){
        map(x,y) +=  this->kHitOdds_;
    }
    else{
        // hit the max value of map
        map(x,y) = std::numeric_limits<CellOdds>::max();
    }
}