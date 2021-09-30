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


// score the whole ray
void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){

    // Decrease CellOdds for each rays

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