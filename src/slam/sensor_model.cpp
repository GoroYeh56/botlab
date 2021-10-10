#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

// TODO : optimize this
double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;

    for(auto &ray: movingScan){
        Point<double> endpoint(ray.origin.x+ray.range * std::cos(ray.theta),
                                ray.origin.y + ray.range*std::sin(ray.theta));

        float gridDimension = 0.05;// 5cm*5cm each grid size
        float fraction = 0.06;     // parameter to be tuned
        float hitIncrement = 3.0;
        
        Point<double> beforepoint(ray.origin.x+ (ray.range+gridDimension) * std::cos(ray.theta),
                                ray.origin.y + (ray.range+gridDimension)*std::sin(ray.theta));
        Point<double> afterpoint(ray.origin.x+ (ray.range-gridDimension) * std::cos(ray.theta),
                                ray.origin.y + (ray.range-gridDimension)*std::sin(ray.theta));

        auto rayEnd = global_position_to_grid_cell(endpoint, map);
        auto afterEnd = global_position_to_grid_cell(afterpoint, map);
        auto beforeEnd = global_position_to_grid_cell(beforepoint, map);


        if( map.logOdds(rayEnd.x, rayEnd.y)  > 0.0){
            // scanScore += 1;
            scanScore += hitIncrement;
        }
        // Increase the before and after grid along that ray if its logOdds >0.0
        else{
            double beforeOdds = map.logOdds(beforeEnd.x, beforeEnd.y);
            double afterOdds = map.logOdds(afterEnd.x, afterEnd.y);
            if(beforeOdds > 0.0){
                scanScore += fraction * beforeOdds;
            }
            if(afterOdds > 0.0){
                scanScore += fraction * afterOdds;
            }           
        }

    }
    return scanScore;
}
