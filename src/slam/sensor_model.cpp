#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;
    //could check max and min range from manufacturing website
    for (auto& ray : movingScan) {
        
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), ray.origin.y + ray.range * std::sin(ray.theta));
        Point<double> rayEnd = global_position_to_grid_position(endpoint, map);
        
        Point<double> pastEndpoint(ray.origin.x + (ray.range + (0.05 )) * std::cos(ray.theta), ray.origin.y + (ray.range + (0.05 )) * std::sin(ray.theta));
        Point<double> past = global_position_to_grid_position(pastEndpoint, map);
        Point<double> beforeEndpoint(ray.origin.x + (ray.range - (0.05)) * std::cos(ray.theta), ray.origin.y + (ray.range - (0.05 )) * std::sin(ray.theta));
        Point<double> before = global_position_to_grid_position(beforeEndpoint, map);
        
        if (map.logOdds(rayEnd.x, rayEnd.y) > 0.0) {
            scanScore += 1.0;
        }
        
        else {
            float pastOdds =  map.logOdds(past.x, past.y);
            float beforeOdds = map.logOdds(before.x, before.y);
            if (pastOdds > 0.0) {
                 scanScore += 0.02 * pastOdds;
            }
            if (beforeOdds > 0.0) {
                 scanScore += 0.02 * beforeOdds;
            }
        }
        
        
        
    }

    return scanScore;
}
