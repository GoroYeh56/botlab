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
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), 
            ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayEnd = global_position_to_grid_position(endpoint, map);
        if (map.logOdds(rayEnd.x, rayEnd.y) > 0.0) {
            scanScore += 1.0;
        }
    }

    return scanScore;
}
