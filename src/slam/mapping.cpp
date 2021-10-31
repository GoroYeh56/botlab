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
    if (!initialized_) {
        previousPose_ = pose;
    }
    
    MovingLaserScan movingScan(scan, previousPose_, pose);
    for (auto& ray : movingScan) {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }
    initialized_ = true;
    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map) {
    
    if (ray.range < kMaxLaserDistance_) {
        Point <float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if (map.isCellInGrid(rayCell.x, rayCell.y)) {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map) {
  
    Point <float> rayStart = global_position_to_grid_cell(ray.origin, map);
    int x0 = static_cast<int>(rayStart.x);
    int y0 = static_cast<int>(rayStart.y);

    int x1 = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    int y1 = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;

    int err = dx - dy;

    int x = x0;
    int y = y0;
    float obstacle_threshold = 80.0;

    while (x != x1 || y != y1) {
        if (map.isCellInGrid(x, y) && map(x,y) <= obstacle_threshold) {
            decreaseCellOdds(x, y, map);
        }

        int e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }

}


void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (map(x, y) - kMissOdds_ > std::numeric_limits<CellOdds>::min()) {
        map(x, y) -= kMissOdds_;
    }
    else {
        map(x, y) = std::numeric_limits<CellOdds>::min();
    }
}
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    
    if (map(x, y) + kHitOdds_ < std::numeric_limits<CellOdds>::max()) {
        map(x, y) += kHitOdds_;
    }
    else {
        map(x, y) = std::numeric_limits<CellOdds>::max();
    }

    
}

