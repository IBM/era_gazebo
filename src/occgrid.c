#include "occgrid.h"
#include <math.h>

/*
 *
 * TODO:
 *  1) Define global structs/variables
 *  2) Implement transformLaserScanToPointCloud() function
 *  3) Implement tf2_ros::Buffer tf for transform function
 * 
 */

void occgrid(const LaserScan& message) {
    struct PointCloud2 cloud;
    cloud.header = message->header;

    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
}

void updateMap() {
    // if we're using a rolling buffer costmap... we need to update the origin using the robot's pose
    if (rolling_window_) {
        double new_origin_x = robot_x - size_x_ / 2; //global variable size_x_ and size_y_
        double new_origin_y = robot_y - size_y_ / 2;
        costmap_.updateOrigin(new_origin_x, new_origin_y);
    }

    minx_ = miny_ = 1e30; //TODO: Where is minx_ and miny_ declared?
    maxx_ = maxy_ = -1e30;

    double prev_minx_ = minx_;
    double prev_miny_ = miny_;
    double prev_maxx_ = maxx_;
    double prev_maxy_ = maxy_;

    updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
}

void updateOrigin(struct Costmap2D costmap, double new_origin_x, double new_origin_) {
    //project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - costmap.origin_x_) / costmap.resolution_);
    cell_oy = int((new_origin_y - costmap.origin_y_) / costmap.resolution_);

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = costmap.origin_x_ + cell_ox * costmap.resolution_;
    new_grid_oy = costmap.origin_y_ + cell_oy * costmap.resolution_;

    // To save casting from unsigned int to int a bunch of times
    int size_x = costmap.size_x_;
    int size_y = costmap.size_y_;

    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    // we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = /*new*/ unsigned char[cell_size_x * cell_size_y]; //Uncomment if in C++

    // copy the local window in the costmap to the local map
    copyMapRegion(costmap.costmap_, lower_left_x, lower_left_y, costmap.size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    // now we'll set the costmap to be completely unknown if we track unknown space
    resetMaps();

    // update the origin with the appropriate world coordinates
    costmap.origin_x_ = new_grid_ox;
    costmap.origin_y_ = new_grid_oy;

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // now we want to copy the overlapping information back into the map, but in its new location
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap.costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
}

void copyMapRegion(unsigned char* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, unsigned char* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y) {
    // we'll first need to compute the starting points for each map
    unsigned char* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    unsigned char* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i){
        memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
        sm_index += sm_size_x;
        dm_index += dm_size_x;
    }
}

void resetMaps() {
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {

    //raytrace free space
    raytraceFreespace(observation, min_x, min_y, max_X, max_y);

    //Iterate through cloud to register obstacles within costmap
    for(unsigned int i = 0; i < cloud.data.size(); i++) {
        //Only consider points within height boundaries
        if (cloud.data[2] <= max_obstacle_height_ && cloud.data[2] >= min_obstacle_height_) {
            double px = cloud.data[i][0];
            double py = cloud.data[i][1];
            double pz = cloud.data[i][2];

            unsigned int mx, my;
            worldToMap(px, py, mx, my);

            unsigned int index = getIndex(mx, my);
            costmap_[index] = LETHAL_OBSTACLE;
            touch(px, py, min_x, min_y, max_x, max_y);
        }
    }
}

void raytraceFreespace(const Obseration& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y) {
    //TODO: Modify to remove Observation
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    const PointCloud2 &cloud = *(clearing_observation.cloud_);

    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0))
    {
        printf("The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.", ox, oy);
        return;
    }

    // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;

    touch(ox, oy, min_x, min_y, max_x, max_y);

    for (int i = 0; i < cloud.data.size(); i++) {
        double wx = cloud.data[i][0];
        double wy = cloud.data[i][1];

        // now we also need to make sure that the enpoint we're raytracing
        // to isn't off the costmap and scale if necessary
        double a = wx - ox;
        double b = wy - oy;

        if (wx < origin_x) {
            double t = (origin_x - ox) / a;
            wx = origin_x;
            wy = oy + b * t;
        }

        if (wy < origin_y) {
            double t = (origin_y - oy) / b;
            wx = ox + a * t;
            wy = origin_y;
        }

        // the maximum value to raytrace to is the end of the map
        if (wx > map_end_x)
        {
            double t = (map_end_x - ox) / a;
            wx = map_end_x - .001;
            wy = oy + b * t;
        }
        if (wy > map_end_y)
        {
            double t = (map_end_y - oy) / b;
            wx = ox + a * t;
            wy = map_end_y - .001;
        }

        // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
        unsigned int x1, y1;

        unsigned int cell_raytrace_range = cellDistance(<REFERENCE>.raytrace_range_); //TODO: Create an object that holds the raytrace_range info

        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(x0, y0, x1, y1, cell_raytrace_range);

        updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
}

bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
    if (wx < origin_x_ || wy < origin_y_) return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_) return true;

    return false;
}

unsigned int cellDistance(double world_dist) {
    double cells_dist = max (0.0, ceil(world_dist/resolution_)); 
    return (unsigned int) cells_dist;
}

void rayTraceLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UNIT_MAX) { //TODO: Just in case, check if right function / define UNIT_MAX
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;

    unsigned int offset = y0 * size_x_ + x0;

    // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = hypot(dx, dy); 
    double scale = (dist == 0.0) ? 1.0 : min(1.0, max_length / dist);

    // if x is dominan

    if (abs_dx >= abs_dy)
    {
        int error_y = abs_dx / 2;
        bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
    }

    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length) {
    unsigned int end = min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
        markCell(costmap_, FREE_SPACE, offset);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
            offset += offset_b;
            error_b -= abs_da;
        }
    }
    markCell(costmap_, FREE_SPACE, offset);
}

void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double* min_x, double* min_y, double* max_x, double* max_y) {
    double dx = wx-ox, dy = wy-oy;
    double full_distance = hypot(dx, dy);
    double scale = min(1.0, range / full_distance);
    double ex = ox + dx * scale, ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
}

void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y) {
    *min_x = min(x, *min_x);
    *min_y = min(y, *min_y);
    *max_x = max(x, *max_x);
    *max_y = max(y, *max_y);
}

/********************** Helper Functions ***********************/

void markCell(unsigned char* costmap, unsigned char value, unsigned int offset) {
    costmap[offset] = value;
}

double hypot(double x, double y) {
    return sqrt(x * x + y * y);
}

int max(int num1, int num2) {
    return (num1 > num2) ? num1 : num2;
}

int max(int num1, int num2) {
    return (num1 > num2) ? num2 : num1;
}

int sign (int x) {
    return x > 0 ? 1.0 : -1.0;
}

int abs(int x) {
    if (x < 0.0) return x * -1.0;
    else return x;
}