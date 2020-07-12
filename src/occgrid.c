#include "occgrid_transform.h"
#include "occgrid.h"

/**
 * 
 * OVERVIEW OF COSTMAP GENERATION :
 *      When the laserScan data is published to the 'occgrid' node, the function <laserScanCallback> is called. This function transforms
 * laserScan data to a pointCloud and then buffers the pointCloud to the observation buffer. Once the pointCloud is transformed to 
 * global frame and added to the queue, the map can be updated using updateMap(). This function calls updateBounds(), which resizes 
 * the frame, updates the current status of the globabl frame, raytraces freespace, ... , and assigns LETHAL_OBSTACLE to the 
 * specified 'cells'. Once the char array 'costap_' is updated, then updateCosts() (which calls updateWithOverwrite()) iterates through
 * the specifed occupancy grid and assigns the corresponding cost value from costap_.
 * 
 * TODO :
 *   (1) port "boost::shared_ptr<ObservationBuffer>"
 *   (2) port "tfs::TransformException"
 * 
 */ 

void occgrid(const LaserScan& message, const boost::shared_ptr<ObservationBuffer>& buffer) {
    PointCloud2 cloud; //Construct a similar struct
    cloud.header = message->header;

    try {
        projector_.transformLaserScanToPointClod(message->header.frame_if, *message, cloud, *tf_);
    }
    catch (tfs::TransformException &ex) {
        printf("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(), ex.what()); //TODO: Possible helper functions
        projector_.projectLaser(*message, cloud); //projector_ comes from laser_geometry::LaserProjection
    }

    buffer->bufferCloud(cloud);
}

void bufferCloud(const PointCloud1& cloud) {
    PointStamped global_origin;

    //create a new observation on the list to be populated
    observation_list_.push_front(Observation()); //TODO: implement helper function for push_front

    //check whether the origin frame has been set explicitly or whether we should get it from the cloud
    string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

    try {
        //given these observations come from sensor... we'll need to store the origin point of the sensor
        :PointStamped local_origin;
        local_origin.header.stamp = cloud.header.stamp;
        local_origin.header.frame_id = origin_frame;
        local_origin.point.x = 0;
        local_origin.point.y = 0;
        local_origin.point.z = 0;
        tf2_buffer_.transform(local_origin, global_origin, global_frame_);
        tf2::convert(global_origin.point, obseration_list_.front().origin_); //TODO: implement in transform_functions.c the function convert()

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;

    sensor_msgs::PointCloud2 global_frame_cloud;

    // transform the point cloud
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    PointCloud2& observation_cloud = *(observation_list_.front().cloud_); //TODO: implement helper function for front()
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height*global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud); //TODO: implement helper function for modifier
    modifier.resize(cloud_size); //TODO: implement helper function for resize()
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z"); //TODO: implement helper function for iter_z
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end(); //TODO: port vector::*iterator
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    for (; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step) {
      if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) {
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs); //TODO: implement HF for copy if not defined by standard C library
        iter_obs += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    // resize the cloud for the number of legal points
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }

  catch (TransformException& ex) {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front(); //TOD0: implement helper function
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what()); //TODO: check imported functions called here
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now(); //TODO: Use C library for time-updating

  // we'll also remove any stale observations from the list
  purgeStaleObservations(); 
}

void purgeStaleObservations() {
    if (!observartion_list_empty()) {
        list<Observation>::iterator obs_it = observation_list_.begin();
        //if we're keeping observation for no time... then we'll only keep one observation
        if (observation_keep_time_ == ros::Duration(0.0)) {
            observation_list_.erase(++obs_it, observation_list.end());
            return;
        }
        // otherwise... we'll have to loop through the observations to see which ones are stale

        for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it){
            Observation& obs = *obs_it;
            // check if the observation is out of date... and if it is, remove it and those that follow from the list
            if ((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_) {
                observation_list_.erase(obs_it, observation_list_.end());
                return;
            }
        }
    }
}

void updateMap(double robot_x, double robot_y, double robot_yaw) {
    //Lock for the remainder of this function, some plugins (e.g. VoxelLayer) implement threade unsafe updateBounds() functions.
    //boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex())); //Uncomment and port to C if multi-thread

    // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
    if (rolling_window_)
    {
        double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
        double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
        costmap_.updateOrigin(new_origin_x, new_origin_y);
    }

    if (plugins_.size() == 0) 
        return;

    minx_ = miny_ = 1e30;
    maxx_ = maxy_ = -1e30;

    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        double prev_minx = minx_;
        double prev_miny = miny_;
        double prev_maxx = maxx_;
        double prev_maxy = maxy_;
        (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
        if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
        {
            ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
        }
    }

    int x0, xn, y0, yn;
    costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
    costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

    x0 = std::max(0, x0);
    xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
    y0 = std::max(0, y0);
    yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

    ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

    if (xn < x0 || yn < y0)
        return;

    costmap_.resetMap(x0, y0, xn, yn);
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
    }

    bx0_ = x0;
    bxn_ = xn;
    by0_ = y0;
    byn_ = yn;

    initialized_ = true;
}

double getSizeInMetersX()
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double getSizeInMetersY()
{
  return (size_y_ - 1 + 0.5) * resolution_;

}

void updateOrigin(double new_origin_x, double new_origin_y) {
    // project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    // Nothing to update
    if (cell_ox == 0 && cell_oy == 0)
        return;

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    // To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;

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
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    // now we'll set the costmap to be completely unknown if we track unknown space
    resetMaps();

    // update the origin with the appropriate world coordinates
    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // now we want to copy the overlapping information back into the map, but in its new location
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

    // make sure to clean up
    /*delete[]*/ local_map; //Uncomment if C++
}

template<typename data_type>//TODO: Configure to plain C
void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y) {
    // we'll first need to compute the starting points for each map
    data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

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
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if (!enabled_)
        return;
    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    /*std::vector<Observation>*/ Observation observations[], clearing_observations[]; //Uncomment and change if C++

    // get the marking observations
    current = current && getMarkingObservations(observations);

    // get the clearing observations
    current = current && getClearingObservations(clearing_observations);

    // update the global current status
    current_ = current;

    // raytrace freespace
    for (unsigned int i = 0; i < clearing_observations.size(); ++i)
    {
        raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    }

    // place the new obstacles into a priority queue... each with a priority of zero to begin with
    for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    {
        const Observation& obs = *it;
        const PointCloud2& cloud = *(obs.cloud_);
        double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"); //TODO: port declaration type to C
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        // if the obstacle is too high or too far away from the robot we won't add it
        if (pz > max_obstacle_height_)
        {
            ROS_DEBUG("The point is too high");
            continue;
        }

        // compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) + (pz - obs.origin_.z) * (pz - obs.origin_.z);

        // if the point is far enough away... we won't consider it
        if (sq_dist >= sq_obstacle_range)
        {
            ROS_DEBUG("The point is too far away");
            continue;
        }

        // now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my))
        {
            ROS_DEBUG("Computing map coords failed");
            continue;
        }

        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
        touch(px, py, min_x, min_y, max_x, max_y);
    }
}

void useExtraBounds(doulbe* min_x, double* min_y, double* max_x, double* max_y) {
    if (!has_extra_bounds) return;

    *min_x = min(extra_min_x_, *min_x); //TODO: Helper function for min() and max()
    *min_y = min(extra_min_y_, *min_y);
    *max_x = max(extra_max_x_, *max_x);
    *max_y = max(extra_max_y_, *max_y);
    extra_min_x_ = 1e6;
    extra_min_y_ = 1e6;
    extra_max_x_ = -1e6;
    extra_max_y_ = -1e6;
    has_extra_bounds_ = false;
}

bool getMarkingObservations(Observation& marking_observations[]) {
    bool current = true;
    
    // get the marking observations
    for (unsigned int i = 0; i < marking_buffers_.size(); ++i) //TODO: remove lock/unlock calls
    {
        marking_buffers_[i]->lock();
        marking_buffers_[i]->getObservations(marking_observations);
        current = marking_buffers_[i]->isCurrent() && current;
        marking_buffers_[i]->unlock();
    }
    marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
    return current;
}


bool getClearingObservations(Observation& clearing_observations[]) {
    bool current = true;

    // get the clearing observations
    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) //TODO: Remove lock/unlock calls
    {
        //clearing_buffers_[i]->lock(); //Uncomment and update if multi-thread
        clearing_buffers_[i]->getObservations(clearing_observations);
        current = clearing_buffers_[i]->isCurrent() && current;
        //clearing_buffers_[i]->unlock(); //Uncomment and update if multi-thread
    }
    clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
    return current;
}

void getObservations(Observation& observations[]) {
    //first, let's make sure that we don't ave any stake observations
    purgeStaleObservations();

    //now we'll ust copy the observations for the caller
    list<Observation>::iterator obs_it;
    for (obs_it observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
        bservations.push_back(*obs_it);
    }
}

bool isCurrent() {
    if (expected_update_rate_ == ros::Duration(0.0)) return true;

    bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
    if (!current)
    {
        ROS_WARN(
            "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
            topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
    }
    return current;
}

void raytraceFreespace(const Obseration& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y) {
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    const PointCloud2 &cloud = *(clearing_observation.cloud_);

    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0))
    {
        ROS_WARN_THROTTLE(
            1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
            ox, oy);
        return;
    }

    // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;

    touch(ox, oy, min_x, min_y, max_x, max_y);

    // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
        double wx = *iter_x;
        double wy = *iter_y;

        // now we also need to make sure that the enpoint we're raytracing
        // to isn't off the costmap and scale if necessary
        double a = wx - ox;
        double b = wy - oy;

        // the minimum value to raytrace from is the origin
        if (wx < origin_x)
        {
            double t = (origin_x - ox) / a;
            wx = origin_x;
            wy = oy + b * t;
        }

        if (wy < origin_y)
        {
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

        // check for legality just in case
        if (!worldToMap(wx, wy, x1, y1))
        continue;

        unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
        MarkCell marker(costmap_, FREE_SPACE); //TODO: investigate the 'marker casting

        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

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

void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y) {
    *min_x = min(x, *min_x);
    *min_y = min(y, *min_y);
    *max_x = max(x, *max_x);
    *max_y = max(y, *max_y);
}

unsigned int cellDistance(double world_dist) {
    double cells_dist = max (0.0, ceil(world_dist/resolution_)); //TODO: import standard math library for ceil()
    return (unsigned int) cells_dist;
}

template<class ActionType> //TODO:
void rayTraceLine(ActionType at, unsigned int x0, unsgined int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UNIT_MAX) { //TODO: Just in case, check if right function / define UNIT_MAX
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;

    unsigned int offset = y0 * size_x_ + x0;

    // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = hypot(dx, dy); //TODO Helper function
    double scale = (dist == 0.0) ? 1.0 : min(1.0, max_length / dist);

    // if x is dominan

    if (abs_dx >= abs_dy)
    {
        int error_y = abs_dx / 2;
        bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
    }

    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

template<class ActionType> //TODO:
void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length) {
    unsigned int end = min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
        at(offset);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
            offset += offset_b;
            error_b -= abs_da;
        }
    }
    at(offset);
}

void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double* min_x, double* min_y, double* max_x, double* max_y) {
    double dx = wx-ox, dy = wy-oy;
    double full_distance = hypot(dx, dy);
    double scale = min(1.0, range / full_distance);
    double ex = ox + dx * scale, ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
}

void worldToMapEnforceBounds(double w, double wy, int& mx, int& my) {
    // Here we avoid doing any math to wx,wy before comparing them to
    // the bounds, so their values can go out to the max and min values
    // of double floating point.
    if (wx < origin_x_)
    {
        mx = 0;
    }
    else if (wx >= resolution_ * size_x_ + origin_x_)
    {
        mx = size_x_ - 1;
    }
    else
    {
        mx = (int)((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_)
    {
        my = 0;
    }
    else if (wy >= resolution_ * size_y_ + origin_y_)
    {
        my = size_y_ - 1;
    }
    else
    {
        my = (int)((wy - origin_y_) / resolution_);
    }
}

unsigned int getSizeInCellsX() {
    return size_x_;
}

unsigned int getSizeInCellsY() {
    return size_y_;
}

void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn) {
    unsigned int len = xn - x0;
    for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
        memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
    }
}

void updateCosts() {
    if (!enabled_) return;

    if (footprint_clearing_enabled_) {
        setConvexPolygonCost(transformed_footprint_, FREE_SPACE); //TODO: Define FREE_SPACE
    }

    switch (combination_method_) {
        case 0:  // Overwrite
            updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
            break;
        case 1:  // Maximum
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
            break;
        default:  // Nothing
            break;
    }
}

bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value) {
    // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
    MapLocation map_polygon[];
    for (unsigned int i = 0; i < polygon.size(); ++i) {
        MapLocation loc;
        if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
        {
            // ("Polygon lies outside map bounds, so we can't fill it");
            return false;
        }
        map_polygon.push_back(loc); //TODO: Helper function for push_back
    }

    MapLocation polygon_cells[];

    // get the cells that fill the polygon
    convexFillCells(map_polygon, polygon_cells);

    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i)
    {
        unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
        costmap_[index] = cost_value;
    }
    return true;
}

void convexFillCells(const MapLocation& polygon[], MapLocation& polygon_cells[]) {
    // we need a minimum polygon of a triangle
    if (polygon.size() < 3)
        return;

    // first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    // quick bubble sort to sort points by x
    MapLocation swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1)
    {
        if (polygon_cells[i].x > polygon_cells[i + 1].x)
        {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

        if (i > 0)
            --i;
        }
        else ++i;
    }

    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = polygon_cells[0].x;
    unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

    // walk through each column and mark cells inside the polygon
    for (unsigned int x = min_x; x <= max_x; ++x)
    {
        if (i >= polygon_cells.size() - 1)
            break;

        if (polygon_cells[i].y < polygon_cells[i + 1].y)
        {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        }
        else
        {
        min_pt = polygon_cells[i + 1];
        max_pt = polygon_cells[i];
        }

        i += 2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x)
        {
            if (polygon_cells[i].y < min_pt.y)
                min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y)
                max_pt = polygon_cells[i];
        ++i;
        }

        MapLocation pt;
        // loop though cells in the column
        for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
        {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}

void polygonOutlineCells(const MapLocation& polygon[], MapLocation& polygon_cells[]) {
    PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells); //TODO: investigate this type-cast caell_gatherer
    for (unsigned int i = 0; i < polygon.size() - 1; ++i)
    {
        raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
    }
    if (!polygon.empty())
    {
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
    }
}

void updateWithOverwrite(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) return;
    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    for (int j = min_j; j < max_j; j++) {
        unsigned int it = span*j+min_i;
        for (int i = min_i; i < max_i; i++) {
            if (costmap_[it] != NO_INFORMATION) master[it] = costmap_[it]; //TODO: NO_INFORMATION
            it++;
        }
    }
}

void updateWithMax(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) return;

    unsigned char* master_array = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    for (int j = min_j; j < max_j; j++)
    {
        unsigned int it = j * span + min_i;
        for (int i = min_i; i < max_i; i++)
        {
            if (costmap_[it] == NO_INFORMATION){
            it++;
            continue;
        }

        unsigned char old_cost = master_array[it];
        if (old_cost == NO_INFORMATION || old_cost < costmap_[it]) master_array[it] = costmap_[it];
        it++;
        }
    }
}

unsigned char* getCharMap() {
    return costmap_;
}