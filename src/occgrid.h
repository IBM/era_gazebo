#include <stdlib.h>

#ifndef OCCGRID_H_
#define OCCGRID_H_

struct Header {
    /* timestamp in the header is the acquisition time of 
    the first ray in the scan. */

    //TODO:
};

struct LaserScan {
    Header header,
    float angle_min,
    float angle_max,
    float angle_increment,
    float time_increment,
    float scan_time,
    float range_min,
    float range_max,
    float ranges[],
    float intensities[]
}; 

struct geometry_msgs_Point {
    double x,
    double y,
    double z
} typedef Point;

struct Pointfield {
    // This message holds the description of one point entry in the
    // PointCloud2 message format.
    unsigned short INT8    = 1,
    unsigned short UINT8   = 2,
    unsigned short INT16   = 3,
    unsigned short UINT16  = 4,
    unsigned short INT32   = 5,
    unsigned short UINT32  = 6,
    unsigned short FLOAT32 = 7,
    unsigned short FLOAT64 = 8,

    string name,              // Name of field
    unsigned int offset,      // Offset from start of point struct
    unsigned short datatype,  // Datatype enumeration, see above
    unsigned int count         // How many elements in the field   
};

struct sensor_msgs_PointCloud2 {
    Header header,
    unsigned int height,
    unsigned int width,
    Pointfield fields[],
    bool is_bigendian,
    unsigned int point_step,
    unsigned int row_step,
    unsigned short data[], //NOTE: Supposed to be int8 but does not exist in C
    bool is_dense
} typedef PointCloud2;

struct Observation {
    geometry_msgs_Point origin_,
    sensor_msgs_PointCloud2* cloud_,
    double obstacle_range_,
    double raytrace_range_
}

struct sensor_msgs_PointCloud1 {
    // This message holds a collection of 3d points, plus optional additional
    // information about each point.

    // Time of sensor data acquisition, coordinate frame ID.
    Header header,

    // Array of 3d points. Each Point32 should be interpreted as a 3d point
    // in the frame given in the header.
    Point32 points[],

    // Each channel should have the same number of elements as points array,
    // and the data in each channel should correspond 1:1 with each point.
    // Channel names in common practice are listed in ChannelFloat32.msg.
    ChannelFloat32 channels[]
} typedef PointCloud1;

struct geometry_msgs_Point32 {
    // This contains the position of a point in free space(with 32 bits of precision).
    // It is recommeded to use Point wherever possible instead of Point32.  
 
    // This recommendation is to promote interoperability.  

    // This message is designed to take up less space when sending
    // lots of points at once, as in the case of a PointCloud. 
    float x,
    float y,
    float z
} typedef Point32;

struct ChannelFloat32 {
    // The channel name should give semantics of the channel (e.g.
    // "intensity" instead of "value").
    string name,
    // The values array should be 1-1 with the elements of the associated
    // PointCloud.
    float values[]
}

struct sensor_msgs_PointStamped {
    // This represents a Point with reference coordinate frame and timestamp
    Header header,
    Point point
};

struct Costmap2D {
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned char* costmap_;
    unsigned char default_value_;
};

/**
 * @brief Handles buffering LaserScan messages
 * @param message The message returned from a message notifer
 * @param buffer A pointer to the obersvation buffer to update
 */
void occgrid(const LaserScan& message, const boost::shared_ptr<ObservationBuffer>& buffer); //TODO: Format second input parameter

/**
 * @brief Transforms a PointCloud to the global frame and buffers it
 * @param cloud The cloud to be buffered
 */ 
void bufferCloud(const PointCloud1& cloud);

/**
 * @brief Removes any stale observations from the buffer list
 */ 
void purgeStaleObservations()

/**
 * @brief Update the underlying costmap with new data
 * @param robot_x x coordinate of the robot's position
 * @param robot_y y coordinate of the robot's position
 * @param robot_yaw The rotational orientation of the robot
 */ 
void updateMap(double robot_x, double robot_y, double robot_yaw);

/**
 * @brief Move the origin of the costmap to a new locaton (keeping data when it can)
 * @param new_origin_x The x coordinate of the new origin
 * @param new_origin_y The y coordinate of the new origin
 */ 
void updateOrigin(double new_origin_x, double new_origin_y) 

/**
 * @brief  Copy a region of a source map into a destination map
 * @param  source_map The source map
 * @param sm_lower_left_x The lower left x point of the source map to start the copy
 * @param sm_lower_left_y The lower left y point of the source map to start the copy
 * @param sm_size_x The x size of the source map
 * @param  dest_map The destination map
 * @param dm_lower_left_x The lower left x point of the destination map to start the copy
 * @param dm_lower_left_y The lower left y point of the destination map to start the copy
 * @param dm_size_x The x size of the destination map
 * @param region_size_x The x size of the region to copy
 * @param region_size_y The y size of the region to copy
 */ 
void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y);

/**
 * @brief Resets the costmap and static_map to be unknown 
 */ 
void resetMaps();

/**
 * @brief 
 * @param robot_x x coordinate of the robot's position
 * @param robot_y y coordinate of the robot's position
 * @param robot_yaw The rotational orientation of the robot
 * @param min_x bounding box (input and output)
 * @param min_y bounding box (input and output)
 * @param max_x bounding box (input and output)
 * @param max_y bounding box (input and output)
 */ 
void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

/**
 * @brief Updates the bounding box specified in the parameters
 * to include the bounding box from the addExtraBounds
 * call. If addExtraBounds was not called, the method will do nothing.
 * Should be called at the beginning of the updateBounds method
 * @param min_x bounding box (input and output)
 * @param min_y bounding box (input and output)
 * @param max_x bounding box (input and output)
 * @param max_y bounding box (input and output)
 */ 
void useExtraBounds(doulbe* min_x, double* min_y, double* max_x, double* max_y)

/**
 * @brief  Get the observations used to mark space
 * @param marking_observations A reference to a vector that will be populated with the observations
 * @return True if all the observation buffers are current, false otherwise
 */ 
bool getMarkingObservations(std::vector<Observation>& marking_observations)

/**
 * @brief  Get the observations used to clear space
 * @param clearing_observations A reference to a vector that will be populated with the observations
 * @return True if all the observation buffers are current, false otherwise
 */ 
bool getClearingObservations(std::vector<Observation>& clearing_observations)

/**
 * @brief
 * @param
 */ 
#endif