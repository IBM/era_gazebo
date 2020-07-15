#include <stdlib.h>
#include <time.h>
#include <math.h>

#ifndef OCCGRID_H_
#define OCCGRID_H_

//Define char values for costmap values
#define LETHAL_SPACE 254
#define FREE_SPACE 0
#define NO_INFORMATION 255

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
void useExtraBounds(doulbe* min_x, double* min_y, double* max_x, double* max_y);

/**
 * @brief Clear freespace based on ne observation
 * @param clearing_observation The observation used to raytrace
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y
 */ 
void rayTraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y);

/**
 * @brief Convert from world coordinates to map coordinates
 * @param  wx The x world coordinate
 * @param  wy The y world coordinate
 * @param  mx Will be set to the associated map x coordinate
 * @param  my Will be set to the associated map y coordinate
 * @return True if the conversion was successful (legal bounds) false otherwise
 */ 
bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

/**
 * @brief Updates the bounding box specified in the parameters to include the location (x,y)
 * @param x x-coordinate to include
 * @param y y-coordinate to include
 * @param min_x bounding box
 * @param min_y bounding box
 * @param max_x bounding box
 * @param max_y bounding box
 */ 
void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

/**
 * @brief Given distance in the world... convert it to cells
 * @param  world_dist The world distance
 * @return The equivalent cell distance
 */ 
unsigned int cellDistance(double world_dist);

/**
 * @brief Raytrace a line and apply some action at each step
 * @param  at The action to take... a functor
 * @param  x0 The starting x coordinate
 * @param  y0 The starting y coordinate
 * @param  x1 The ending x coordinate
 * @param  y1 The ending y coordinate
 * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
 */ 
void rayTraceLine(ActionType at, unsigned int x0, unsgined int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UNIT_MAX);

/**
 * @brief A 2D implementiation of Bresenham's raytracing algorithm (applies an at action at each step)
 * @param at
 * @param abs_da
 * @param abs_db 
 * @param error_b
 * @param offset_a
 * @param offset_b
 * @param offset
 * @param max_length
 */ 
void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length)

/**
 * @brief 
 * @param ox
 * @param oy
 * @param wx
 * @param wy
 * @param range
 * @param min_x
 * @param min_y
 * @param max_x
 * @param max_y
 */ 
void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double* min_x, double* min_y, double* max_x, double* max_y);

/**
 * @brief Convert from world coordinates to map coordinates, constraining results to legal bounds.
 * @param  wx The x world coordinate
 * @param  wy The y world coordinate
 * @param  mx Will be set to the associated map x coordinate
 * @param  my Will be set to the associated map y coordinate
 * @note   The returned map coordinates are guaranteed to lie within the map.
 */ 
void worldToMapEnforceBounds(double w, double wy, int& mx, int& my);

#endif
