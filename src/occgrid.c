#include "occgrid.h"
#include <stdio.h>
#include <stdlib.h>

/*
 * Changes:
 *  (1) deleted and replaced instances of push_back with helper function push_back_MapLocation
 *  (2) replaced instances of std::vector with struct Vector
 *  (3) replaced instances of geometry_msgs::Point with struct Point
 *  (4) commented all instances of multithreading and ROS_ERROR/DEBUG
 *  (5) replaced instance(s) of 'voxel_grid::VoxelStatus' with 'enum VoxelStatus'
 *  (6) removed/commented instances of 'inline' from function definitions
 * 
 * To Dos:
 *  (1) Replace all instances of "polygon[index]" with "polygon.data[index]"
 *  (2) Finish Costmap2D constructor/initializer
 *  (3) Remove/Modify template function definitions with pointer typecasting (or possibly token pasting)
 */

static void mapToWorld3D(const unsigned int mx, const unsigned int my, const unsigned int mz,
                                      const double origin_x, const double origin_y, const double origin_z,
                                      const double x_resolution, const double y_resolution, const double z_resolution,
                                      double& wx, double& wy, double& wz)
{
    //returns the center point of the cell
    wx = origin_x + (mx + 0.5) * x_resolution;
    wy = origin_y + (my + 0.5) * y_resolution;
    wz = origin_z + (mz + 0.5) * z_resolution;
}

float g_colors_r[] = {0.0f, 0.0f, 1.0f};
float g_colors_g[] = {0.0f, 0.0f, 0.0f};
float g_colors_b[] = {0.0f, 1.0f, 0.0f};
float g_colors_a[] = {0.0f, 0.5f, 1.0f};

/*
//Initializes Costmap2D and returns its pointer
Costmap2D& Costmap2DInit(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x,
          double origin_y, unsigned char default_value) {
    //Initialize map
    size_x_ = cells_size_x;
    size_y_ = cells_size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    costmap_ = NULL;
    default_value_ = default_value;

    //access_ = new mutex_t(); //For multithreading

    //create the costmap
    initMaps(size_x_, size_y_);
    resetMaps();

    return 
}
*/

void deleteMaps() {
    //clean up data
    //boost::unique_lock<mutex_t> lock(*access_); //For multithreading
    free(costmap_);
    costmap_ = (unsigned char *) malloc(size_x * size_y * sizeof(unsigned char))); // CHECK: IMPLENTATION
}

void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
               double origin_x, double origin_y) {
    size_x_ = size_x;
    size_y_ = cells_size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    initMaps(size_x, size_y);

    //reset our maps to have no information
    resetMaps();                  
}

void resetMaps() {
    //boost::unique_lock<mutex_t> lock(*access_); //For multithreading
    memset(costmap_, defualt_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned nt yn) {
    unsigned int len = xn - x0;
    for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
        memset(costmap_ + y, default_value_, len * sizeof(unsigned char));    
    }
}

bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y,
                       double sin_size_x, double win_size_y) {
    //check for self windowing
    if (this == &map) {
        //ROS_ERROR("Cannot convert this costap into a window of itself");
        printf("Cannot convert this costmap into a window of itself\n");
        return false;
    }

    //clean up old data
    deleteMaps();

    //compute the bounds of our new map
    unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y)
        || !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y)) {
        //ROS_ERROR("Cannot window a map that the window bounds dont fit inside of")
        printf("Cannot window a map that the window bounds dont fit inside of\n")
        return false;
    }

    size_x_ = upper_right_x - lower_left_x;
    size_y_ = upper_right_y - lower_left_y;
    resolution_ = map.resolution_;
    origin_x_ = win_origin_x;
    origin_y_ = win_origin_y;

    // initialize our various maps and reset markers for inflation
    initMaps(size_x_, size_y_);

    // copy the window of the static map and the costmap that we're taking
    copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
    return true;
}

unsigned int cellDistance(double world_dist) {
    double cells_dist = max(0.0, ceil(world_dist / resolution_));
    return (unsigned int) cells_dist;
}

unsigned char* getCharMap() {
    return costmap_;
}

unsigned char getCost(unsgined int mx, unsigned int my) {
    return costmap_[getIndex(mx, my)];
}

void setCost(unsigned int mx, unsigned int my, unsigned char cost) {
    costmap_[getIndex(mx, my)] = cost;
}

void mapToWorld(unsigned int x, unsigned int my, double& wx, double& wy) {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
    if (wx < origin_x_ || wy < origin_y_) return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_) return true;

    return false;
}

void worldToMapNoBounds(double wx, double wy, int& mx, int& my) {
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
}

void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) {
    if (wx < origin_x_) mx = 0;
    else if (wx >= resolution_ * size_x_ + origin_x_) {
        mx = size_x_ - 1;
    }
    else {
        mx = (int)((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_) my = 0;
    else if (wy >= resolution_ * size_y_ + origin_y_) {
        my = size_y_ - 1;
    }
    else {
        my = (int)((wy - origin_y_) / resolution_);
    }
}

void updateOrigin(double new_origin_x, double new_origin_y) {
    //project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    // Nothing to update
    if (cell_ox == 0 && cell_oy == 0) return;

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

    //we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = malloc(sizeof(unsigned char) * cell_size_x * cel_size_y);

    //copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    //now we'll see the costmap to be completely unknown if we track unknown space
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
    free(local_map);
}

unsigned int getSizeInCellsX() {
  return size_x_;
}

unsigned int getSizeInCellsY() {
  return size_y_;
}

double getSizeInMetersX() {
  return (size_x_ - 1 + 0.5) * resolution_;
}

double getSizeInMetersY() {
  return (size_y_ - 1 + 0.5) * resolution_;
}

double getOriginX() {
  return origin_x_;
}

double getOriginY() {
  return origin_y_;
}

double getResolution() {
  return resolution_;
}

bool saveMap(string file_name) {
    FILE *fp = fopen(file_name.c_str(), "w");
    if (!fp) {
        return false;
    }

    fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
    for (unsigned int iy = 0; iy < size_y_; iy++) {
        for (unsigned int ix = 0; ix < size_x_; ix++) {
            unsigned char cost = getCost(ix, iy);
            fprintf(fp, "%d ", cost);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}

Costmap2D& operator=(const Costmap2D& map) {
    //check for self assignement
    if (this == &map) return *this;

    //clean up old data
    deleteMaps();

    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    resolution_ = map.resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;

    //initialize our various maps
    initMaps(size_x_, size_y_);

    //copy the cost map
    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    //CHECK IF THIS A DECLARED OR C++ FUNCTION

    return *this;
}

Costmap2D(const Costmap2D& map) {
    costmap_ = NULL;
    //access_ = new mute_t(); //For multithreading
    *this = map;
}

Costmap2D() {
    size_x_ = 0;
    size_y_ = 0;
    resolution = 0.0;
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    costmap_ = NULL;

    //access_ = new mutex_t(); //For multithreading
}

~Costmap2D() {
    deleteMaps();
    //free access_; //For multithreading
}

bool setConvexPolygonCost(const Vector& polygon, unsigned char cost_value) {
    // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
    Vector map_polygon;
    for (unsigned int i = 0; i < sizeof(polygon.data); ++i) {
        MapLocation loc;
        if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) {
            // ("Polygon lies outside map bounds, so we can't fill it");
            return false;
        }
    push_back_MapLocation(map_polygon, loc);
    }

    Vector polygon_cells;

    // get the cells that fill the polygon
    convexFillCells(map_polygon, polygon_cells);

    // set the cost of those cells
    for (unsigned int i = 0; i < sizeof(polygon_cells.data); ++i) {
        unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
        costmap_[index] = cost_value;
    }

    return true;
}

void polygonOutlineCells(const Vector polygon, Vector polygon_cells) {
    PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
    for (unsigned int i = 0; i < sizeof(polygon.data) - 1; ++i) {
        raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
    }
    if (!polygon.empty()) {
        unsigned int last_index = sizeof(polygon.data) - 1;
        // we also need to close the polygon by going from the last point to the first
        raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
    }
}

void convexFillCells(const Vector polygon, Vector polygon_cells) {
    // we need a minimum polygon of a triangle
    if (siseof(polygon.data) < 3) return;

    // first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    // quick bubble sort to sort points by x
    MapLocation swap;
    unsigned int i = 0;
    while (i < sizeof(polygon_cells.data) - 1) {
        if (polygon_cells[i].x > polygon_cells[i + 1].x) {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;
            if (i > 0) --i;
        }
        else ++i;
    }

    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = polygon_cells[0].x;
    unsigned int max_x = polygon_cells[sizeof(polygon_cells.data) - 1].x;

    // walk through each column and mark cells inside the polygon
    for (unsigned int x = min_x; x <= max_x; ++x) {
        if (i >= sizeof(polygon_cells.data) - 1) break;
        if (polygon_cells[i].y < polygon_cells[i + 1].y) {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        }
        else {
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }
        i += 2;

        while (i < polygon_cells.size() && polygon_cells[i].x == x) {
            if (polygon_cells[i].y < min_pt.y) min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y) max_pt = polygon_cells[i];
            ++i;
        }

        MapLocation pt;
        // loop though cells in the column
        for (unsigned int y = min_pt.y; y < max_pt.y; ++y) {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back_MapLocation(pt);
        }
    }
}

/*********************HELPER FUNCTIONS***********************/

//This assumes that vector is occupied by type 'MapLocation' points
bool push_back_MapLocation(struct Vector vec, struct MapLocation pt) {
    if (Vector.current_size == Vector.limit) {
        printf("Vector reached its size limit")
        return false;
    }
    else {
        Vector.data[Vector.current_size] = pt;
        Vector.current_size++;
        return true;
    }
}