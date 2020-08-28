#include "occgrid.h"

unsigned char* combineGrids(unsigned char* source_map, unsigned char* dest_map, double robot_x1, double robot_y1,
                            double robot_x2, double robot_y2, unsigned int size_x, unsigned int size_y, double resolution, char def_val){
    //TODO: Input parameters needed: size of map, resolution, default value
    //TODO: Change size_x to cell_size_x
    //grid1 is previous map, grid2 is current map

    //Calculate the new origin of the map
    double new_origin_x = robot_x2 - (size_x - 1) / 2.0;
    double new_origin_y = robot_y2 - (size_y - 1) / 2.0;

    //Calculate the old origin of the map
    double origin_x = robot_x1 - (size_x - 1) / 2.0;
    double origin_y = robot_y1 - (size_y - 1) / 2.0;

    //Calculate the number of cells between the old and new origin
    unsigned int cell_ox = ((new_origin_x - origin_x) / resolution);
    unsigned int cell_oy = ((new_origin_y - origin_y) / resolution);

    //Determine the lower left cells of the origin
    unsigned int sm_lower_left_x = min(max(cell_ox, 0), size_x);
    unsigned int sm_lower_left_y = min(max(cell_oy, 0), size_y);
    unsigned int sm_upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    unsigned int sm_upper_right_y = min(max(cell_oy + size_y, 0), size_y);
    unsigned int dm_lower_left_x = sm_lower_left_x - cell_ox;
    unsigned int dm_lower_left_y = sm_lower_left_y - cell_oy;

    unsigned int cell_size_x = sm_upper_right_x - sm_lower_left_x;
    unsigned int cell_size_y = sm_upper_right_y - sm_lower_left_y;

    //Calculate the indexes of which to start 'copying' over
    unsigned char *sm_index = source_map + (sm_lower_left_y * size_x + sm_lower_left_x);
    unsigned char *dm_index = dest_map + (dm_lower_left_y * size_x + dm_lower_left_x);

    unsigned int region_size_x = cell_size_x;
    unsigned int region_size_y = cell_size_y;

    /*printf("Lower Left/Upper Right of Old Map = (%d, %d) / (%d, %d) \n", sm_lower_left_x, sm_lower_left_y, sm_upper_right_x, sm_upper_right_y);
    printf("Lower Left of New Map = (%d, %d) \n", dm_lower_left_x, dm_lower_left_y);
    printf("Cell Size = (%d, %d) \n", cell_size_x, cell_size_y);
    printf("Index of Old Map, Index of New Map = %d, %d \n", sm_index, dm_index);
    printf("Dimensions of Overlapping Region = (%d, %d) \n", region_size_x, region_size_y);*/

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i)
    {
        //memcpy((dm_index), (sm_index), sizeof(unsigned char) * region_size_x);
        for (unsigned int j = 0; j < region_size_x; ++j) {
            if (*(sm_index + j) < *(dm_index + j)) *(dm_index + j) = *(sm_index + j);
        }
        //This assumes that both maps have the same size
        sm_index += size_x;
        dm_index += size_x;
    }

    return dest_map;
}

