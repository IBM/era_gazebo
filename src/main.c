#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "occgrid.h"

void initCostmap(unsigned int size_x, unsigned int size_y, double resolution, unsigned char default_value, double origin_x, double origin_y, double origin_z) {
    printf("Initialize Master Costmap\n");

    master_observation.rolling_window_ = true;
    master_observation.min_obstacle_height_ = 0.05;
    master_observation.max_obstacle_height_ = 2.0;
    master_observation.raytrace_range_ = 101.0;

    master_observation.master_costmap.size_x = size_x;
    master_observation.master_costmap.size_y = size_y;
    master_observation.master_costmap.default_value = default_value;

    master_observation.master_resolution = resolution;

    master_observation.master_origin.x = origin_x;
    master_observation.master_origin.y = origin_y;
    master_observation.master_origin.z = origin_z;

    for (int i = 0; i < master_observation.master_costmap.size_x * master_observation.master_costmap.size_y / (master_observation.master_resolution * master_observation.master_resolution); i++) {
        master_observation.master_costmap.costmap_[i] = master_observation.master_costmap.default_value;
    }
}

void addStaticObstacle(unsigned char* obstacle_type) {
    int cell_size_x = master_observation.master_costmap.size_x / master_observation.master_resolution;
    int cell_size_y = master_observation.master_costmap.size_y / master_observation.master_resolution;
    if (obstacle_type == "border") {
        printf("Add Static Obstacle: Wall Border \n");
        for (int i = 0; i < cell_size_x; i++) {
            for (int j = 0; j < cell_size_y; j++) {
                int index = cell_size_x * j + i;
                if (i == (int) master_observation.master_origin.x || i == cell_size_x - 1) master_observation.master_costmap.costmap_[index] = 1;
                else if (j == (int) master_observation.master_origin.y || j == cell_size_y - 1 ) master_observation.master_costmap.costmap_[index] = 1;
            }
        }
    }
}

void printMap() {
    printf("map: \n");
    for (int i = 0; i < master_observation.master_costmap.size_y / master_observation.master_resolution; i++) {
        for (int j = 0; j < master_observation.master_costmap.size_x / master_observation.master_resolution; j++) {
            int index = i * master_observation.master_costmap.size_y / master_observation.master_resolution + j;
            printf("%4d", master_observation.master_costmap.costmap_[index]);
        }
        printf("\n\n");
    }
}

void parseArray(char* array, int size) {
    int front_bracket_index;

    int space_index;
    bool first_comma_detected = false;

    int count = 0;
    for (int i = 0; i < size; i++) {
        if (array[i] == '[') front_bracket_index = i;

        else if (array[i] == ' ') space_index = i;

        else if (array[i] == ',' || array[i] == ']') {
            int diff;
            if (first_comma_detected) {
                diff = i - space_index;
            }
            else diff = i - front_bracket_index;
            int x;
            int value = 0;
            first_comma_detected = true;

            for (int j = 1; j < diff; j++) {
                //printf("%c ... ", array[i - count + j]);
                int dec = pow(10, (diff - j - 1));
                x = array[i - diff + j] - 48; //Convert from ASCII decimal to decimal
                //printf("%d * %d, ", dec, x);
                value = value + x * dec;
            }

            //printf("Value (%d) = %c\n", count, value);
            data[count + 1] = value;
            count++;
            //printf("Value -> %c \n", data[count]);
            //printf("Value (%c) = %d\n", count - 1, value);
        }
    }
    //printf("Count = %d \n", count);
}

//TODO: Retrieve global parameters such as origin, resolution, etc.
int main()
{
    //Initialize costmap with default cost value
    initCostmap(100, 100, 2.0, NO_INFORMATION, 0.0, 0.0, 0.0);

    printMap();

    //Add a static obstacle along the outer borders of costmap window
    //addStaticObstacle("border");

    printMap();

    FILE *f = fopen("data_003.txt", "r");
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* cloud_data = malloc(fsize + 1);
    printf("fsize = %d \n", fsize);
    fread(cloud_data, 1, fsize, f);
    fclose(f);

    cloud_data[fsize] = 0;
    parseArray(cloud_data, fsize);

    /*printf("{");
    for(int i = 1; i < 175895; i++) {
        printf("%d, ", data[i]);
    }
    printf("}\n");
    //printf("%s \n", cloud_data);*/

    PointCloud2 cloud;
    cloud.point_step = 12;
    Odometry odom;
    odom.pose.pose.position.x = 2.0;
    odom.pose.pose.position.y = 2.0;


    printf("Number of Elements = %d\n", (int) sizeof(data) / sizeof(data[0]));
    printf("Number of Coordinates = %d\n", (int) sizeof(data) / sizeof(data[0]) / cloud.point_step);

    //char array[] = {249, 23, 154, 64, 124, 77, 139, 62, 27, 227, 27, 192, 121, 46, 154, 64, 174, 152, 88, 62, 27, 227, 27, 192};
    //char data[] = {146, 37, 39, 66, 155, 83, 83, 193, 52, 108, 173, 63, 143, 52, 44, 66, 165, 71, 99, 193, 16, 124, 179, 63, 155, 253, 42, 66, 10, 63, 107, 193, 208, 18, 179, 63};

    //Iterate through data array and convert to floats
    int num_coords = sizeof(data) / sizeof(data[0]) / cloud.point_step;
    printf("Number of Coordinates Imported -> %d \n", num_coords);
    //float new_data[num_coords * 3];
    for (int i = 0; i < num_coords; i++) {
        float f;
        for (int j = i * cloud.point_step; j < (i + 1) * cloud.point_step; j = j + sizeof(f)) {
            memcpy(&f, &data[j + 1], sizeof(f));
            cloud.data[j / 4] = f;
            //printf("Data: %f\n", f);
        }
        printf("Data Point -> <%f, %f, %f>\n", cloud.data[i*3], cloud.data[i*3 + 1], cloud.data[i*3 + 2]);
    }

    //printf("Number of elements : %d\n", sizeof(cloud.data) / sizeof(cloud.data[0]));


    updateMap(cloud, 0.0, 0.0, 0.0, odom);

    printMap();


    return 0;
}
