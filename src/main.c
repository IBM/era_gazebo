#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "occgrid.h"

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
    printf("Count = %d \n", count);
}

int main()
{
    //Initialize input parameters data structures
    PointCloud2 cloud;
    cloud.point_step = 12;
    Odometry odom;
    odom.pose.pose.position.x = -45.094379425;
    odom.pose.pose.position.y = -50.2917823792;
    odom.pose.pose.position.z = 0.0357649326324;

    //Initialize costmap with default cost value
    bool rolling_window = true;
    double min_obstacle_height = 0.05;
    double max_obstacle_height = 2.0;
    double raytrace_range = 101.0;
    unsigned int size_x = 100;
    unsigned int size_y = 100;
    double resolution = 2.0;
    initCostmap(rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range,
                size_x, size_y, resolution, NO_INFORMATION, odom.pose.pose.position.x, odom.pose.pose.position.y,
                odom.pose.pose.position.z); //TODO: Include yaw data

    printMap();

    //Add a static obstacle along the outer borders of costmap window
    //addStaticObstacle("border");
    //printMap();

    //Retrieve data from text file
    printf("\nReading Map Data\n");
    FILE *f = fopen("data_001.txt", "r");
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* cloud_data = malloc(fsize + 1);
    //printf("fsize = %d \n", fsize);
    fread(cloud_data, 1, fsize, f);
    fclose(f);

    cloud_data[fsize] = 0;
    parseArray(cloud_data, fsize + 1);

    //printf("Number of Elements = %d\n", (int) sizeof(data) / sizeof(data[0]));
    printf("Number of Coordinates = %d\n", (int) sizeof(data) / sizeof(data[0]) / cloud.point_step);

    //Iterate through data array and convert to floats
    int num_coords = sizeof(data) / sizeof(data[0]) / cloud.point_step;
    //printf("Number of Coordinates Imported -> %d \n", num_coords);
    //float new_data[num_coords * 3];
    for (int i = 0; i < num_coords; i++) {
        float f;
        for (int j = i * cloud.point_step; j < (i + 1) * cloud.point_step; j = j + sizeof(f)) {
            memcpy(&f, &data[j + 1], sizeof(f));
            cloud.data[j / 4] = f;
            //printf("Data: %f\n", f);
        }
        //printf("Data Point -> <%f, %f, %f>\n", cloud.data[i*3], cloud.data[i*3 + 1], cloud.data[i*3 + 2]);
    }

    //Update map from pointcloud and robot's odometry
    updateMap(cloud, odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom);

    printMap();


    return 0;
}
