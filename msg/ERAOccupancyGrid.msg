# This represents a custom OccupancyGrid message for map exchanging in ERA.

Header header 

#MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
