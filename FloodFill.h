#pragma once

#include <cstdlib>

#define MAP_SIZE 16

typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
} Orientation;

typedef struct MapNode {
    int distance = 0;
    MapNode *connected_nodes[4] = {NULL, NULL, NULL, NULL};
} MapNode;

void update_map_viz(MapNode map[MAP_SIZE][MAP_SIZE]);

void generate_map(MapNode map_in[MAP_SIZE][MAP_SIZE]);

void sever_path(MapNode *node, Orientation absolute_wall_orientation);

void update_node(MapNode *node);

Orientation get_most_optimal_node_direction(MapNode *node);