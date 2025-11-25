#pragma once


#define MAP_SIZE 16

typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
} Orientation;

typedef struct MapNode {
    int distance = 0;
    MapNode *connected_nodes[4] = {0, 0, 0, 0};
    bool mapped = false;
} MapNode;

typedef struct {
    MapNode *parent_node = 0;
    MapNode *connected_node = 0;
    Orientation connection_orientation;
} NodeConnection;

typedef struct {
    NodeConnection connections[4];
    int connections_count = 0;
} NodeConnections;

void update_map_viz(MapNode map[MAP_SIZE][MAP_SIZE]);

void generate_map(MapNode map_in[MAP_SIZE][MAP_SIZE]);

void propogate_race_distances(MapNode map[MAP_SIZE][MAP_SIZE]);

void propogate_rehome_distances(MapNode map[MAP_SIZE][MAP_SIZE]);

void map_node(MapNode *node);

void sever_path(MapNode *node, Orientation absolute_wall_orientation);

void update_node(MapNode *node);

NodeConnection get_most_optimal_node_connection(MapNode *node, bool request_unmapped, bool randomize_search);

NodeConnections get_most_optimal_node_connections(MapNode *node);

double percent_mapped(MapNode map[MAP_SIZE][MAP_SIZE]);
