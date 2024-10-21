#include "FloodFill.h"
#include "API.h"

#include <cstdlib>
#include <iostream>
#include <math.h>

void update_map_viz(MapNode map[MAP_SIZE][MAP_SIZE]) {
    for (int y = MAP_SIZE - 1; y >= 0; y--) for (int x = 0; x < MAP_SIZE; x++)  {
        API::setText(x, y, std::to_string(map[x][y].distance));
    }
}

// Recursively go through all nodes in one of the 4 cardinal middirections (idk the real name lol)
void propogate_distances(int distance, MapNode *node, Orientation direction1, Orientation direction2) {
    node->distance = distance;
    distance++;

    MapNode *direction1_node = node->connected_nodes[direction1];
    MapNode *direction2_node = node->connected_nodes[direction2];

    if (direction1_node != NULL)
        propogate_distances(distance, direction1_node, direction1, direction2);
    
    if (direction2_node != NULL)
        propogate_distances(distance, direction2_node, direction1, direction2);
}

void generate_map(MapNode map[MAP_SIZE][MAP_SIZE]) {
    // Initialize connections between nodes
    // TODO: Could maybe collapse these two for loops into 1 for loop but for now this is fine
    for (int x = 0; x < MAP_SIZE; x++) for (int y = 0; y < MAP_SIZE; y++) {
        MapNode *node = &map[x][y];

        // Connect each node with its (up to) 4 surrounding nodes
        for (int orientation = 0; orientation < 4; orientation++) {
            switch (orientation) {
                case NORTH:
                    if (y < MAP_SIZE - 1)
                        node->connected_nodes[NORTH] = &map[x][y+1];
                    break;
                case EAST:
                    if (x < MAP_SIZE - 1)
                        node->connected_nodes[EAST] = &map[x+1][y];
                    break;
                case SOUTH:
                    if (y > 0)
                        node->connected_nodes[SOUTH] = &map[x][y-1];
                    break;
                case WEST:
                    if (x > 0)
                        node->connected_nodes[WEST] = &map[x-1][y];
                    break;
            }
        }
    }

    // Fill out distances
    for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) {
        MapNode node = map[MAP_SIZE / 2 - x][MAP_SIZE / 2 - y];
        propogate_distances(0, &node, y == 1 ? SOUTH : NORTH, x == 1 ? WEST : EAST);
    }
}

void sever_path(MapNode *node, Orientation absolute_wall_orientation) {
    // Grab ptr to will-be-severed node
    // TODO: Is this even needed lol
    MapNode *severed_node = node->connected_nodes[absolute_wall_orientation];

    // Sever node
    node->connected_nodes[absolute_wall_orientation] = NULL;

    // Update both nodes
    // TODO: Can prolly be made more efficient by not updating one of the two nodes if they aren't affected by this but maybe not
    update_node(node);
    update_node(severed_node);
}

void update_node(MapNode *node) {
    if (node == NULL) return;
    
    // Target nodes cannot be updated
    if (node->distance == 0) return;

    // Get the lowest distance from the surrounding nodes
    int lowest_distance = INT_MAX;
    for (int i = 0; i < 4; i++) {
        MapNode *connected_node = node->connected_nodes[i];
        if (connected_node != NULL) {
            int connected_node_distance = connected_node->distance;
            lowest_distance = connected_node_distance < lowest_distance ? connected_node_distance : lowest_distance;
        }
    }

    // Check to see if an update is actually needed
    int new_node_distance = lowest_distance + 1;
    if (new_node_distance == node->distance) return;

    // If an update is needed, update the current node and recursively update all surrounding nodes
    node->distance = new_node_distance;
    for (int i = 0; i < 4; i++) {
        MapNode *connected_node = node->connected_nodes[i];
        if (connected_node != NULL) update_node(connected_node);
    }
}

Orientation get_most_optimal_node_direction(MapNode *node) {
    // Get the lowest distance from the surrounding nodes
    int lowest_distance = INT_MAX;
    Orientation lowest_orientation = NORTH;
    for (int i = 0; i < 4; i++) {
        MapNode *connected_node = node->connected_nodes[i];
        if (connected_node != NULL) {
            int connected_node_distance = connected_node->distance;
            if (connected_node_distance < lowest_distance) {
                lowest_distance = connected_node_distance;
                lowest_orientation = static_cast<Orientation>(i);
            }
        }
    }

    return lowest_orientation;
}