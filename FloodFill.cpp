#include "FloodFill.h"
#include "API.h"

#include <cstdlib>
#include <iostream>
#include <math.h>

void update_map_viz(MapNode map[MAP_SIZE][MAP_SIZE]) {
    for (int y = MAP_SIZE - 1; y >= 0; y--) for (int x = 0; x < MAP_SIZE; x++)  {
        MapNode node = map[x][y];
        API::setText(x, y, node.mapped ? std::to_string(node.distance) : "?");
        //API::setText(x, y, std::to_string(map[x][y].connected_nodes[1] != NULL));
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

        //if ()
        //node->distance = abs(target_x - x) + abs(target_y - y);
    }
}

void propogate_race_distances(MapNode map[MAP_SIZE][MAP_SIZE]) {
    // Fill out distances
    for (int x = 0; x <= 1; x++) for (int y = 0; y <= 1; y++) {
        MapNode node = map[MAP_SIZE / 2 - x][MAP_SIZE / 2 - y];
        propogate_distances(0, &node, y == 1 ? SOUTH : NORTH, x == 1 ? WEST : EAST);
    }
}

void propogate_rehome_distances(MapNode map[MAP_SIZE][MAP_SIZE]) {
    MapNode node = map[0][0];
    propogate_distances(0, &node, NORTH, EAST);
}


void map_node(MapNode *node) {
    node->mapped = true;
}

void sever_path(MapNode *node, Orientation absolute_wall_orientation) {
    // Grab ptr to will-be-severed node
    // TODO: Is this even needed lol
    MapNode *severed_node = node->connected_nodes[absolute_wall_orientation];

    // Sever node
    //std::cerr << "Wall orientation: " << absolute_wall_orientation << " Inverse orientation: " << (absolute_wall_orientation + 2) % 4 << std::endl;
    node->connected_nodes[absolute_wall_orientation] = NULL;
    if (severed_node != NULL) severed_node->connected_nodes[(absolute_wall_orientation + 2) % 4] = NULL;

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

NodeConnection get_most_optimal_node_connection(MapNode *node, bool request_unmapped) {
    NodeConnection connection = NodeConnection();
    NodeConnection unmapped_connection = NodeConnection();
    // Get the lowest distance from the surrounding nodes
    int lowest_distance = INT_MAX;
    int lowest_unmapped_distance = INT_MAX;
    for (int i = 0; i < 4; i++) {
        MapNode *connected_node = node->connected_nodes[i];
        if (connected_node != NULL) {
            int connected_node_distance = connected_node->distance;
            if (connected_node_distance < lowest_distance) {
                lowest_distance = connected_node_distance;
                connection.connected_node = connected_node;
                connection.connection_orientation = static_cast<Orientation>(i);

                if (request_unmapped && !connected_node->mapped) {
                    lowest_unmapped_distance = connected_node_distance;
                    unmapped_connection.connected_node = connected_node;
                    unmapped_connection.connection_orientation = static_cast<Orientation>(i);
                }
            }
        }
    }

    // If an unmapped node is requested AND there is a surrounding unmapped node return that unmapped node
    return request_unmapped && unmapped_connection.connected_node != NULL ? unmapped_connection : connection;
}

NodeConnections get_most_optimal_node_connections(MapNode *node) {
    NodeConnections connections = NodeConnections();

    // Get the lowest distance from the surrounding nodes
    int lowest_distance = INT_MAX;
    for (int i = 0; i < 4; i++) {
        MapNode *connected_node = node->connected_nodes[i];
        if (connected_node != NULL) {
            int connected_node_distance = connected_node->distance;
            if (connected_node_distance <= lowest_distance && connected_node->mapped) {
                // Fill out and add connection
                NodeConnection connection = NodeConnection();
                lowest_distance = connected_node_distance;
                connection.parent_node = node;
                connection.connected_node = connected_node;
                connection.connection_orientation = static_cast<Orientation>(i);
                connections.connections[i] = connection;
                continue;
            }
        }

        // Initialize connections array to an empty node connection if no real connection was established
        connections.connections[i] = NodeConnection();
    }
    
    // Remove any that were added before the real lowest distance was set
    for (int i = 0; i < 4; i++) {
        NodeConnection connection = connections.connections[i];
        if (connection.connected_node != NULL) {
            // Prune connections
            if (connection.connected_node->distance > lowest_distance) {
                connections.connections[i] = NodeConnection();
                connections.connections_count--;
            } else {
                connections.connections_count++;
            }
        }
    }

    return connections;
}

double percent_mapped(MapNode map[MAP_SIZE][MAP_SIZE]) {
    int mapped_nodes = 0;
    for (int x = 0; x < MAP_SIZE; x++) for (int y = 0; y < MAP_SIZE; y++) map[x][y].mapped ? mapped_nodes ++ : mapped_nodes;

    return mapped_nodes * 1.0 / (MAP_SIZE * MAP_SIZE);
}