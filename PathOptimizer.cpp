#include "PathOptimizer.h"
#include "API.h"
#include "FloodFill.h"

extern LoggingLevel logging_level;

void move_coord_in_direction(Orientation orientation, int *x, int *y) {
    int dx = 0;
    int dy = 0;
    switch (orientation) {
        case NORTH:
            dy = 1;
            break;
        case EAST:
            dx = 1;
            break;
        case SOUTH:
            dy = -1;
            break;
        case WEST:
            dx = -1;
            break;
    }
    *x += dx;
    *y += dy;
}

int iters = 0;

void reset_iters() {
    iters = 0;
}

int calculate_paths(MapNode map[MAP_SIZE][MAP_SIZE], MapNode *node, VersatilePathSegment *path_node, int x, int y) {
    iters++;
    if (iters > 200) return INT_MAX;

    bool straight_path_exists = false;
    do {
        if (node->distance == 0) {
            if (logging_level == DEBUG) std::cerr << iters << "  (" << x << ", " << y << ") CONCLUDED PATH" << std::endl;
            return 1;
        }

        // Used to ensure that we only keep adding to this path while a straight path exists, once we are forced to turn this path ends and it just calculates all subpaths
        straight_path_exists = false;

        NodeConnections ideal_connections = get_most_optimal_node_connections(node);
        if (logging_level == DEBUG) std::cerr << iters << "  (" << x << ", " << y << ")" << "  Got ideal connections " << ideal_connections.connections_count << std::endl;

        for (int i = 0; i < 4; i++) {
            if (logging_level == DEBUG) std::cerr << "      i: " << i << "  iters: " << iters << std::endl;
            NodeConnection connection = ideal_connections.connections[i];

            if (connection.parent_node == NULL) continue;

            if (logging_level == DEBUG) std::cerr << "          NOT NULL" << std::endl;

            // If one of the next ideal nodes is straight ahead just increase this path length and keep going
            if (connection.connection_orientation == path_node->movement_direction && ideal_connections.connections_count < 2) {
                path_node->distance++;
                node = connection.connected_node;
                straight_path_exists = true;
            } else {
                // Create next path node to follow
                VersatilePathSegment *next_path_node = (VersatilePathSegment*) malloc(sizeof(VersatilePathSegment));
                
                next_path_node->distance = 1;
                next_path_node->movement_direction = NORTH;
                next_path_node->turn_direction = EAST;

                for (int i = 0; i < 4; i++) {
                    next_path_node->cum_path_lengths[i] = INT_MAX;
                }

                next_path_node->next_path_nodes_length = 0;
                for (int i = 0; i < 4; i++) {
                    next_path_node->next_path_nodes[i] = NULL;
                }

                if (next_path_node == NULL) {
                    if (logging_level == DEBUG) std::cerr << "erm...." << std::endl;
                }
                next_path_node->movement_direction = connection.connection_orientation;

                path_node->next_path_nodes_length++;
                path_node->next_path_nodes[i] = next_path_node;

                // Recurse to generate sub paths
                
                int tx = x;
                int ty = y;
                for (int i = 1; i < path_node->distance; i++) {
                    move_coord_in_direction(path_node->movement_direction, &tx, &ty);
                }

                move_coord_in_direction(next_path_node->movement_direction, &tx, &ty);

                path_node->cum_path_lengths[i] = calculate_paths(map, connection.connected_node, next_path_node, tx, ty) + 1;
            }
        }
    } while (straight_path_exists);

    int lowest_cum_path_length = INT_MAX;
    for (int i = 0; i < 4; i++) {
        int current_cum_path_length = path_node->cum_path_lengths[i];
        if (current_cum_path_length < lowest_cum_path_length) {
            lowest_cum_path_length = current_cum_path_length;
        }
    }
    return lowest_cum_path_length;
}

RacePathSegment* get_ideal_path(VersatilePathSegment *versatile_path_segment) {
    bool is_final_segment = true;
    for (int i = 0; i < 4; i++) {
        if (versatile_path_segment->next_path_nodes[i] != NULL) is_final_segment = false;
    }

    if (is_final_segment) return NULL;

    int lowest_cum_path_length = INT_MAX;
    int lowest_index = 0;
    for (int i = 0; i < 4; i++) {
        int current_cum_path_length = versatile_path_segment->cum_path_lengths[i];
        if (current_cum_path_length < lowest_cum_path_length) {
            lowest_cum_path_length = current_cum_path_length;
            lowest_index = i;
        }
    }

    RacePathSegment *race_path_segment = (RacePathSegment *) malloc(sizeof(RacePathSegment));
    race_path_segment->distance = versatile_path_segment->distance;
    race_path_segment->movement_direction = versatile_path_segment->movement_direction;
    race_path_segment->turn_direction = versatile_path_segment->turn_direction;
    race_path_segment->next_path_segment = get_ideal_path(versatile_path_segment->next_path_nodes[lowest_index]);

    return race_path_segment;
}

void print_all_paths(VersatilePathSegment *path, int indents_count, int x, int y) {
    std::string indents(indents_count, ' ');
    if (logging_level == DEBUG) std::cerr << indents << indents << indents << "Distance: " << path->distance << " Currently Moving: " << path->movement_direction << " (" << path->next_path_nodes_length << " possible paths)" << std::endl;

    API::setColor(x, y, 'c');
    for (int i = 1; i < path->distance; i++) {
        move_coord_in_direction(path->movement_direction, &x, &y);

        API::setColor(x, y, 'c');
    }

    for (int i = 0; i < 4; i++) {
        VersatilePathSegment *next_path = path->next_path_nodes[i];

        if (next_path != NULL) {
            move_coord_in_direction(next_path->movement_direction, &x, &y);

            print_all_paths(next_path, indents_count + 1, x, y);
        }
    }
}

void print_ideal_path(RacePathSegment *path, int x, int y) {
    API::setColor(x, y, 'g');
    for (int i = 1; i < path->distance; i++) {
        move_coord_in_direction(path->movement_direction, &x, &y);

        API::setColor(x, y, 'g');
    }

    if (path->next_path_segment != NULL) {
        move_coord_in_direction(path->next_path_segment->movement_direction, &x, &y);

        print_ideal_path(path->next_path_segment, x, y);
    }
}
