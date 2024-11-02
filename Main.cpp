#include "FloodFill.h"
#include "API.h"

#include <iostream>
#include <string>
#include <stack>
#include <math.h>

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

typedef struct {
    int x = 0;
    int y = 0;
} Position;

typedef enum {
    SEARCHING,
    REHOMING,
    RUNNING,
} State;

typedef struct VersatilePathSegment {
    int distance = 1;
    Orientation movement_direction = NORTH;
    Orientation turn_direction = EAST;

    int cum_path_lengths[4] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX};

    int next_path_nodes_length = 0;
    VersatilePathSegment *next_path_nodes[4] = {NULL, NULL, NULL, NULL};
} VersatilePathSegment;

typedef struct RacePathSegment {
    int distance = 1;
    Orientation movement_direction = NORTH;
    Orientation turn_direction = EAST;

    RacePathSegment *next_path_segment;
} RacePathSegment;

VersatilePathSegment path = VersatilePathSegment();

State state = SEARCHING;
int times_searched = 1;

MapNode race_map[MAP_SIZE][MAP_SIZE];
MapNode rehome_map[MAP_SIZE][MAP_SIZE];

Position last_position = Position();
Position position =  Position();
Orientation orientation = NORTH;

Orientation rotate_orientation_left(Orientation orientation) {
    if (orientation == NORTH) {
        orientation = WEST;
        return orientation;
    }

    orientation = static_cast<Orientation>(orientation - 1);

    return orientation;
}

Orientation rotate_orientation_right(Orientation orientation) {
    if (orientation == WEST) {
        orientation = NORTH;
        return orientation;
    }

    orientation = static_cast<Orientation>(orientation + 1);
    
    return orientation;
}

void turnLeft() {
    API::turnLeft();

    orientation = rotate_orientation_left(orientation);
}

void turnRight() {
    API::turnRight();

    orientation = rotate_orientation_right(orientation);
}

void moveForward() {
    API::moveForward(1);

    last_position = position;

    switch (orientation) {
        case NORTH:
            position.y++;
            break;
        case EAST:
            position.x++;
            break;
        case SOUTH:
            position.y--;
            break;
        case WEST:
            position.x--;
            break;
    }
}

void move_to_next_node(MapNode map[MAP_SIZE][MAP_SIZE], bool attempt_unmapped) {
    //update_map_viz(map);
    update_map_viz(race_map);
    Orientation target_dir = get_most_optimal_node_connection(&map[position.x][position.y], attempt_unmapped).connection_orientation;

    int delta_theta = orientation - target_dir; // + means turn left | - means turn right

    if (abs(delta_theta) > 2) delta_theta = signbit(delta_theta) ? 1 : -1;

    //std::cerr << "Target Dir: " << target_dir << "     Orientation: " << orientation << "     Delta Theta: " << delta_theta << std::endl; 

    for (int i = 0; i < delta_theta; i++) turnLeft();
    for (int i = 0; i > delta_theta; i--) turnRight();

    moveForward();
}

char orientation_to_char(Orientation orientation) {
    switch (orientation) {
        case NORTH:
            return 'n';
        case EAST:
            return 'e';
        case SOUTH:
            return 's';
        case WEST:
            return 'w';
    }
    return 'n';
}

bool should_not_explore() {
    return percent_mapped(race_map) > 0.1 || times_searched > 2;
}

int iters = 0;

int calculate_paths(MapNode map[MAP_SIZE][MAP_SIZE], MapNode *node, VersatilePathSegment *path_node, int x, int y) {
    iters++;
    if (iters > 200) return INT_MAX;

    bool straight_path_exists = false;
    do {
        if (node->distance == 0) {
            std::cerr << iters << "  (" << x << ", " << y << ") CONCLUDED PATH" << std::endl;
            return 1;
        }

        // Used to ensure that we only keep adding to this path while a straight path exists, once we are forced to turn this path ends and it just calculates all subpaths
        straight_path_exists = false;

        NodeConnections ideal_connections = get_most_optimal_node_connections(node);
        std::cerr << iters << "  (" << x << ", " << y << ")" << "  Got ideal connections " << ideal_connections.connections_count << std::endl;
        
        /*API::setColor(x, y, 'c');
        API::setText(x, y, std::to_string(ideal_connections.connections_count));
        //API::setText(x, y, std::to_string(ideal_connections.connections[0].connection_orientation));
        //API::setText(x, y, std::to_string(path_node->movement_direction));*/

        for (int i = 0; i < 4; i++) {
            std::cerr << "      i: " << i << "  iters: " << iters << std::endl;
            NodeConnection connection = ideal_connections.connections[i];

            if (connection.parent_node == NULL) continue;

            std::cerr << "          NOT NULL" << std::endl;

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
                    std::cerr << "erm...." << std::endl;
                }
                //std::cerr << "Distance: " << next_path_node.distance << " Direction: " << next_path_node.movement_direction << " Connecting Nodes: " << next_path_node.next_path_nodes_length << std::endl;
                next_path_node->movement_direction = connection.connection_orientation;

                path_node->next_path_nodes_length++;
                path_node->next_path_nodes[i] = next_path_node;

                // Recurse to generate sub paths
                
                int tx = x;
                int ty = y;
                for (int i = 1; i < path_node->distance; i++) {
                    int dx = 0;
                    int dy = 0;
                    switch (path_node->movement_direction) {
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
                    tx += dx;
                    ty += dy;
                }

                int dx = 0;
                int dy = 0;
                switch (next_path_node->movement_direction) {
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
                tx += dx;
                ty += dy;

                path_node->cum_path_lengths[i] = calculate_paths(map, connection.connected_node, next_path_node, tx, ty) + 1;
            }
        }
    } while (straight_path_exists/* && iters < 3*/);

    int lowest_cum_path_length = INT_MAX;
    for (int i = 0; i < 4; i++) {
        int current_cum_path_length = path_node->cum_path_lengths[i];
        if (current_cum_path_length < lowest_cum_path_length) {
            lowest_cum_path_length = current_cum_path_length;
        }
    }
    //API::setText(x,y, std::to_string(lowest_cum_path_length));
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
    std::cerr << indents << indents << indents << "Distance: " << path->distance << " Currently Moving: " << path->movement_direction << " (" << path->next_path_nodes_length << " possible paths)" << std::endl;

    //if (indents_count > 4) return;

    //API::setText(x,y, std::to_string(path->distance));
    //API::setText(x,y, std::to_string(path->next_path_nodes_length));
    /*int lowest_cum_path_length = INT_MAX;
    for (int i = 0; i < 4; i++) {
        int current_cum_path_length = path->cum_path_lengths[i];
        if (path->cum_path_lengths[i] < lowest_cum_path_length)
            lowest_cum_path_length = current_cum_path_length;
    }
    API::setText(x,y, std::to_string(lowest_cum_path_length));*/
    API::setColor(x, y, 'c');
    for (int i = 1; i < path->distance; i++) {
        int dx = 0;
        int dy = 0;
        switch (path->movement_direction) {
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
        x += dx;
        y += dy;

        //API::setText(x,y, std::to_string(path->distance));
        //API::setText(x,y, std::to_string(path->next_path_nodes_length));

        API::setColor(x, y, 'c');
        /*if (path->next_path_nodes_length > 1)
            API::setColor(x, y, 'g');
        else
            API::setColor(x, y, 'c');*/
    }

    for (int i = 0; i < 4; i++) {
        VersatilePathSegment *next_path = path->next_path_nodes[i];

        if (next_path != NULL) {

            int dx = 0;
            int dy = 0;
            switch (next_path->movement_direction) {
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
            x += dx;
            y += dy;

            print_all_paths(next_path, indents_count + 1, x, y);
        }
    }
}

void print_ideal_path(RacePathSegment *path, int x, int y) {
    API::setColor(x, y, 'g');
    for (int i = 1; i < path->distance; i++) {
        int dx = 0;
        int dy = 0;
        switch (path->movement_direction) {
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
        x += dx;
        y += dy;

        //API::setText(x,y, std::to_string(path->distance));
        //API::setText(x,y, std::to_string(path->next_path_nodes_length));

        API::setColor(x, y, 'g');
        /*if (path->next_path_nodes_length > 1)
            API::setColor(x, y, 'g');
        else
            API::setColor(x, y, 'c');*/
    }

    if (path->next_path_segment != NULL) {

        int dx = 0;
        int dy = 0;
        switch (path->next_path_segment->movement_direction) {
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
        x += dx;
        y += dy;

        print_ideal_path(path->next_path_segment, x, y);
    }
}

int main(int argc, char* argv[]) {
    generate_map(race_map);
    propogate_race_distances(race_map);
    generate_map(rehome_map);
    propogate_rehome_distances(rehome_map);

    update_map_viz(race_map);

    while (true) {
        std::cerr << "STATE: " << state <<  "   (" << position.x << ", " << position.y << ")   Percent mapped: " << percent_mapped(race_map) * 100 << "\%   Should Explore: " << !should_not_explore() << std::endl;

        if (API::wallLeft()) {
            Orientation rotated_orientation = rotate_orientation_left(orientation);
            API::setWall(position.x, position.y, orientation_to_char(rotated_orientation));
            sever_path(&race_map[position.x][position.y], rotated_orientation);
            sever_path(&rehome_map[position.x][position.y], rotated_orientation);
        }
        if (API::wallRight()) {
            Orientation rotated_orientation = rotate_orientation_right(orientation);
            API::setWall(position.x, position.y, orientation_to_char(rotated_orientation));
            sever_path(&race_map[position.x][position.y], rotated_orientation);
            sever_path(&rehome_map[position.x][position.y], rotated_orientation);
        }
        if (API::wallFront()) {
            API::setWall(position.x, position.y, orientation_to_char(orientation));
            sever_path(&race_map[position.x][position.y], orientation);
            sever_path(&rehome_map[position.x][position.y], orientation);
        }

        map_node(&race_map[position.x][position.y]);
        map_node(&rehome_map[position.x][position.y]);

        //std::cerr << "   Final distance: " << race_map[position.x][position.y].distance << std::endl;

        Orientation target_dir;

        switch (state) {
            case SEARCHING:
                if (race_map[position.x][position.y].distance == 0) {
                    while (!API::wasReset()) {}

                    state = REHOMING;
                    break;
                }
                move_to_next_node(race_map, true);
                break;
            case REHOMING:
                if (rehome_map[position.x][position.y].distance == 0) {
                    if (should_not_explore()) {
                        state = RUNNING;
                        path = VersatilePathSegment();
                        int best_path_length = calculate_paths(race_map, &race_map[0][0], &path, 0, 0);
                        iters = 0;

                        if (best_path_length == INT_MAX) {
                            state = SEARCHING;
                        } else {
                            RacePathSegment *ideal_path = get_ideal_path(&path);

                            API::clearAllColor();

                            print_all_paths(&path, 0, 0, 0);
                            print_ideal_path(ideal_path, 0, 0);
                        }
                    } else {
                        state = SEARCHING;
                        times_searched++;
                    }
                    break;
                }
                move_to_next_node(rehome_map, true);
                break;
            case RUNNING:
                if (race_map[position.x][position.y].distance == 0) {
                    state = REHOMING;
                    break;
                }
                move_to_next_node(race_map, false);
                break;
        }

        /*if (state != RUNNING) API::setColor(last_position.x, last_position.y, 'c');
        else API::setColor(last_position.x, last_position.y, 'b');
        API::setColor(position.x, position.y, 'g');*/
    }

    return 0;
}