#include "Main.h"
#include "FloodFill.h"
#include "PathOptimizer.h"
#include "API.h"

#include <iostream>
#include <string>
#include <stack>
#include <math.h>

typedef struct {
    int x = 0;
    int y = 0;
} Position;

typedef enum {
    SEARCHING,
    REHOMING,
    RUNNING,
} State;

VersatilePathSegment path = VersatilePathSegment();

State state = SEARCHING;
int times_searched = 1;

MapNode race_map[MAP_SIZE][MAP_SIZE];
MapNode rehome_map[MAP_SIZE][MAP_SIZE];

Position last_position = Position();
Position position =  Position();
Orientation orientation = NORTH;

LoggingLevel logging_level = NONE;

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

void move_to_next_node(MapNode map[MAP_SIZE][MAP_SIZE], bool attempt_unmapped, bool randomize_search) {
    update_map_viz(map);
    Orientation target_dir = get_most_optimal_node_connection(&map[position.x][position.y], attempt_unmapped, randomize_search).connection_orientation;
    int delta_theta = orientation - target_dir; // + means turn left | - means turn right

    if (abs(delta_theta) > 2) delta_theta = signbit(delta_theta) ? 1 : -1;

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
    return percent_mapped(race_map) > 0.80 || times_searched > 2;
}

int main(int argc, char* argv[]) {
    generate_map(race_map);
    propogate_race_distances(race_map);
    generate_map(rehome_map);
    propogate_rehome_distances(rehome_map);

    update_map_viz(race_map);

    while (true) {
        if (logging_level == DEBUG) std::cerr << "STATE: " << state <<  "   (" << position.x << ", " << position.y << ")   Percent mapped: " << percent_mapped(race_map) * 100 << "\%   Should Explore: " << !should_not_explore() << std::endl;

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

        Orientation target_dir;

        switch (state) {
            case SEARCHING:
                if (race_map[position.x][position.y].distance == 0) {
                    state = REHOMING;
                    break;
                }
                move_to_next_node(race_map, true, true);
                break;
            case REHOMING:
                if (rehome_map[position.x][position.y].distance == 0) {
                    if (should_not_explore()) {
                        state = RUNNING;
                        path = VersatilePathSegment();
                        reset_iters();
                        int best_path_length = calculate_paths(race_map, &race_map[0][0], &path, 0, 0);

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
                move_to_next_node(rehome_map, true, true);
                break;
            case RUNNING:
                if (race_map[position.x][position.y].distance == 0) {
                    state = REHOMING;
                    break;
                }
                move_to_next_node(race_map, false, false);
                break;
        }
    }

    return 0;
}