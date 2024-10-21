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

MapNode map[MAP_SIZE][MAP_SIZE];

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

int main(int argc, char* argv[]) {
    generate_map(map);
    update_map_viz(map);

    while (true) {
        if (map[position.x][position.y].distance == 0) return 0;

        if (API::wallLeft()) {
            sever_path(&map[position.x][position.y], rotate_orientation_left(orientation));
        }
        if (API::wallRight()) {
            sever_path(&map[position.x][position.y], rotate_orientation_right(orientation));
        }
        if (API::wallFront()) {
            sever_path(&map[position.x][position.y], orientation);
        }

        update_map_viz(map);

        Orientation target_dir = get_most_optimal_node_direction(&map[position.x][position.y]);

        int delta_theta = orientation - target_dir; // + means turn left | - means turn right

        if (abs(delta_theta) > 2) delta_theta = signbit(delta_theta) ? 1 : -1;

        for (int i = 0; i < delta_theta; i++) turnLeft();
        for (int i = 0; i > delta_theta; i--) turnRight();

        moveForward();

        API::setColor(last_position.x, last_position.y, 'c');
        API::setColor(position.x, position.y, 'g');
    }

    return 0;
}