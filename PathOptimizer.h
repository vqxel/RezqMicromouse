#pragma once

#include "Main.h"
#include "FloodFill.h"

#include <cstdlib>
#include <iostream>

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


void move_coord_in_direction(Orientation orientation, int *x, int *y);

void reset_iters(); 

int calculate_paths(MapNode map[MAP_SIZE][MAP_SIZE], MapNode *node, VersatilePathSegment *path_node, int x, int y);

RacePathSegment* get_ideal_path(VersatilePathSegment *versatile_path_segment); 

void print_all_paths(VersatilePathSegment *path, int indents_count, int x, int y); 
void print_ideal_path(RacePathSegment *path, int x, int y); 