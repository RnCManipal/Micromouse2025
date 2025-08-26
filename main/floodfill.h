#ifndef FLOODFILL_H
#define FLOODFILL_H

#include "config.h"
#include "movement.h"
#include "data_structures.h" // Include the header with the Queue class

// Global Variables (Declared as extern in the header, defined in the .c file)
constexpr int length = 6;

extern char path_taken[length *length];
extern int path_index;
extern char path[4000];
extern bool wall_data[length][length][4];
extern bool dup_arr[length][length][4];
extern char short_path[4000];
extern int short_path_index;
extern bool last_was_back;
extern short arena_map[length][length];

// Function Prototypes
int isOpposite(char a, char b);
void reduceDirections(const char* input);
void print_maze(int bot_x, int bot_y);
void print_path_taken();
void swap(int* x, int* y);
int* minimum_cost(short int arena_map[length][length], short int bot_pos[2], int* sortedArray);
int minimum_value_accessible_neighbors(short int arena_map[length][length], short int pos[2], int* smallest_accessible_regardless, bool wall_data[][16][4]);
void rearrange_map(short int arena_map[length][length], short int base_pos[2], bool wall_data[][length][4]);
int direction_wrt_compass(short int arena_map[length][length], short int bot_pos[2], bool wall_data[][length][4]);
int direction_wrt_bot(short int arena_map[length][length], short int bot_pos[2], int facing, bool wall_data[][length][4]);
int floodfill();
void final_run(const char* reduced);

#endif /* FLOODFILL_H_ */