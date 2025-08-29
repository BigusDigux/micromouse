#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>
#include <stdbool.h>

// Maze size
#define W 16
#define H 16

// Directions
typedef enum {
    North = 0,
    East,
    South,
    West
} Direction;

// Cell structure
typedef struct {
    bool walls[4];  // N, E, S, W
    uint8_t dist;
} Cell;

// Pair for queue
typedef struct {
    int8_t F, S;
} Pair;

// Public API
void FloodFill_Init(void);
void FloodFill_SetGoal(int gx, int gy);
void FloodFill_UpdateWalls(bool wallFront, bool wallRight, bool wallLeft);
void FloodFill_Run(void);
bool FloodFill_AtGoal(void);
void FloodFill_MoveStep(void);

// Accessors
int8_t FloodFill_GetX(void);
int8_t FloodFill_GetY(void);
Direction FloodFill_GetDir(void);

#endif // FLOODFILL_H
