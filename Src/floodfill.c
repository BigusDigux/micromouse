#include "floodfill.h"
#include "motion.h"
#include "OLED.h"
#include <stdio.h>

// Maze and robot state
static Cell maze[W][H];
static int8_t x, y;
static int8_t goalX = W/2, goalY = H/2;
static Direction currentDir;

// ===== Queue Implementation =====
typedef struct {
    Pair data[W * H];
    int front, rear;
} Queue;

static void queue_init(Queue* q) {
    q->front = q->rear = 0;
}

static void queue_push(Queue* q, Pair p) {
    q->data[q->rear++] = p;
}

static Pair queue_pop(Queue* q) {
    return q->data[q->front++];
}

static bool queue_empty(Queue* q) {
    return q->front >= q->rear;
}

// ===== Internal helpers =====
static inline bool check(int8_t xx, int8_t yy) {
    return xx >= 0 && xx < W && yy >= 0 && yy < H;
}

static Direction rightDir(void) {
    return (Direction)((currentDir + 1) % 4);
}

static Direction leftDir(void) {
    return (Direction)((currentDir + 3) % 4);
}

// ===== API =====

void FloodFill_Init(void) {
    x = 0;
    y = 0;
    currentDir = North;

    for (int i = 0; i < W; i++) {
        for (int j = 0; j < H; j++) {
            for (int d = 0; d < 4; d++) {
                maze[i][j].walls[d] = false;
            }
            maze[i][j].dist = 255;
        }
    }
}

void FloodFill_SetGoal(int gx, int gy) {
    if (check(gx, gy)) {
        goalX = gx;
        goalY = gy;
    }
}

void FloodFill_UpdateWalls(bool wallFront, bool wallRight, bool wallLeft) {
    // Update walls for current cell based on current direction
    maze[x][y].walls[currentDir] = wallFront;
    maze[x][y].walls[rightDir()] = wallRight;
    maze[x][y].walls[leftDir()] = wallLeft;

    // Mirror walls to neighboring cells
    if (maze[x][y].walls[North] && check(x, y + 1))
        maze[x][y + 1].walls[South] = true;
    if (maze[x][y].walls[East] && check(x + 1, y))
        maze[x + 1][y].walls[West] = true;
    if (maze[x][y].walls[South] && check(x, y - 1))
        maze[x][y - 1].walls[North] = true;
    if (maze[x][y].walls[West] && check(x - 1, y))
        maze[x - 1][y].walls[East] = true;
}

void FloodFill_Run(void) {
    Queue q;
    queue_init(&q);

    // Initialize distances to 255
    for (int i = 0; i < W; i++) {
        for (int j = 0; j < H; j++) {
            maze[i][j].dist = 255;
        }
    }

    // Mark the goal cell's distance as 0 and add it to the queue
    maze[goalX][goalY].dist = 0;
    Pair p = {goalX, goalY};
    queue_push(&q, p);

    // Optionally support center cells goal for even-dimensioned maze
    int cx = W / 2 - ((W & 1) ^ 1);
    int cy = H / 2 - ((H & 1) ^ 1);
    for (int i = cx; i <= W / 2; i++) {
        for (int j = cy; j <= H / 2; j++) {
            maze[i][j].dist = 0;
            Pair p = {i, j};
            queue_push(&q, p);
        }
    }

    // BFS to calculate distances to goal
    while (!queue_empty(&q)) {
        Pair p = queue_pop(&q);
        int8_t xq = p.F, yq = p.S;

        for (int dir = 0; dir < 4; dir++) {
            if (!maze[xq][yq].walls[dir]) {
                int8_t nx = xq, ny = yq;
                switch (dir) {
                    case North: ny++; break;
                    case East: nx++; break;
                    case South: ny--; break;
                    case West: nx--; break;
                }
                if (check(nx, ny) && maze[nx][ny].dist > maze[xq][yq].dist + 1) {
                    maze[nx][ny].dist = maze[xq][yq].dist + 1;
                    Pair np = {nx, ny};
                    queue_push(&q, np);
                }
            }
        }
    }
}

void FloodFill_MoveStep(void) {
    uint8_t bestDist = 255;
    Direction bestDir = currentDir;
    int8_t fromX = x, fromY = y;

    // Find neighbor with smallest distance
    for (int dir = 0; dir < 4; dir++) {
        if (!maze[x][y].walls[dir]) {
            int8_t nx = x, ny = y;
            switch (dir) {
                case North: ny++; break;
                case East: nx++; break;
                case South: ny--; break;
                case West: nx--; break;
            }
            if (check(nx, ny) && maze[nx][ny].dist < bestDist) {
                bestDist = maze[nx][ny].dist;
                bestDir = (Direction)dir;
            }
        }
    }

    // Rotate towards best direction
    int rotation = (bestDir - currentDir + 4) % 4;
    switch (rotation) {
        case 1:
            OLED_Clear();
            OLED_Print("Turn Right", 0, 0);
            turn90(false);
            break;
        case 2:
            OLED_Clear();
            OLED_Print("Turn 180", 0, 0);
            turn180();
            break;
        case 3:
            OLED_Clear();
            OLED_Print("Turn Left", 0, 0);
            turn90(true);
            break;
        default:
            OLED_Clear();
            OLED_Print("Forward", 0, 0);
            break;
    }

    currentDir = bestDir;

    // Move forward one cell physically
    driveForward(1);

    // Update internal coordinates
    switch (currentDir) {
        case North: y++; break;
        case East: x++; break;
        case South: y--; break;
        case West: x--; break;
    }

    // Show coordinate transition
    char buf[32];
    snprintf(buf, sizeof(buf), "(%d,%d) -> (%d,%d)", fromX, fromY, x, y);
    OLED_Print(buf, 2, 0);
}

void FloodFill_GetBestPath(Direction path[], int *length) {
    int cx = x;
    int cy = y;
    Direction cdir = currentDir;
    int idx = 0;

    while (!(cx == goalX && cy == goalY)) {
        uint8_t bestDist = 255;
        Direction bestDir = cdir;

        // Find neighbor with smallest distance
        for (int dir = 0; dir < 4; dir++) {
            if (!maze[cx][cy].walls[dir]) {
                int nx = cx, ny = cy;
                switch (dir) {
                    case North: ny++; break;
                    case East:  nx++; break;
                    case South: ny--; break;
                    case West:  nx--; break;
                }
                if (check(nx, ny) && maze[nx][ny].dist < bestDist) {
                    bestDist = maze[nx][ny].dist;
                    bestDir = (Direction)dir;
                }
            }
        }

        // Store step
        path[idx++] = bestDir;

        // Move virtually
        cdir = bestDir;
        switch (bestDir) {
            case North: cy++; break;
            case East:  cx++; break;
            case South: cy--; break;
            case West:  cx--; break;
        }
    }
    *length = idx;
}

void FloodFill_RunBestPath(void) {
    Direction path[W*H];
    int length = 0;
    FloodFill_GetBestPath(path, &length);

    for (int i = 0; i < length; i++) {
        Direction nextDir = path[i];
        int rotation = (nextDir - currentDir + 4) % 4;

        switch (rotation) {
            case 1: turn90(false); break;
            case 2: turn180(); break;
            case 3: turn90(true); break;
            default: break;
        }
        currentDir = nextDir;
        driveForward(1);
    }
}

bool FloodFill_AtGoal(void) {
    return (x == goalX && y == goalY);
}

int8_t FloodFill_GetX(void) { return x; }

int8_t FloodFill_GetY(void) { return y; }

Direction FloodFill_GetDir(void) { return currentDir; }
