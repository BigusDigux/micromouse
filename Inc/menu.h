#ifndef MENU_H
#define MENU_H

#include "config.h"
#include "OLED.h"
#include <stdbool.h>

/* Menu states */
typedef enum { 
    MENU_MAIN, 
    MENU_SPEED, 
    MENU_TURN, 
    MENU_GOAL_X, 
    MENU_GOAL_Y 
} MenuState;

/* Globals that other files may use */
extern int selectedSpeedIndex;
extern int selectedTurnIndex;
extern int goalX, goalY;
extern MenuState currentMenu;
extern int mainIndex;
extern int subIndex;

/* Menu API */
void processMenu(void);

#endif // MENU_H
