#include "menu.h"
#include <stdio.h>

/* Global menu variables */
int selectedSpeedIndex = 0;
int selectedTurnIndex  = 0;
int goalX = 0, goalY = 0;
MenuState currentMenu = MENU_MAIN;
int mainIndex = 0;
int subIndex  = 0;

/* Menu option labels */
static const char * const speedOptions[] = { "Medium", "Fast" };
static const char * const turnOptions[]  = { "Pivot", "Curve", "Diagonal" };

void processMenu(void) {
    OLED_Clear();

    if (currentMenu == MENU_MAIN) {
        OLED_Print("Main Menu", 0, 0);
        switch (mainIndex) {
            case 0: OLED_Print("Speed:", 2, 0);
                    OLED_Print((char*)speedOptions[selectedSpeedIndex], 3, 0); break;
            case 1: OLED_Print("Turn:", 2, 0);
                    OLED_Print((char*)turnOptions[selectedTurnIndex], 3, 0); break;
            case 2: {
                char buf[20];
                snprintf(buf, sizeof(buf), "X=%d Y=%d", goalX, goalY);
                OLED_Print("Goal:", 2, 0);
                OLED_Print(buf, 3, 0);
            } break;
            case 3: OLED_Print("Start", 2, 0); break;
            default: mainIndex = 0; break;
        }
        return;
    }

    const char *title = NULL, *value = NULL;
    char buf[12];

    switch (currentMenu) {
        case MENU_SPEED: title = "Set Speed"; value = speedOptions[subIndex]; break;
        case MENU_TURN:  title = "Set Turn";  value = turnOptions[subIndex]; break;
        case MENU_GOAL_X: title = "Set Goal X"; snprintf(buf, sizeof(buf), "%d", goalX); value = buf; break;
        case MENU_GOAL_Y: title = "Set Goal Y"; snprintf(buf, sizeof(buf), "%d", goalY); value = buf; break;
        default: return;
    }

    OLED_Print(title, 0, 0);
    OLED_Print((char*)value, 2, 0);
}
