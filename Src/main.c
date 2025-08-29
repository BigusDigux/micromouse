#include "config.h"
#include "init.h"
#include "OLED.h"
#include "encoder.h"
#include "vl6180x.h"
#include "drv8833.h"
#include "buzzer.h"
#include "MPU.h"
#include "floodfill.h"
#include "motion.h"
#include "menu.h"
#include <stdbool.h>
#include <stdlib.h>

static bool started = false;

/*=========================== Helpers ==========================*/
static inline bool btnPressed(GPIO_TypeDef* port, uint16_t pin) {
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

static void waitConfirm(void) {
    HAL_Delay(50);
    while (btnPressed(BTN_CONFIRM_PORT, BTN_CONFIRM_PIN)) {}
}

/*=========================== Main =============================*/
int main(void) {
    System_Init();

    OLED_Print("Initializing", 2, 25);
    Buzzer_Startup();
    MPU_CalibrateGyroZ();

    OLED_Clear();

    processMenu();

    int leftAccum = 0, rightAccum = 0;

    while (1) {
        int32_t leftCount = ENCODER_GetLeft();
        if (leftCount) {
            leftAccum += leftCount; ENCODER_ResetLeft();
            if (abs(leftAccum) >= ENCODER_STEP) {
                leftAccum = 0; Buzzer_Tick();
                if (currentMenu == MENU_MAIN) {
                    mainIndex = (mainIndex + (leftCount > 0 ? 1 : -1) + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;
                    processMenu();
                }
            }
        }

        int32_t rightCount = ENCODER_GetRight();
        if (rightCount) {
            rightAccum += rightCount; ENCODER_ResetRight();
            if (abs(rightAccum) >= ENCODER_STEP) {
                rightAccum = 0; Buzzer_Tick();
                switch (currentMenu) {
                    case MENU_SPEED: subIndex = (subIndex + (rightCount > 0 ? 1 : -1) + 2) % 2; selectedSpeedIndex = subIndex; break;
                    case MENU_TURN:  subIndex = (subIndex + (rightCount > 0 ? 1 : -1) + 3) % 3; selectedTurnIndex  = subIndex; break;
                    case MENU_GOAL_X: goalX += (rightCount > 0 ? 1 : -1); break;
                    case MENU_GOAL_Y: goalY += (rightCount > 0 ? 1 : -1); break;
                    default: break;
                }
                processMenu();
            }
        }

        if (btnPressed(BTN_CONFIRM_PORT, BTN_CONFIRM_PIN)) {
            waitConfirm();
            if (currentMenu == MENU_MAIN) {
                if      (mainIndex == 0) { currentMenu = MENU_SPEED; subIndex = selectedSpeedIndex; }
                else if (mainIndex == 1) { currentMenu = MENU_TURN;  subIndex = selectedTurnIndex; }
                else if (mainIndex == 2) { currentMenu = MENU_GOAL_X; }
                else if (mainIndex == 3) {
                    OLED_Clear(); Buzzer_Short();
                    OLED_Print("Wait for confirmation", 0, 0);
                    while (!started) {
                        uint8_t l = VL6180X_ReadAverage(&tofLeft, 3);
                        uint8_t r = VL6180X_ReadAverage(&tofRight, 3);
                        if (l && r && l <= SENSOR_SIDE_LIMIT && r <= SENSOR_SIDE_LIMIT) {
                            started = true;
                            Buzzer_Confirm();
                            FloodFill_SetGoal(goalX, goalY);
                            FloodFill_Init();
                            FloodFill_UpdateWalls(
                                (VL6180X_ReadAverage(&tofFront, 3) <= SENSOR_FRONT_LIMIT),
                                (VL6180X_ReadAverage(&tofRight, 3) <= SENSOR_FRONT_LIMIT),
                                (VL6180X_ReadAverage(&tofLeft, 3)  <= SENSOR_FRONT_LIMIT));
                            FloodFill_Run();
                        }
                        if (btnPressed(BTN_BACK_PORT, BTN_BACK_PIN)){ break; }
                    }
                }
            } else if (currentMenu == MENU_GOAL_X) currentMenu = MENU_GOAL_Y;
            else currentMenu = MENU_MAIN;
            Buzzer_Short(); processMenu();
        }

        if (btnPressed(BTN_BACK_PORT, BTN_BACK_PIN)) {
            HAL_Delay(50);
            while (btnPressed(BTN_BACK_PORT, BTN_BACK_PIN)) {}
            if (currentMenu != MENU_MAIN) { currentMenu = MENU_MAIN; Buzzer_Short(); processMenu(); }
        }

        if (started) {
            FloodFill_UpdateWalls(
                (VL6180X_ReadRange(&tofFront) <= SENSOR_FRONT_LIMIT),
                (VL6180X_ReadRange(&tofRight) <= SENSOR_FRONT_LIMIT),
                (VL6180X_ReadRange(&tofLeft)  <= SENSOR_FRONT_LIMIT));
            FloodFill_Run(); FloodFill_MoveStep();
            if (FloodFill_AtGoal()) { started = false; reset_motion(); Buzzer_Short(); }
        }
    }
}
