/*|ROBT 1270: SCARA Console| --------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Advanced Control
# Program: scaraConsole.cpp
#
# Description:
#   This porogram contains the code for the Scara Command Console.
#
# Author: <Your Name>
# Date Created: <Day Started>
# Last Modified: <Today's Date>
# -----------------------------------------------------------------------------*/
#ifndef SCARA_CONSOLE_H_
#define SCARA_CONSOLE_H_

/*|Includes|-------------------------------------------------------------------*/
#include "robot.h"  // <list of functions used> // NOTE: DO NOT REMOVE.
#include "scara.h"
#include <math.h>
#include <string.h> // strcmp,

/*|CONSTANTS|------------------------------------------------------------------*/
#define MAX_CMD 7
#define MAX_ARGS 5
#define MAX_SCARA_STRING 50

enum scaraCmd {MOVE_SCARA_J, MOVE_SCARA_L, SCARA_PEN_UP, SCARA_PEN_DOWN, SCARA_SPEED, SCARA_PEN_COLOR, QUIT};

/*|Structures|-----------------------------------------------------------------*/
// Command
struct CMD{
    const char* name = "";
    const int nArgs = 0;
};

// Console
struct SCARA_CONSOLE{
    char userInput[MAX_SCARA_STRING];
    SCARA_ROBOT scaraRobot;
    char* command;
    char* args[MAX_ARGS + 1];
    int nArgs;
    int cmdInd;
};

/*|Function Declarations|------------------------------------------------------*/
SCARA_CONSOLE initScaraConsole();
void readScaraConsole(SCARA_CONSOLE *con);
void executeScaraCommand(SCARA_CONSOLE *con);
int parseScaraCommand(SCARA_CONSOLE *con);
int validateScaraCommand(SCARA_CONSOLE *con);

#endif /* SCARA_CONSOLE_H_ */
