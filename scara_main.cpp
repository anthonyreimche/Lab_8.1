/*|SCARA Simulator|------------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Advanced Control
# Program: scara_main.c
#
# Description:
#   This program demonstrates advanced control over the SCARA Robot Simulator.
# The following commands can be entered:
#
# 	- moveScaraJ x, y
#	- moveScaraL x1, y1, x2, y2, n
#	- scaraPenUp
#	- scaraPenDown
#	- scaraSpeed s
#	- scaraPenColor r, g, b
#	- quit
#
# Other Programs:
#   ScaraRobotSimulator.exe (Version 4.3)
#
# Other Information:
#  - IP Address: 127.0.0.1 Port 1270
#  - BCIT Blue: 10 64 109
#  - If using VS Code, add the following args to tasks.json g++ build task.
#     "-std=c++14"
#	  "-lwsock32"
#     "-Wno-deprecated"
#  - Also change the "${file}" argument to "scara_main.cpp" and add "scara.cpp".
#
# Author: <Name>
# Date Created: <Date>
# Last Modified: <Today>
# -----------------------------------------------------------------------------*/
#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>  // <list of functions used>
#include <math.h>   // <list of functions used>
#include <stdlib.h>
#include "scaraConsole.h"  // <list of functions and structures used>

/*|Enumerations|---------------------------------------------------------------*/

/*|Global Variables|-----------------------------------------------------------*/
extern CRobot robot;
extern CMD scaraCommands[MAX_CMD];

/*|Function Declarations|------------------------------------------------------*/

int main(){
	// Variables
	SCARA_CONSOLE scon;
	int nArgs = 0;

   	// Initialize Scara Console
	initScaraConsole();

	// Repeat Until 5: Quit is selected
	do {
		readScaraConsole(&scon);
		parseScaraCommand(&scon);

		if (validateScaraCommand(&scon))
			executeScaraCommand(&scon);
	} while(scon.cmdInd != QUIT);

	// Close the Connection to the SCARA Simulator V3
   robot.Send("END\n");
   printf("\n\nPress ENTER to end the program...\n");
   getchar();

   
   robot.Close(); // close remote connection
   return 0;
}