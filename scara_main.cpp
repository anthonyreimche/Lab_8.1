/*|SCARA Simulator|------------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Intermediate Control
# Program: scara_main.c
#
# Description:
#   This program demonstrates intermediate control over the SCARA Robot 
# Simulator. The simulator moves with only commands to control the motor 
# angles. To move to a desired (x, y) coordinate requires the implementation of
# inverse kinematics. Intermediate control introduces linear interpolation.
# Available commands are listed below.
#
# Other Programs:
#   ScaraRobotSimulator.exe (Version 4.3)
#
# Simulator Commands:
#  - PEN_UP
#  - PEN_DOWN
#  - PEN_COLOR <r> <g> <b>
#  - CYCLE_PEN_COLORS ON/OFF
#  - ROTATE_JOINT ANG1 <deg1> ANG2 <deg2>
#  - CLEAR_TRACE
#  - CLEAR_REMOTE_COMMAND_LOG
#  - CLEAR_POSITION_LOG
#  - SHUTDOWN_SIMULATION
#  - MOTOR_SPEED HIGH/MEDIUM/LOW
#  - MESSAGE <"string">
#  - HOME
#  - END
#
# Other Information:
#  - IP Address: 127.0.0.1 Port 1270
#  - BCIT Blue: 10 64 109
#  - If using VS Code, add the following args to tasks.json g++ build task.
#     "-std=c++11"
#		"-lwsock32"
#		"-Wno-deprecated"
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
#include "scara.h"  // <list of functions and structures used> // Your code should be inside here.

/*|Enumerations|---------------------------------------------------------------*/
typedef enum {
	SIMULATOR = 1, SCARA, JOINT, LINEAR, QUIT
} menu;

/*|Global Variables|-----------------------------------------------------------*/
extern CRobot robot;

/*|Function Declarations|------------------------------------------------------*/
void scaraTest(void);
void simulatorCommandTest(void);
void moveScaraJTest(void);
void moveScaraLTest(void);

int main(){

	// Variables
	int select = 0;

	// Initialize SCARA Simulator V3
	if(!robot.Initialize()) return 0;
	robot.Send("PEN_UP\n");
	robot.Send("HOME\n");
	robot.Send("CLEAR_TRACE\n");
	robot.Send("CLEAR_POSITION_LOG\n");
	robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");

	// Repeat Until 5: Quit is selected
	do {
		//system("CLS");
		printf("\n\n");
		// Display the Menu
		printf("==========MAIN MENU==========\n\n");
		printf("1) Task 1: Simulator Command Functions\n");
		printf("2) Task 2: Scara State Functions\n");
		printf("3) Task 3.1: Joint Interpolated Move\n");
		printf("4) Task 3.2: Linear Interpolated Move\n");
		printf("5) Quit\n\n");

		// Get the User Input
		scanf("%d", &select);
		getchar();

		// Execute the Selected Program
		switch (select) {
		case SIMULATOR:
			simulatorCommandTest();
			break;
		case SCARA:
			scaraTest();
			break;
		case JOINT:
			moveScaraJTest();
			break;
		case LINEAR:
			moveScaraLTest();
			break;
		case QUIT:
			robot.Send("END\n");
			robot.Close();
			break;
		default:
			printf("\n\nPlease Select one of the Menu Items.\n");
			getchar();
		}
	} while(select != QUIT);

	// Close the Connection to the SCARA Simulator V3
   robot.Send("END\n");
   printf("\n\nPress ENTER to end the program...\n");
   getchar();

   
   robot.Close(); // close remote connection
   return 0;
}

/****************************************************************************************
* Function: simulatorCommandTest
*
* Description:
*	This function will test setScaraAngles, setScaraPen, setScaraColor, setScaraSpeed.
*
* Inputs: void
*
* Returns: void
*
* Last Modified: April 15, 2025 by Isaiah Regacho
*****************************************************************************************/
void simulatorCommandTest(void) {
	setScaraSpeed('H');
	setScaraPen('u');
	setScaraAngles(0, 0);

	setScaraPen('d');
	setScaraColor(255,0,0);
	setScaraAngles(90, -90);
	setScaraPen('u');
	
	setScaraSpeed('H');
	setScaraAngles(0, 0);

	setScaraSpeed('M');
	setScaraPen('d');
	setScaraColor(0, 255, 0);
	setScaraAngles(-90, 90);
	setScaraPen('u');

	setScaraSpeed('H');
	setScaraAngles(0, 0);
	setScaraColor(0, 0, 255);
	setScaraAngles(90, 90);
	setScaraPen('d');
	setScaraSpeed('L');
	setScaraAngles(-90, -90);
	setScaraPen('u');
	setScaraSpeed('H');
	setScaraAngles(0, 0);
}

/****************************************************************************************
* Function: scaraTest
*
* Description:
*	This function will test initScaraState, setScaraState.
*
* Inputs: void
*
* Returns: void
*
* Last Modified: April 15, 2025 by Isaiah Regacho
*****************************************************************************************/
void scaraTest(void) {
	SCARA_ROBOT testRobot = initScaraState(200, 200, LEFT_ARM_SOLUTION,{'u',{255, 0, 0}},'H');

	scaraIK(200, 200, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	testRobot.toolPos.penPos = 'd';

	scaraIK(200, 400, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	scaraIK(400, 400, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	scaraIK(400, 200, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);
	
	scaraIK(200, 200, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	testRobot.toolPos.penPos = 'u';
	testRobot.toolPos.penColor.b = 255;
	testRobot.motorSpeed = 'L';
	scaraIK(200, -200, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	testRobot.toolPos.penPos = 'd';
	scaraIK(200, -400, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	scaraIK(400, -400, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

	scaraIK(400, -200, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);
	
	scaraIK(200, -200, &testRobot.armPos.theta1, &testRobot.armPos.theta2, testRobot.armPos.armSol);
	scaraSetState(testRobot);

}

/****************************************************************************************
* Function: moveScaraJTest
*
* Description:
*	This function will test moveScaraJ.
*
* Inputs: void
*
* Returns: void
*
* Last Modified: May 1, 2021 by Isaiah Regacho
*****************************************************************************************/
void moveScaraJTest(void) {
	// Starting Point
	SCARA_ROBOT robot = initScaraState(600, 0, LEFT_ARM_SOLUTION, {'u', {24, 0, 66}}, 'L');
	moveScaraJ(&robot);

	robot.armPos.x = 300;
	robot.armPos.y = 300;
	robot.armPos.armSol = RIGHT_ARM_SOLUTION;
	robot.toolPos.penPos = 'u';
	moveScaraJ(&robot);
	scaraDisplayState(robot);
	getchar();

	robot.armPos.x = 300;
	robot.armPos.y = 0;
	robot.armPos.armSol = RIGHT_ARM_SOLUTION;
	robot.toolPos.penPos = 'd';
	moveScaraJ(&robot);
	scaraDisplayState(robot);
	getchar();

	robot.armPos.x = 300;
	robot.armPos.y = -300;
	robot.armPos.armSol = LEFT_ARM_SOLUTION;
	robot.toolPos.penPos = 'd';
	moveScaraJ(&robot);
	scaraDisplayState(robot);
	getchar();

}

/****************************************************************************************
* Function: moveScaraLTest
*
* Description:
*	This function will test moveScaraL.
*
* Inputs: void
*
* Returns: void
*
* Last Modified: May 1, 2021 by Isaiah Regacho
*****************************************************************************************/
void moveScaraLTest(void) {
	LINE_DATA lineData;

	// Initialized Simulator from this Point
	SCARA_ROBOT robot = initScaraState(300, 300, RIGHT_ARM_SOLUTION, {'u', {0, 0, 255}}, 'H');
	moveScaraJ(&robot);

	// Easy Set of Lines
	lineData = initLine(300, 0, 300, 300, 10);
	moveScaraL(&robot, lineData);

	lineData = initLine(100, 500, 300, 300, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(300, 0, 300, -300, 10);
	moveScaraL(&robot, lineData);

	lineData = initLine(100, -500, 300, -300, 20);
	moveScaraL(&robot, lineData);

	printf("Easy Lines Complete!");
	getchar();
	
	// Medium Set of Lines
	lineData = initLine(0, 600, 600, 0, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(600, 0, 0, -600, 20);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(300, 300, -500, 300, 10);
	moveScaraL(&robot, lineData);

	lineData = initLine(-500, 300, -300, 500, 5);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(300, -300, -500, -300, 10);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(-500, -300, -300, -500, 5);
	moveScaraL(&robot, lineData);

	printf("Medium Lines Complete!");
	getchar();

	// Hard Set of Lines
	lineData = initLine(-500, 300, 600, 0, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(600, 0, -300, 500, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(-500, -300, 600, 0, 20);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(600, 0, -300, -500, 20);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(-500, -300, -300, 500, 10);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(-500, 300, -300, -500, 10);
	moveScaraL(&robot, lineData);

	printf("Hard Lines Complete!");
	getchar();

	// Challenge Set of Lines
	lineData = initLine(-500, 300, 0, -600, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(0, -600, -300, 500, 20); 
	moveScaraL(&robot, lineData);

	lineData = initLine(-500, -300, 0, 600, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(0, 600, -300, -500, 20);
	moveScaraL(&robot, lineData);

	printf("Challenge Lines Complete!");
	getchar();
}