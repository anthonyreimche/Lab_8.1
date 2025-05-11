/*|ROBT 1270: SCARA |----------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Intermediate Control
# Program: scara.cpp
#
# Description:
#   This program contains code for drawing lines with the SCARA.
#
# Author: <Your Name>
# Date Created: <Day Started>
# Last Modified: <Today's Date>
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scara.h"

/*|Global Variables|-----------------------------------------------------------*/
CRobot robot;     // the global robot Class instance.  Can be used everywhere
                  // robot.Initialize()
                  // robot.Send()
                  // robot.Close()

/*|Function Definitions|-------------------------------------------------------*/
/****************************************************************************************
* Function: scaraDisplayState
*
* Description:
*	Displays the SCARA information. Feel free to modify.
*
* Inputs:
*	scaraState  - Contains SCARA information
*
* Returns: void
*
* Last Modified: May 1, 2021 by Isaiah Regacho
*****************************************************************************************/
void scaraDisplayState(SCARA_ROBOT scaraState) {
	SCARA_POS arm = scaraState.armPos;
	SCARA_TOOL tool = scaraState.toolPos;

	printf("|SCARA STATE|\n");

	// Display Position
	printf("| Theta 1 | Theta 2 |    X    |    Y    |   Arm   |\n");
	printf("|%9.2lf|%9.2lf|%9.2lf|%9.2lf|    %d    |\n", arm.theta1, arm.theta2, arm.x, arm.y, arm.armSol);

	// Display Tool
	printf("|Position |   RED   |  GREEN  |   BLUE  |\n");
	printf("|    %c    |   %3d   |   %3d   |   %3d   |\n", tool.penPos, tool.penColor.r, tool.penColor.g, tool.penColor.b);
}

/****************************************************************************************
* Function: scaraFK
*
* Description:
*	Calculates forward kinematics for the SCARA robot.
*
* Inputs:
*	ang1   - Angle of the first joint (degrees)
*	ang2   - Angle of the second joint (degrees)
*	toolX  - Pointer to store the calculated X coordinate
*	toolY  - Pointer to store the calculated Y coordinate
*
* Returns: int - 0 for success
*
* Last Modified: April 11, 2025
*****************************************************************************************/
int scaraFK(double ang1, double ang2, double* toolX, double* toolY) {
    // Convert angles from degrees to radians
    double theta1 = ang1 * PI / 180.0;
    double theta2 = ang2 * PI / 180.0;

    // Calculate end effector position using forward kinematics
    *toolX = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    *toolY = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    return 0;
}

/****************************************************************************************
* Function: scaraIK
*
* Description:
*	Calculates inverse kinematics for the SCARA robot.
*
* Inputs:
*	toolX  - X coordinate of the tool
*	toolY  - Y coordinate of the tool
*	ang1   - Pointer to store the calculated angle of the first joint (degrees)
*	ang2   - Pointer to store the calculated angle of the second joint (degrees)
*	arm    - Arm solution (LEFT_ARM_SOLUTION or RIGHT_ARM_SOLUTION)
*
* Returns: int - 0 for success, -1 for failure (unreachable position)
*
* Last Modified: April 11, 2025
*****************************************************************************************/
int scaraIK(double toolX, double toolY, double* ang1, double* ang2, int arm) {
    double r = sqrt(toolX * toolX + toolY * toolY);

    // Check if the position is reachable
    if (r > L1 + L2 || r < fabs(L1 - L2)) {
        // Position is unreachable
        return -1;
    }

    // Calculate angle 2 using cosine law
    double cos_theta2 = (toolX * toolX + toolY * toolY - L1 * L1 - L2 * L2) / (2 * L1 * L2);

    // Clamp to valid range to avoid numerical issues
    if (cos_theta2 > 1.0) cos_theta2 = 1.0;
    if (cos_theta2 < -1.0) cos_theta2 = -1.0;

    // Calculate theta2 based on arm solution
    double theta2;
    if (arm == LEFT_ARM_SOLUTION) {
        theta2 = -acos(cos_theta2);  // Elbow down (negative angle)
    } else {
        theta2 = acos(cos_theta2);   // Elbow up (positive angle)
    }

    // Calculate theta1
    double theta1 = atan2(toolY, toolX) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

    // Convert angles from radians to degrees
    *ang1 = theta1 * 180.0 / PI;
    *ang2 = theta2 * 180.0 / PI;

	while (*ang1 > MAX_ABS_THETA1_DEG) *ang1 -= 360;
	while (-*ang1 > MAX_ABS_THETA1_DEG) *ang1 += 360;
	while (*ang2 > MAX_ABS_THETA2_DEG) *ang2 -= 360;
	while (-*ang2 > MAX_ABS_THETA2_DEG) *ang2 += 360;

    // Check if angles are within mechanical limits
    if (fabs(*ang1) > MAX_ABS_THETA1_DEG || fabs(*ang2) > MAX_ABS_THETA2_DEG) {
    	printf("\033[90m");
        printf("Point (%.2f, %.2f) requires joint angles (%.2f, %.2f) that exceed mechanical limits\n", toolX, toolY, *ang1, *ang2);
    	printf("\033[0m");
        return -1;
    }

    return 0;
}

/****************************************************************************************
* Function: moveScaraJ
* This function will perform joint interpolated moves.
*
* Inputs:
* scaraState - Contains SCARA information
*
* Returns:
* check - Report if requested move was invalid
*****************************************************************************************/
int moveScaraJ(SCARA_ROBOT* scaraState) {
	if (scaraIK(scaraState->armPos.x,scaraState->armPos.y, &scaraState->armPos.theta1, &scaraState->armPos.theta2, scaraState->armPos.armSol)) {
		printf("\033[31m");
		printf("moveScaraJ() failed: scaraIK() returned -1 (invalid move operation)\nInput X: %f Y: %f\n", scaraState->armPos.x, scaraState->armPos.y);
		printf("\033[0m");
		return -1;
	}
	scaraSetState(*scaraState);
	return 0;
}

/****************************************************************************************
* Function: initLine
* This function creates a LINE_DATA object.
*
* Inputs:
* xA - Starting x-coordinate
* yA - Starting y-coordinate
* xB - End x-coordinate
* yB - End y-coordinate
* numPts - Number of points to make on the line.
*
* Returns:
* line - Contains line information
*****************************************************************************************/
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts) {
	LINE_DATA line;

	// Initialize line endpoints
	line.xA = xA;
	line.yA = yA;
	line.xB = xB;
	line.yB = yB;

	// Set number of points
	line.numPts = numPts;

	// Calculate the slope and set color based on slope
	double dx = xB - xA;
	double dy = yB - yA;
	double tolerance = 1.0e-5;

	// Zero slope (horizontal line): Green
	if (fabs(dy) <= tolerance) {
		line.color.r = 0;
		line.color.g = 255;
		line.color.b = 0;
	}
	// Infinity slope (vertical line): Black
	else if (fabs(dx) <= tolerance) {
		line.color.r = 0;
		line.color.g = 0;
		line.color.b = 0;
	}
	// Positive slope: Blue
	else if (dy / dx > 0) {
		line.color.r = 0;
		line.color.g = 0;
		line.color.b = 255;
	}
	// Negative slope: Red
	else {
		line.color.r = 255;
		line.color.g = 0;
		line.color.b = 0;
	}

	return line;
}

/****************************************************************************************
* Function: moveScaraL
* This function will perform a linear interpolated move.
*
* Inputs:
* scaraState - Contains SCARA information
* line - Line data containing start and end points
*
* Returns:
* check - Report if requested move was invalid (0 for success, -1 for error)
*****************************************************************************************/
int moveScaraL(SCARA_ROBOT *scaraState, LINE_DATA line){
	double x,y,j1,j2;
	
	// Set the pen color based on the line color
	scaraState->toolPos.penColor = line.color;

	//move to first line
	scaraState->armPos.x = line.xA;
	scaraState->armPos.y = line.yA;
	scaraState->toolPos.penPos = 'u';
	if (line.yA >= 0 && line.yB >= 0) {
		scaraState->armPos.armSol = RIGHT_ARM_SOLUTION;
	} else {
		scaraState->armPos.armSol = LEFT_ARM_SOLUTION;
	}
	if (!moveScaraJ(scaraState)) {
		scaraState->toolPos.penPos = 'd';
	}
	
	for (int i = 0; i < line.numPts; i++) {

		//Calculate x,y coordinates
		x = ((line.xB - line.xA) / (line.numPts - 1) * i) + line.xA;
		y = ((line.yB - line.yA) / (line.numPts - 1) * i) + line.yA;

		if ((y < 0 & scaraState->armPos.y > 0) || (y > 0 & scaraState->armPos.y < 0)) {
			printf("Y-axis crossing detected.\n");
			if ((line.yA <= 0 && line.yB >= 0) || (line.yA >= 0 && line.yB <= 0)) { //Create a transfer point
				printf("Creating Transfer point...\n");
				double slope = (line.yB - line.yA) / (line.xB - line.xA);
				double b = line.yA - slope * line.xA;
				scaraState->armPos.x = -b / slope; // x-coordinate where y=0
				scaraState->armPos.y = 0;
				if (isnan(scaraState->armPos.x) || isnan(scaraState->armPos.y)) {
					printf("Failed to create point.\n");
					scaraState->toolPos.penPos = 'u';
				} else {
					printf("Point Created.\n");
				}

				moveScaraJ(scaraState);
				scaraState->toolPos.penPos = 'u';
				scaraState->armPos.armSol = scaraState->armPos.armSol == LEFT_ARM_SOLUTION ? RIGHT_ARM_SOLUTION : LEFT_ARM_SOLUTION;
				if (!moveScaraJ(scaraState)) {
					scaraState->toolPos.penPos = 'd';
				}
			}
		}

		//Determine best solution
		if (scaraIK(x,y,&j1,&j2,scaraState->armPos.armSol)) { //Switch arm if needed
			scaraState->armPos.armSol = scaraState->armPos.armSol == LEFT_ARM_SOLUTION ? RIGHT_ARM_SOLUTION : LEFT_ARM_SOLUTION;
			printf("Changed arm: %d\n",scaraState->armPos.armSol);
			scaraState->toolPos.penPos = 'u';
			scaraSetState(*scaraState);
			//If both arm solutions are invalid
			if (scaraIK(x,y,&j1,&j2,scaraState->armPos.armSol)) {
				printf("\033[31mInvalid arm solution.\033[0m\n");
				continue;
			}
			if (!moveScaraJ(scaraState)) {
				scaraState->toolPos.penPos = 'd';
			}
			printf("Changing arm.");
		}
		//printf("Current arm: %d\n",scaraState->armPos.armSol);

		//Update Scara robot
		scaraState->armPos.x = x;
		scaraState->armPos.y = y;
		scaraState->armPos.theta1 = j1;
		scaraState->armPos.theta2 = j2;
		if (!moveScaraJ(scaraState) || i==0) {
			scaraState->toolPos.penPos = 'd';
		}
	}
	scaraState->toolPos.penPos = 'u';
	scaraSetState(*scaraState);
	return 0;
}

/****************************************************************************************
* Function: initScaraState
* This function will assign the provided values to a SCARA_ROBOT structure.
*
* Inputs:
* x - X-Coordinate of the SCARA
* y - Y-Coordinate of the SCARA
* armSol - Left or Right Arm Solution for IK
* penState - Color and Position of the pen
* mtrSpeed - SCARA Speed Setting
*
* Returns:
* robot - Contains all provided information
*****************************************************************************************/
SCARA_ROBOT initScaraState(double x, double y, int armSol, SCARA_TOOL penState, char mtrSpeed) {
	//Initialize robot
	printf("Initializing robot...\n");
	SCARA_ROBOT rbt;
	rbt.armPos.x = x;
	rbt.armPos.y = y;
	rbt.armPos.armSol = armSol;
	rbt.toolPos = penState;
	rbt.motorSpeed = mtrSpeed;

	//Calculate joint angles
	printf("Calculating scara angles...\n");
	if (scaraIK(x,y, &rbt.armPos.theta1, &rbt.armPos.theta2, armSol)) {
		printf("\033[31m");
		printf("scara angles failed. Using J1 = J2 = 0 deg...\n");
		printf("\033[0m");
		rbt.armPos.theta1 = 0;
		rbt.armPos.theta2 = 0;
	} else {
		printf("Initialization complete.\n");
	}
	return rbt;
}

/****************************************************************************************
* Function: scaraSetState
* Sends commands to the scara simulator.
*
* Inputs:
* scaraState - Contains SCARA information
*
* Returns: void
*****************************************************************************************/
void scaraSetState(SCARA_ROBOT scaraState) {
	// Static SCARA_ROBOT to hold previous state
    static int initialized = 0;
    static SCARA_ROBOT prevState;

    // Initialize prevState with invalid values on first call
    if (!initialized) {
        prevState.armPos.theta1 = 1e9;
        prevState.armPos.theta2 = 1e9;
        prevState.armPos.x = 1e9;
        prevState.armPos.y = 1e9;
        prevState.armPos.armSol = -1;
        prevState.toolPos.penPos = '\0';
        prevState.toolPos.penColor.r = -1;
        prevState.toolPos.penColor.g = -1;
        prevState.toolPos.penColor.b = -1;
        prevState.motorSpeed = '\0';
        initialized = 1;
    }

	// Update pen position only if changed
	if (scaraState.toolPos.penPos != prevState.toolPos.penPos) {
		setScaraPen(scaraState.toolPos.penPos);
		prevState.toolPos.penPos = scaraState.toolPos.penPos;
	}

    // Update pen color only if changed
    if (scaraState.toolPos.penColor.r != prevState.toolPos.penColor.r || scaraState.toolPos.penColor.g != prevState.toolPos.penColor.g || scaraState.toolPos.penColor.b != prevState.toolPos.penColor.b) {
    	setScaraAngles(0, 0);
    	setScaraColor(scaraState.toolPos.penColor.r, scaraState.toolPos.penColor.g, scaraState.toolPos.penColor.b);
        prevState.toolPos.penColor.r = scaraState.toolPos.penColor.r;
        prevState.toolPos.penColor.g = scaraState.toolPos.penColor.g;
        prevState.toolPos.penColor.b = scaraState.toolPos.penColor.b;
    	setScaraAngles(scaraState.armPos.theta1, scaraState.armPos.theta2);
    }

    // Update motor speed only if changed
    if (scaraState.motorSpeed != prevState.motorSpeed) {
        setScaraSpeed(scaraState.motorSpeed);
        prevState.motorSpeed = scaraState.motorSpeed;
    }

    // Update joint angles only if changed
    if (scaraState.armPos.theta1 != prevState.armPos.theta1 || scaraState.armPos.theta2 != prevState.armPos.theta2) {
        setScaraAngles(scaraState.armPos.theta1, scaraState.armPos.theta2);
        prevState.armPos.theta1 = scaraState.armPos.theta1;
        prevState.armPos.theta2 = scaraState.armPos.theta2;
    }
}

/****************************************************************************************
* Function: setScaraAngles
* This function will rotate the SCARA joints.
*
* Inputs:
* ang1 - Angle of joint 1 in degrees.
* ang2 - Angle of joint 2 in degrees.
*
* Returns: void
******************************************************************************************/
void setScaraAngles(double ang1, double ang2) {
	char command[MAX_STRING];
	sprintf(command, "ROTATE_JOINT ANG1 %.2f ANG2 %.2f\n", ang1, ang2);
	robot.Send(command);
}

/****************************************************************************************
* Function: setScaraPen
* sets the pen of the SCARA.
*
* Inputs:
* pen - 'd' for pen down, otherwise lift pen.
*
* Returns: void
*****************************************************************************************/
void setScaraPen(char pen) {
	switch (pen) {
		case 'D':
		case 'd':
			robot.Send("PEN_DOWN\n");
		break;
		default:
			robot.Send("PEN_UP\n");
	}
}

/****************************************************************************************
* Function: setScaraColor
* This function will change the pen color.
*
* Inputs:
* r, g, b - Pen color
*
* Returns: void
*****************************************************************************************/
void setScaraColor(int r, int g, int b) {
	char command[MAX_STRING];
	sprintf(command, "PEN_COLOR %d %d %d\n", r, g, b);
	robot.Send(command);
}

/****************************************************************************************
* Function: setScaraSpeed
* This function sets the Speed of the SCARA motors.
*
* Inputs:
* speed - H/M/L
*
* Returns: void
*****************************************************************************************/
void setScaraSpeed(char speed) {

	switch (speed) {
		case 'H':
		case 'h':
			robot.Send("MOTOR_SPEED HIGH\n");
		printf("Motor speed changed: HIGH\n");
		break;
		case 'M':
		case 'm':
			robot.Send("MOTOR_SPEED MEDIUM\n");
		printf("Motor speed changed: MEDIUM\n");
		break;
		case 'L':
		case 'l':
			robot.Send("MOTOR_SPEED LOW\n");
			printf("Motor speed changed: LOW\n");
		break;
		default:
			printf("\033[31m");
			printf("Invalid motor speed: \'%c\'\n",speed);
			printf("\033[0m");
	}
}
