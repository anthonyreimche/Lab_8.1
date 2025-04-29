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
		printf("moveScaraJ() failed: scaraIK() returned -1 (invalid move operation)");
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

	// Default color (can be modified later)
	line.color.r = 0;
	line.color.g = 0;
	line.color.b = 0;

	return line;
}

/****************************************************************************************
* Function: moveScaraL
* This function will perform a linear interpolated move.
* It calculates the line length, splits it into points at fixed distance intervals,
* and determines the best arm solution for the entire line. If a single solution
* is not possible, it creates two separate line segments with different solutions.
*
* Inputs:
* scaraState - Contains SCARA information
* line - Line data containing start and end points
*
* Returns:
* check - Report if requested move was invalid (0 for success, -1 for error)
*****************************************************************************************/
int moveScaraL(SCARA_ROBOT *scaraState, LINE_DATA line){
    const double pointDist = 50.0;  // Default distance between points
    const char pen = scaraState->toolPos.penPos;
    double theta1, theta2;
    int leftArmReachable = 0, rightArmReachable = 0;
    int leftArmUnreachableCount = 0, rightArmUnreachableCount = 0;
    int bestOverallArmSol;
    
    // Calculate line length
    double dx = line.xB - line.xA;
    double dy = line.yB - line.yA;
    double lineLength = sqrt(dx*dx + dy*dy);
    
    // Calculate number of points based on pointDist
    int numPoints = max(2, (int)(lineLength / pointDist) + 1);
    
    // Check if start and end points are reachable with either arm solution
    int startLeftReachable = scaraIK(line.xA, line.yA, &theta1, &theta2, LEFT_ARM_SOLUTION) == 0;
    int startRightReachable = scaraIK(line.xA, line.yA, &theta1, &theta2, RIGHT_ARM_SOLUTION) == 0;
    int endLeftReachable = scaraIK(line.xB, line.yB, &theta1, &theta2, LEFT_ARM_SOLUTION) == 0;
    int endRightReachable = scaraIK(line.xB, line.yB, &theta1, &theta2, RIGHT_ARM_SOLUTION) == 0;
    
    // Arrays to track reachability of each point with each arm solution
    int* leftArmSolReachable = (int*)malloc(numPoints * sizeof(int));
    int* rightArmSolReachable = (int*)malloc(numPoints * sizeof(int));
    
    // Analyze all points along the line for both arm solutions
    double step = 1.0 / (numPoints - 1);
    
    // Check each point along the line for both arm solutions
    for (int i = 0; i < numPoints; i++) {
        double t = i * step;
        double x = line.xA + t * dx;
        double y = line.yA + t * dy;
        
        // Check left arm solution
        leftArmSolReachable[i] = (scaraIK(x, y, &theta1, &theta2, LEFT_ARM_SOLUTION) == 0);
        if (leftArmSolReachable[i]) {
            leftArmReachable++;
        } else {
            leftArmUnreachableCount++;
        }
        
        // Check right arm solution
        rightArmSolReachable[i] = (scaraIK(x, y, &theta1, &theta2, RIGHT_ARM_SOLUTION) == 0);
        if (rightArmSolReachable[i]) {
            rightArmReachable++;
        } else {
            rightArmUnreachableCount++;
        }
    }
    
    // Check if any points are reachable
    if (leftArmReachable == 0 && rightArmReachable == 0) {
        printf("\033[31m");
        printf("Error: No points along the line from (%.2f, %.2f) to (%.2f, %.2f) are reachable\n", 
               line.xA, line.yA, line.xB, line.yB);
        printf("\033[0m");
        free(leftArmSolReachable);
        free(rightArmSolReachable);
        return -1;
    }
    
    // Determine the best overall arm solution
    if (leftArmReachable > rightArmReachable) {
        bestOverallArmSol = LEFT_ARM_SOLUTION;
        printf("Left arm solution is better (reachable points: %d/%d)\n", leftArmReachable, numPoints);
    } else {
        bestOverallArmSol = RIGHT_ARM_SOLUTION;
        printf("Right arm solution is better (reachable points: %d/%d)\n", rightArmReachable, numPoints);
    }
    
    // Find first and last reachable points for each solution
    int leftFirstReachable = -1, leftLastReachable = -1;
    int rightFirstReachable = -1, rightLastReachable = -1;
    
    for (int i = 0; i < numPoints; i++) {
        if (leftArmSolReachable[i]) {
            if (leftFirstReachable == -1) leftFirstReachable = i;
            leftLastReachable = i;
        }
        if (rightArmSolReachable[i]) {
            if (rightFirstReachable == -1) rightFirstReachable = i;
            rightLastReachable = i;
        }
    }
    
    // Choose the solution that can reach more of the line
    int firstReachable, lastReachable;
    int armSol;
    
    if ((leftLastReachable - leftFirstReachable) > (rightLastReachable - rightFirstReachable)) {
        firstReachable = leftFirstReachable;
        lastReachable = leftLastReachable;
        armSol = LEFT_ARM_SOLUTION;
        printf("Using LEFT arm solution for maximum coverage\n");
    } else {
        firstReachable = rightFirstReachable;
        lastReachable = rightLastReachable;
        armSol = RIGHT_ARM_SOLUTION;
        printf("Using RIGHT arm solution for maximum coverage\n");
    }
    
    // Calculate the coordinates of the reachable segment
    double t_first = firstReachable * step;
    double t_last = lastReachable * step;
    double x_first = line.xA + t_first * dx;
    double y_first = line.yA + t_first * dy;
    double x_last = line.xA + t_last * dx;
    double y_last = line.yA + t_last * dy;
    
    printf("Reachable segment: (%.2f, %.2f) to (%.2f, %.2f)\n", x_first, y_first, x_last, y_last);
    
    // Check if we need to use two arm solutions
    int needTwoSolutions = false;
    int transitionPoint = -1;
    
    // Check for gaps in reachability with the primary solution
    int* primarySolReachable = (armSol == LEFT_ARM_SOLUTION) ? leftArmSolReachable : rightArmSolReachable;
    int secondarySol = (armSol == LEFT_ARM_SOLUTION) ? RIGHT_ARM_SOLUTION : LEFT_ARM_SOLUTION;
    int* secondarySolReachable = (armSol == LEFT_ARM_SOLUTION) ? rightArmSolReachable : leftArmSolReachable;
    
    // Find a point where we need to switch solutions
    for (int i = firstReachable; i < lastReachable; i++) {
        if (!primarySolReachable[i] && secondarySolReachable[i]) {
            needTwoSolutions = true;
            transitionPoint = i - 1; // Last point before the gap
            break;
        }
    }
    
    // If we need two solutions but didn't find a transition point, find the midpoint
    if (needTwoSolutions && transitionPoint == -1) {
        // Check if the line crosses the x-axis (y=0)
        if ((line.yA > 0 && line.yB < 0) || (line.yA < 0 && line.yB > 0)) {
            // Calculate the point where the line crosses y=0
            // Using the line equation: y = line.yA + t * dy, solve for t when y = 0
            double t_crossing = -line.yA / dy;
            
            // Convert t to the nearest point index
            int crossingPoint = (int)(t_crossing / step);
            
            // Ensure the crossing point is within our range
            if (crossingPoint >= firstReachable && crossingPoint <= lastReachable) {
                transitionPoint = crossingPoint;
                printf("Line crosses x-axis. Setting transition at y=0 point.\n");
            }
        }
        
        // If we still don't have a transition point, use the midpoint
        if (transitionPoint == -1) {
            // Find the midpoint of the reachable range
            transitionPoint = (firstReachable + lastReachable) / 2;
        }
    }
    
    // If we can draw the line with a single solution
    if (!needTwoSolutions) {
        // Create a line segment for the reachable portion
        LINE_DATA reachableSegment;
        reachableSegment.xA = x_first;
        reachableSegment.yA = y_first;
        reachableSegment.xB = x_last;
        reachableSegment.yB = y_last;
        reachableSegment.numPts = lastReachable - firstReachable + 1;
        reachableSegment.color = line.color;
        
        // Move to the start point
        scaraState->armPos.x = reachableSegment.xA;
        scaraState->armPos.y = reachableSegment.yA;
        
        // Set the arm solution
        if (scaraIK(reachableSegment.xA, reachableSegment.yA, &theta1, &theta2, armSol) == 0) {
            scaraState->armPos.armSol = armSol;
            scaraState->armPos.theta1 = theta1;
            scaraState->armPos.theta2 = theta2;
            
            // Move to start point
            moveScaraJ(scaraState);
            
            // Set pen down for drawing
            if (pen == 'D' || pen == 'd') {
                scaraState->toolPos.penPos = 'D';
                scaraSetState(*scaraState);
            }
            
            // Draw the reachable segment
            lerp(scaraState, reachableSegment, reachableSegment.numPts);
        } else {
            printf("\033[31m");
            printf("Error: Failed to move to start point (%.2f, %.2f)\n", reachableSegment.xA, reachableSegment.yA);
            printf("\033[0m");
        }
    } else {
        // We need to use two arm solutions
        // Calculate the transition coordinates
        double t_transition = transitionPoint * step;
        double x_transition = line.xA + t_transition * dx;
        double y_transition = line.yA + t_transition * dy;
        
        printf("Using two arm solutions with transition at (%.2f, %.2f)\n", x_transition, y_transition);
        
        // Find the first reachable point with the secondary solution
        int secondaryFirstReachable = -1;
        for (int i = transitionPoint; i <= lastReachable; i++) {
            if (secondarySolReachable[i]) {
                secondaryFirstReachable = i;
                break;
            }
        }
        
        if (secondaryFirstReachable == -1) {
            printf("\033[33m");
            printf("Warning: No reachable points with secondary solution after transition\n");
            printf("\033[0m");
            secondaryFirstReachable = transitionPoint;
        }
        
        // Calculate the coordinates for the secondary start point
        double t_secondary = secondaryFirstReachable * step;
        double x_secondary = line.xA + t_secondary * dx;
        double y_secondary = line.yA + t_secondary * dy;
        
        // Create first line segment (start to transition)
        LINE_DATA firstSegment;
        firstSegment.xA = x_first;
        firstSegment.yA = y_first;
        firstSegment.xB = x_transition;
        firstSegment.yB = y_transition;
        firstSegment.numPts = transitionPoint - firstReachable + 1;
        firstSegment.color = line.color;
        
        // Move to the start point with primary solution
        if (scaraIK(firstSegment.xA, firstSegment.yA, &theta1, &theta2, armSol) == 0) {
            scaraState->armPos.armSol = armSol;
            scaraState->armPos.theta1 = theta1;
            scaraState->armPos.theta2 = theta2;
            scaraState->armPos.x = firstSegment.xA;
            scaraState->armPos.y = firstSegment.yA;
            
            // Move to start point
            moveScaraJ(scaraState);
            
            // Set pen down for drawing
            if (pen == 'D' || pen == 'd') {
                scaraState->toolPos.penPos = 'D';
                scaraSetState(*scaraState);
            }
            
            // Draw first segment
            lerp(scaraState, firstSegment, firstSegment.numPts);
            
            // Lift pen before switching arm solution
            scaraState->toolPos.penPos = 'U';
            scaraSetState(*scaraState);
        } else {
            printf("\033[31m");
            printf("Error: Failed to move to first segment start point (%.2f, %.2f)\n", firstSegment.xA, firstSegment.yA);
            printf("\033[0m");
        }
        
        // Create second line segment (secondary start to end)
        LINE_DATA secondSegment;
        secondSegment.xA = x_secondary;
        secondSegment.yA = y_secondary;
        secondSegment.xB = x_last;
        secondSegment.yB = y_last;
        secondSegment.numPts = lastReachable - secondaryFirstReachable + 1;
        secondSegment.color = line.color;
        
        // Move to the secondary start point with secondary solution
        if (scaraIK(secondSegment.xA, secondSegment.yA, &theta1, &theta2, secondarySol) == 0) {
            scaraState->armPos.armSol = secondarySol;
            scaraState->armPos.theta1 = theta1;
            scaraState->armPos.theta2 = theta2;
            scaraState->armPos.x = secondSegment.xA;
            scaraState->armPos.y = secondSegment.yA;
            
            // Move to secondary start point
            moveScaraJ(scaraState);
            
            // Set pen down for drawing
            if (pen == 'D' || pen == 'd') {
                scaraState->toolPos.penPos = 'D';
                scaraSetState(*scaraState);
            }
            
            // Draw second segment
            lerp(scaraState, secondSegment, secondSegment.numPts);
        } else {
            printf("\033[31m");
            printf("Error: Failed to move to second segment start point (%.2f, %.2f)\n", secondSegment.xA, secondSegment.yA);
            printf("\033[0m");
        }
    }
    
    // Lift pen after drawing
    scaraState->toolPos.penPos = 'U';
    scaraSetState(*scaraState);
    
    // Free memory
    free(leftArmSolReachable);
    free(rightArmSolReachable);
    
    return 0;
}

/****************************************************************************************
* Function: checkBounds
*
* Description:
*	Checks if a point is reachable by the SCARA robot by testing inverse kinematics
*   for both arm solutions.
*
* Inputs:
*	x          - X coordinate to check
*	y          - Y coordinate to check
*   armSol     - Current arm solution to try first
*   theta1     - Joint 1 angle in degrees
*   theta2     - Joint 2 angle in degrees
*   bestArmSol - Recommended arm solution as plan B
*
* Returns: int - 1 if reachable, 0 if not reachable
*
* Last Modified: April 25, 2025
*****************************************************************************************/
int checkBounds(double x, double y, int armSol, double* theta1, double* theta2, int* bestArmSol) {
    double t1, t2, alt_t1, alt_t2;

    // Try with the current arm solution first
    if (scaraIK(x, y, &t1, &t2, armSol) == 0) {
        if (theta1) *theta1 = t1;
        if (theta2) *theta2 = t2;
        if (bestArmSol) *bestArmSol = armSol;
        return 1; // Point is reachable with current arm solution
    }

    // Try with the alternative arm solution
    int altArmSol = (armSol == LEFT_ARM_SOLUTION) ? RIGHT_ARM_SOLUTION : LEFT_ARM_SOLUTION;
    if (scaraIK(x, y, &alt_t1, &alt_t2, altArmSol) == 0) {
        if (theta1) *theta1 = alt_t1;
        if (theta2) *theta2 = alt_t2;
        if (bestArmSol) *bestArmSol = altArmSol;
        return 1; // Point is reachable with alternative arm solution
    }

    // Point is not reachable with either arm solution
    return 0;
}

/****************************************************************************************
* Function: lerp
*
* Description:
*	Performs linear interpolation between two points, breaking the line into segments
*   and calling moveScaraJ for each point.
*
* Inputs:
*	scaraState - Pointer to the SCARA robot state
*	line       - Line data for the movement
*   numpoints  - Number of points to interpolate (including start and end points)
*
* Returns: int - 0 for success
*
* Last Modified: April 25, 2025
*****************************************************************************************/
int lerp(SCARA_ROBOT *scaraState, LINE_DATA line, int numpoints) {
    int i;
    double t;
    double dx, dy;
    double theta1, theta2;
    int bestArmSol;
    int lastArmSol = scaraState->armPos.armSol;
    int armSwitchOccurred = 0;

    // Check if numpoints is valid
    if (numpoints < 2) {
    	printf("\033[31m");
        printf("Error: Number of points must be at least 2\n");
    	printf("\033[0m");
        return -1;
    }

    // Calculate the step size for interpolation
    double step = 1.0 / (numpoints - 1);

    // Calculate the differences between start and end points
    dx = line.xB - line.xA;
    dy = line.yB - line.yA;

    // Set pen down for drawing
    scaraState->toolPos.penPos = 'd';

    // Move to each interpolated point
    for (i = 0; i < numpoints; i++) {
        // Calculate interpolation parameter t (0.0 to 1.0)
        t = i * step;

        // Linear interpolation: p = p1 + t * (p2 - p1)
        double x = line.xA + t * dx;
        double y = line.yA + t * dy;

        // Check if point is reachable and determine best arm solution
        if (!checkBounds(x, y, scaraState->armPos.armSol, &theta1, &theta2, &bestArmSol)) {
            // Point is unreachable, lift pen and continue to next point
        	printf("\033[33m");
            printf("Warning: Point (%.2f, %.2f) is unreachable, lifting pen\n", x, y);
        	printf("\033[0m");
            scaraState->toolPos.penPos = 'u';
            scaraSetState(*scaraState);
            continue;
        }

        // Check if arm solution needs to be switched
        if (bestArmSol != scaraState->armPos.armSol) {
            // Arm solution switch required
            printf("Switching arm solution at point (%.2f, %.2f): %s to %s\n",
                   x, y,
                   (scaraState->armPos.armSol == LEFT_ARM_SOLUTION) ? "LEFT" : "RIGHT",
                   (bestArmSol == LEFT_ARM_SOLUTION) ? "LEFT" : "RIGHT");

            // Lift pen before switching arm solution
            scaraState->toolPos.penPos = 'u';
            scaraSetState(*scaraState);

            // Update arm solution
            scaraState->armPos.armSol = bestArmSol;
            armSwitchOccurred = 1;
        }

        // Update position and angles
        scaraState->armPos.x = x;
        scaraState->armPos.y = y;
        scaraState->armPos.theta1 = theta1;
        scaraState->armPos.theta2 = theta2;

        // If pen was lifted due to arm switch, put it back down
        if (armSwitchOccurred) {
            moveScaraJ(scaraState); // Move to position with new arm solution
            scaraState->toolPos.penPos = 'd';
            scaraSetState(*scaraState);
            armSwitchOccurred = 0;
        } else {
            // Move to the interpolated point
            moveScaraJ(scaraState);
        }

        // Remember last arm solution
        lastArmSol = scaraState->armPos.armSol;
    }

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

    // Update joint angles only if changed
    if (scaraState.armPos.theta1 != prevState.armPos.theta1 || scaraState.armPos.theta2 != prevState.armPos.theta2) {
        setScaraAngles(scaraState.armPos.theta1, scaraState.armPos.theta2);
        prevState.armPos.theta1 = scaraState.armPos.theta1;
        prevState.armPos.theta2 = scaraState.armPos.theta2;
    }

    // Update pen color only if changed
    if (scaraState.toolPos.penColor.r != prevState.toolPos.penColor.r || scaraState.toolPos.penColor.g != prevState.toolPos.penColor.g || scaraState.toolPos.penColor.b != prevState.toolPos.penColor.b) {
        setScaraColor(scaraState.toolPos.penColor.r, scaraState.toolPos.penColor.g, scaraState.toolPos.penColor.b);
        prevState.toolPos.penColor.r = scaraState.toolPos.penColor.r;
        prevState.toolPos.penColor.g = scaraState.toolPos.penColor.g;
        prevState.toolPos.penColor.b = scaraState.toolPos.penColor.b;
    }

    // Update motor speed only if changed
    if (scaraState.motorSpeed != prevState.motorSpeed) {
        setScaraSpeed(scaraState.motorSpeed);
        prevState.motorSpeed = scaraState.motorSpeed;
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
