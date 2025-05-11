/*|ROBT 1270: SCARA Console| --------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Advanced Control
# Program: scaraConsole.cpp
#
# Description:
#   This program contains the code for the Scara Command Console.
#
# Author: <Your Name>
# Date Created: <Day Started>
# Last Modified: <Today's Date>
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scaraConsole.h"
#include <stdio.h>
#include <stdlib.h>

/*|Global Variables|-----------------------------------------------------------*/
extern CRobot robot;     // the global robot Class instance.  Can be used everywhere
LINE_DATA line;

CMD scaraCommands[MAX_CMD] = {
    {"moveScaraJ", 2},
    {"moveScaraL", 5},
    {"scaraPenUp", 0},
    {"scaraPenDown", 0},
    {"scaraSpeed", 1},
    {"scaraPenColor", 3},
    {"quit", 0}
};

/*|Function Definitions|-------------------------------------------------------*/
SCARA_CONSOLE initScaraConsole(){
    // Variable Declarations
    SCARA_CONSOLE con;

    // Customize Output
	system("COLOR 0A");
	system("CLS");

	// Initialize SCARA Simulator V3
	if(!robot.Initialize()) exit(0);
	robot.Send("PEN_UP\n");
	robot.Send("HOME\n");
	robot.Send("CLEAR_TRACE\n");
    robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");

    // Default Position for Scara
    con.scaraRobot = initScaraState(600, 0, LEFT_ARM_SOLUTION,{'u',{255, 0, 0}},'H');
    scaraSetState(con.scaraRobot);

    return con;
}

void readScaraConsole(SCARA_CONSOLE *con){
    printf("\n>>>");
    fgets(con->userInput, MAX_SCARA_STRING, stdin);
    
    // Remove newline character if present
    size_t len = strlen(con->userInput);
    if (len > 0 && con->userInput[len-1] == '\n') {
        con->userInput[len-1] = '\0';
    }
}


void executeScaraCommand(SCARA_CONSOLE* con){
    switch(con->cmdInd){
        case MOVE_SCARA_J:
        {
            printf("\nMoveScaraJ\n>>>");
            con->scaraRobot.armPos.x = strtod(con->args[0], NULL);
            con->scaraRobot.armPos.y = strtod(con->args[1], NULL);
            moveScaraJ(&con->scaraRobot);
            break;
        }
        case MOVE_SCARA_L:
        {
            printf("\nMoveScaraL\n>>>");
            // Parse all 5 arguments correctly: x1, y1, x2, y2, nPts
            double x1 = strtod(con->args[0], NULL);
            double y1 = strtod(con->args[1], NULL);
            double x2 = strtod(con->args[2], NULL);
            double y2 = strtod(con->args[3], NULL);
            int nPts = atoi(con->args[4]);
            
            // Initialize line with the parsed parameters
            line = initLine(x1, y1, x2, y2, nPts);
            
            // Move robot along the line
            moveScaraL(&con->scaraRobot, line);
            break;
        }
        case SCARA_PEN_UP:
        {
            printf("\nScaraPenUp\n>>>");
            con->scaraRobot.toolPos.penPos = 'u';
            scaraSetState(con->scaraRobot);
            break;
        }
        case SCARA_PEN_DOWN:
        {
            printf("\nScaraPenDown\n>>>");
            con->scaraRobot.toolPos.penPos = 'd';
            scaraSetState(con->scaraRobot);
            break;
        }
        case SCARA_SPEED:
        {
            printf("\nScaraSpeed\n>>>");
            con->scaraRobot.motorSpeed = con->args[0][0];
            scaraSetState(con->scaraRobot);
            break;
        }
        case SCARA_PEN_COLOR:
        {
            printf("\nScaraPenColor\n>>>");
            con->scaraRobot.toolPos.penColor.r = atoi(con->args[0]);
            con->scaraRobot.toolPos.penColor.g = atoi(con->args[1]);
            con->scaraRobot.toolPos.penColor.b = atoi(con->args[2]);
            scaraSetState(con->scaraRobot);
            break;
        }
        case QUIT:
        {
            printf("\nQuit\n>>>");
            break;
        }
        default:
        {
            printf("\nInvalid Command: %s\n>>>", con->command);
        }
    }

}


int parseScaraCommand(SCARA_CONSOLE* con){
    // Default values
    con->nArgs = 0;
    con->cmdInd = -1; // INVALID_COMMAND
    
    // Create a local copy of the input string to work with
    // This ensures we have a stable copy throughout the parsing process
    static char inputCopy[MAX_SCARA_STRING];
    strncpy(inputCopy, con->userInput, MAX_SCARA_STRING - 1);
    inputCopy[MAX_SCARA_STRING - 1] = '\0'; // Ensure null termination
    
    // Create local storage for arguments
    static char argStorage[MAX_ARGS][MAX_SCARA_STRING];
    
    // Get first token (command)
    char* token = strtok(inputCopy, " ");
    if(token) {
        // Store command in a safe way
        static char cmdStorage[MAX_SCARA_STRING];
        strncpy(cmdStorage, token, MAX_SCARA_STRING - 1);
        cmdStorage[MAX_SCARA_STRING - 1] = '\0';
        con->command = cmdStorage;
        
        // Simple command lookup
        con->cmdInd = (int)(sizeof(scaraCommands) / sizeof(CMD)) - 1;
        while (con->cmdInd >= 0 && strcmp(con->command, scaraCommands[con->cmdInd].name)) {
            con->cmdInd--;
        }
        
        // Get remaining tokens (arguments)
        while((token = strtok(NULL, " ")) && con->nArgs < MAX_ARGS) {
            // Copy each argument to its own storage
            strncpy(argStorage[con->nArgs], token, MAX_SCARA_STRING - 1);
            argStorage[con->nArgs][MAX_SCARA_STRING - 1] = '\0';
            con->args[con->nArgs] = argStorage[con->nArgs];
            con->nArgs++;
        }
        
        // NULL-terminate args array
        con->args[con->nArgs] = NULL;
    } else {
        static char emptyStr[1] = {0}; // Empty string buffer
        con->command = emptyStr;
    }
    
    return con->nArgs;
}

int validateScaraCommand(SCARA_CONSOLE* con){
    // Check if command exists
    if (con->cmdInd < 0) {
        printf("Error: Command '%s' not found.\n", con->command);
        return 0;
    }
    
    // Check if correct number of arguments were provided
    int expectedArgs = scaraCommands[con->cmdInd].nArgs;
    if (con->nArgs != expectedArgs) {
        printf("Error: Expected %d arguments, received %d.\n", expectedArgs, con->nArgs);
        return 0;
    }
    
    return 1;
}