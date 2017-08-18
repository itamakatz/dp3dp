/*
# FILE: slicer.py
# @date 3/5/2017
*/

#include <iostream>
#include <string.h>
#include <fstream>



///////////////////////////////////////////////////////////////////
///////// CHANGE THESE ACCORDING TO THE FILE NAME  ////////////////
///////////////////////////////////////////////////////////////////
#define ABSOLUTE_INPUT_PATH "/home/liav/Desktop/dynamic_slicer/aaa.gcode"
#define ABSOLUTE_OUTPUT_PATH "/home/liav/Desktop/dynamic_slicer/aaa_updated.gcode"
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////


#ifndef NUMERIC
    #define NUMERIC(a) ((a) >= '0' && '9' >= (a))
#endif

#ifndef NUMERIC_SIGNED
    #define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-')
#endif

#define MOVE_ON -1

#define WRITE(fd, cmd) (fprintf((fd), strchr(cmd, '\n') == NULL ? "%s\n": "%s", (cmd)))
#define READ(fd, cmd) (fscanf((fd), "%s", (cmd)))


float g_current_R_position = 3.0;
float g_current_Theta_position = 0.0;
float g_current_X_position = 3.0;
float g_current_Y_position = 0.0;
float printer_origin_X = 0.0;
float printer_origin_Y = 0.0;


static char *current_command, *current_command_args, *seen_pointer;


bool code_seen(char code) {
    seen_pointer = strchr(current_command_args, code);
    return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}



void G0_G1_gcode(){

}

/**
 *
 * @param cmd
 * @return
 */
int get_code_number(char* cmd){
    current_command = cmd;
    while (*current_command == ' ') ++current_command;
    if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
        current_command += 2; // skip N[-0-9]
        while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
        while (*current_command == ' ') ++current_command; // skip [ ]*
    }
    char* starpos = strchr(current_command, '*');	// * should always be the last parameter
    if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

    char *cmd_ptr = current_command;

    // Get the command code, which must be G, M, or T
    char command_code = *cmd_ptr++;
    if (command_code != 'G') return MOVE_ON;

    int codenum = 0;
    // Get and skip the code number
    do {
        codenum = ((codenum * 10) + (*cmd_ptr - '0'));
        cmd_ptr++;
    } while (NUMERIC(*cmd_ptr));

    // Skip all spaces to get to the first argument, or null
    while (*cmd_ptr == ' ') cmd_ptr++;

    // The command's arguments (if any) start here, for sure!
    current_command_args = cmd_ptr;
    return codenum;
}



void parse(){

    FILE* input_fd = fopen(ABSOLUTE_INPUT_PATH, "r");
    FILE* output_fd = fopen(ABSOLUTE_OUTPUT_PATH, "w");
    char cmd[100];

    while (READ(input_fd, cmd) != EOF){
        int codenum = get_code_number(cmd);
        if (codenum == MOVE_ON){
            WRITE(output_fd, cmd);
            continue;
        }
        // check the code number next to the 'G' char
        switch (codenum){
            case 0: case 1:
                G0_G1_gcode();
                break;
            case 5: case 28:
                break;
            default:
                // intact code
                WRITE(output_fd, cmd);
                break;
        }

    }
    return;

}




int main() {
    FILE* f = fopen(ABSOLUTE_INPUT_PATH, "w");
    fprintf(f, "%s\n", "WWWWWWWW");
    fclose(f);
    return EXIT_SUCCESS;
}