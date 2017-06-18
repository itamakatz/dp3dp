#############################################################
# FILE: refiner.py
# UPDATED: 18.6.17
#############################################################

import os


REFINE_LEVEL = 15


g_current_R = 3
g_current_theta = 0

g_code_pointer = -1

g_refined_commands = []
g_command_args = []


def code_value_float(arg):
    return float(arg[g_code_pointer + 1:])


def code_seen(code, char):
    global g_code_pointer
    g_code_pointer = code.find(char)
    return g_code_pointer >= 0



def refine(path):
    global g_refined_commands, g_current_R, g_current_theta
    res = open("{}_refined.gcode".format(os.path.splitext(path)[0]), "w")

    for gcode in open(path, "r").readlines():
        if not code_seen(gcode, "G1"):
            g_refined_commands.append(gcode.replace("\n", ""))
            continue
        untouched_args = []
        next_R_position, next_theta_position = g_current_R, g_current_theta
        command_args = list(map(lambda x: x.strip(), gcode.replace("\n", "").split(" ")))[1:]

        affected = False

        for arg in command_args:
            if code_seen(arg, "X"):
                next_R_position = code_value_float(arg)
                affected = True
            elif code_seen(arg, "Y"):
                next_theta_position = code_value_float(arg)
                affected = True
            else:
                untouched_args.append(arg)

        if not affected:
            g_refined_commands.append(gcode.replace("\n", ""))
            continue

        R_delta = float(next_R_position - g_current_R) / REFINE_LEVEL
        theta_delta = float(next_theta_position - g_current_theta) / REFINE_LEVEL

        for i in range(REFINE_LEVEL):
            g_refined_commands.append("G1 X{R} Y{theta} ".format(
                R = g_current_R, theta = g_current_theta*g_current_R) + " ".join(untouched_args))
            g_current_R += R_delta
            g_current_theta += theta_delta

    res.write("\n".join(g_refined_commands))
    res.close()
    return



if __name__ == "__main__":
    refine("cs_updated.gcode")