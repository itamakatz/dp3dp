#############################################################
# FILE: slicer.py
# UPDATED: 17.5.17
#############################################################


import math
import sys
import os

g_updated_commands = []
g_command_args = []

g_current_X_position = 3
g_current_Y_position = 0

g_current_R_position = 3
g_current_alpha_position = 0

printer_X_origin = 0
printer_Y_origin = 0

g_code_pointer = -1

#####################################################


class switch(object):
    value = None

    def __new__(cls, value):
        cls.value = value
        return True


def case(*args):
    return any((arg == switch.value for arg in args))


def relpath(filename):
    return os.path.join(os.path.dirname(__file__), filename)


def join(it):
    return ("\n" if it is g_updated_commands else " ").join(it)


def change_coordinates(X, Y):

    """
    we define the axes as follows:
        ^
        |           we "simulate" the run of the gcode file,
        |           to know how to convert our axis.
        |
        |           if the new angle is smaller than the current angle (relative to the origins),
        |           we set the motion clockwise, otherwise CCW.
        |
        |  <---|
        |      |  <== alpha
        |      |
        ------------------------>
                            =======> R

    """
    global g_current_R_position, g_current_alpha_position
    # compute the radius
    R = math.hypot(X, Y)
    # compute the arc length, by the radius and the angle
    # if it makes trouble, comment it and uncomment the next line
    alpha = math.atan2(Y, X)
    # alpha = math.degrees(math.atan2(Y, X))

    g_current_R_position = R
    g_current_alpha_position = alpha

    return R, alpha



def R_will_change_direction(point0, point1, point2):
    """
    to attack the case in which we should change the direction of R move


                                        point1 (current arm position)
                                            *
                                            |
                                            |
                                          __|
                                         |  |
                    *-----------------------|  (x3, y3)  <==  until this point we should shorter R,
          point0 (origin)                   |               and from this point we should longer R.
                                            |
                                            |
                                            |
                                            |
                                            *
                                        point2 (next arm position)
    """

    x0, y0 = point0[0], point0[1]
    x1, y1 = point1[0], point1[1]
    x2, y2 = point2[0], point2[1]

    try:
        m1 = (x1 - x2) / (y2 - y1)
        m2 = (y2 - y1) / (x2 - x1)
        x3 = ((m2 * x1) - (m1 * x0) - y1 + y0) / (m2 - m1)
        y3 = m1 * (x3 - x0) + y0
    except ZeroDivisionError:
        (x3, y3) = (x0, y1) if y1 == y2 else (x1, y0)

    return ((min(x1, x2) <= x3 <= max(x1, x2)) and (min(y1, y2) <= y3 <= max(y1, y2))), (x3, y3)


def code_seen(code, char):
    global g_code_pointer
    g_code_pointer = code.find(char)
    return g_code_pointer >= 0


def G0_G1_gcode():

    global g_current_X_position, \
           g_current_Y_position, \
           g_updated_commands, \
           g_command_args

    untouched_args             = []
    next_X_position            = None
    next_Y_position            = None
    X_continuation_position    = None
    Y_continuation_position    = None
    continuation_command       = False

    for arg in g_command_args[1:]:
        if code_seen(arg, 'X'):
            next_X_position = float(arg[g_code_pointer + 1:])
        elif code_seen(arg, 'Y'):
            next_Y_position = float(arg[g_code_pointer + 1:])
        elif code_seen(arg, 'I'):
            X_continuation_position = next_X_position + float(arg[g_code_pointer + 1:])
            continuation_command = True
        elif code_seen(arg, 'J'):
            Y_continuation_position = next_Y_position + float(arg[g_code_pointer + 1:])
            continuation_command = True
        else:
            untouched_args.append(arg)

    if next_X_position is None and next_Y_position is None:
        g_updated_commands.append(join(g_command_args))
        return

    # check if they even appeared in the command
    next_X_position = next_X_position if next_X_position is not None else g_current_X_position
    next_Y_position = next_Y_position if next_Y_position is not None else g_current_Y_position

    # if it is a straight line, R needs to be changes during its move
    R_will_change, point = R_will_change_direction((printer_X_origin, printer_Y_origin),
                                                   (g_current_X_position, g_current_Y_position),
                                                   (next_X_position, next_Y_position))
    if R_will_change:
        R, alpha = change_coordinates(point[0], point[1])
        command = "G1 X{R} Y{alpha} ".format(R = R, alpha = alpha) + join(untouched_args)
        g_updated_commands.append(command)
        g_current_X_position, g_current_Y_position = point[0], point[1]

    # after that, continue from the same spot
    R, alpha = change_coordinates(next_X_position, next_Y_position)
    command = "G1 X{R} Y{alpha} ".format(R = R, alpha = alpha) + join(untouched_args)
    g_updated_commands.append(command)
    g_current_X_position, g_current_Y_position = next_X_position, next_Y_position

    # if I or J appeared, we need to add one more command. simply rewind this function
    if continuation_command:
        current_command = join(g_command_args)
        command = "G1 "
        command += "X{} ".format(X_continuation_position if code_seen(current_command, 'I') else g_current_X_position)
        command += "Y{} ".format(Y_continuation_position if code_seen(current_command, 'J') else g_current_Y_position)
        g_command_args = (command + join(untouched_args)).split(" ")
        G0_G1_gcode()

    return



def parse(path):

    global g_command_args
    res = open("{}_updated.gcode".format(os.path.splitext(path)[0]), "w")

    for gcode in open(path, "r").readlines():
        # if its not a command, pass
        if not code_seen(gcode, "G"):
            g_updated_commands.append(gcode.replace("\n", ""))
            continue
        g_command_args = list(map(lambda x: x.strip(), gcode.replace("\n", "").split(" ")))
        while switch(g_command_args[0]):
            if case("G0", "G1"):
                G0_G1_gcode()
                break
            if case("G28"):
                # make a new endstop
                g_updated_commands.append("G1 X3 Y0")
                break
            # default, the code is intact
            g_updated_commands.append(join(g_command_args))
            break

    res.write(join(g_updated_commands))
    res.close()
    return


def check():
    a = g_command_args
    print(a is g_command_args)
    print(str.join(" ", a))


if __name__ == "__main__":
    # check()
    parse("csv6.gcode")
    # parse(relpath(sys.argv[1]))

