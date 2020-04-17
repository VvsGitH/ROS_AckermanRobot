#!/usr/bin/env python

# WITH THIS SCRIPT YOU CAN CONTROL THE MOVEMENT OF THE ROBOT WITH THE KEYBOARD.
# IT IMPLEMENTS THE ACKERMAN STEERING.
# THIS SCRIPT PUBLISH TO THREE TOPICS:
#   * THE VELOCITY CONTROLLER TOPIC
#   * THE STEERING CONTROLLER OF THE RIGHT WHEEL
#   * THE STEERING CONTROLLER OF THE LEFT WHEEL 

import rospy
import math
import curses
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

# GLOBAL: Starting message on the console
start_msg = """
Reading from keyboard and publishing to Twist
--------------------------------------------
* Commands: WASD
* To quit press Q or CTLR+C
"""

# GLOBAL KEYS' DICTIONARY: Associates a keyboard key to a vector (x,y,z,theta)
vel_bindings = {
    'w': [3, 3, 3, 3],      # Forward 2m/s
    's': [-3, -3, -3, -3]   # Backward 2m/s
}

steer_bindings = {
    'a': 0.17,  # 10grad anti-clock-wise
    'd': -0.17  # 10grad clock-wise
}

# GLOBAL: MODEL GEOMETRY
L = 0.7  # Distance between wheels axes
W = 0.7  # Distance between wheels


# FUNCTION start_curses(): Start a curses terminal app
def start_curses():
    app = curses.initscr()  # Create a terminal window
    curses.noecho()         # Makes input invisible
    app.addstr(start_msg)   # Print the start message
    return app


def init_msgs():
    # Velocity Reference
    vel_ref = Float64MultiArray()
    vel_ref.layout.data_offset = 0
    vel_ref.layout.dim.append(MultiArrayDimension())
    vel_ref.layout.dim[0].label = "ctrl_signal"
    vel_ref.layout.dim[0].size = 4
    vel_ref.layout.dim[0].stride = 4
    vel_ref.data = [0.0, 0.0, 0.0, 0.0]
    return vel_ref


def ackrm_ctrl(theta):
    global L, W
    Rsteer_ref = Float64()
    Lsteer_ref = Float64()
    if theta > 0:
        d = L/math.tan(theta)
        alpha = math.atan(L/(d+W))
        Lsteer_ref.data = theta
        Rsteer_ref.data = alpha
    elif theta < 0:
        theta_abs = abs(theta)
        d = L/math.tan(theta_abs)
        alpha_abs = math.atan(L/(d+W))
        alpha = - alpha_abs
        Lsteer_ref.data = alpha
        Rsteer_ref.data = theta
    else:
        Lsteer_ref.data = theta
        Rsteer_ref.data = theta
    return Rsteer_ref, Lsteer_ref


# MAIN FUNCTION
def move():
    # Defining the topic to publish on
    pub_vel = rospy.Publisher(
        '/ackrm/wheels_vel_ctrl/command', Float64MultiArray, queue_size=5)
    pub_Lsteer = rospy.Publisher(
        '/ackrm/l_steer_ctrl/command', Float64, queue_size=5)
    pub_Rsteer = rospy.Publisher(
        '/ackrm/r_steer_ctrl/command', Float64, queue_size=5)

    # Defining the name of the node represented by this script
    rospy.init_node('ackrm_robot_teleop', anonymous=True)

    # Setting the rate of publications at 30hz
    rate = rospy.Rate(30)

    # Initializing velocity msg
    vel_ref = init_msgs()
    theta = 0.0

    # Starting and configuring Curses application
    app = start_curses()

    # MAIN WHILE LOOP
    while not rospy.is_shutdown():

        # Reading the pressed key from the curses app
        key = app.getkey()

        # Selecting correct speed and angle based on the pressed key
        if key in vel_bindings.keys():
            vel_ref.data = vel_bindings[key]
            theta = 0.0  # Auto return of the steering wheels
        elif key in steer_bindings.keys():
            theta = theta + steer_bindings[key]
        else:
            # Incorrect key => Robot stop
            vel_ref.data = [0.0, 0.0, 0.0, 0.0]
            theta = 0.0
            # q => exit from loop
            if (key == 'q'):
                curses.endwin()  # End Curses application
                break

        # Calculating wheel's steering angle
        Rsteer_ref, Lsteer_ref = ackrm_ctrl(theta)

        # Publishing the messages
        pub_vel.publish(vel_ref)
        pub_Rsteer.publish(Rsteer_ref)
        pub_Lsteer.publish(Lsteer_ref)
        # Pause the loop
        rate.sleep()


# SIMPLE SCRIPT MAIN WITH ERROR EXCEPTION
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
    except curses.error:
        curses.endwin()
