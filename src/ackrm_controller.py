#!/usr/bin/env python

""" NON USATO - QUESTO SCRIPT ERA PENSATO PER FARE DA INTERMEDIARIO TRA ACKRM_MOVE E 
GAZEBO. SI SOTTOSCRIVE SIA A AL TOPIC ACKRM_DESIRED_STEER, CHE PRIMA ERA CREATO DA
ACKRM_MOVE, SIA A JOINT STATE. L'IDEA ERA DI PRENDERE L'ANGOLO DI STERZATA DESIDEATO
E ELABORARE GLI ANGOLI DELLE RUOTE ESTRAENDO LA GEOMETRIA DIRETTAMENTE DA JOINT STATE.
A POSTERIORI, DATO CHE SI PARLA DI COSTANTI SAREBBE STATO MEGLIO USARE /TF. IN OGNI CASO
QUESTO APPROCCIO COMPLESSO NON E' STATO UTILIZZATO.
TUTTAVIA LO SCRIPT RIMAMANE, IN QUANTO E' UN OTTIMO ESEMPIO DI COME BISOGNA FARE PER
SINCRONIZZARE I MESSAGGI PROVENIENTI DA DUE TOPIC, ELABORARE I DATI E INVIARLI SU ALTRI
DUE TOPIC 
NB: COMPRENDE ANCHE L'UTILIZZO DEL MESSAGGIO CUSTOM HEADERFLOAT64, CREATO PER FORNIRE
I FLOAT64 DI UN HEADER, IN MODO DA POTER ESSERE SINCRONIZZATI """


import random
import math
import rospy
import message_filters  # Topi syncronization
from ackrm_robot.msg import HeaderFloat64 # Custom message
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# GLOBAL - TOPIC TO PUBLISH TO
pub_Rsteer = rospy.Publisher(
    '/ackrm/r_steer_ctrl/command', Float64, queue_size=1)
pub_Lsteer = rospy.Publisher(
    '/ackrm/l_steer_ctrl/command', Float64, queue_size=1)


# Create a dictionary to organize useful info from JointState message
def create_joint_dict(joint_data):  # data of type JointState
    joint_dict = {}
    for i in range(len(joint_data.name)):
        joint_dict[joint_data.name[i]] = joint_data.position[i]
    return joint_dict


# Extract geometry from joint state dictionary
def geometry(joint_dict):
    #### TO DO ####
    l = 0.0
    w = 0.0
    return l, w


# Ackerman controller - Calculate wheels' steering angles
def ackerman_ctrl(theta, L, W):
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


# Callback function called when the two topics are synched
def main_callback(theta_data, joint_data):
    # Create joint dictionary
    joint_dict = create_joint_dict(joint_data)
    # Log print
    rospy.loginfo(theta_data)
    rospy.loginfo(joint_data)
    # Reconstructing geometry
    l, w = geometry(joint_dict)
    # Controller
    Rsteer_ref, Lsteer_ref = ackerman_ctrl(theta_data.data, l, w)
    # Publish steering angles
    pub_Rsteer.publish(Rsteer_ref)
    pub_Lsteer.publish(Lsteer_ref)


if __name__ == '__main__':
    try:
        rospy.init_node('ackrm_robot_state', anonymous=True)
        # First topic - Desired steering angle from ackrm_move.py
        theta_data = message_filters.Subscriber(
            '/ackrm/desired_steer', HeaderFloat64)
        # Second topic - Joint data from Gazebo
        joint_data = message_filters.Subscriber('/ackrm/joint_states', JointState)
        # Message syncronization
        ats = message_filters.ApproximateTimeSynchronizer(
            [theta_data, joint_data], queue_size=5, slop=0.1)
        ats.registerCallback(main_callback)
        # Loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
