#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3Stamped

from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

attonly = True # Set to True for to send sp based on time rather than position
attcount = 0
omnipose = None
linecount = 0
acc_rad = 0.1
sp = PositionTarget()
sp.coordinate_frame = sp.FRAME_BODY_NED
sp.type_mask = sp.IGNORE_YAW|sp.IGNORE_YAW_RATE
omerr = PoseStamped()

def traj_pub(trajfile, length):
    global linecount
    global attcount
    global attonly
    # Load position, acceleration, and attitude setpoint from traj file
    sp = getSetpoint(trajfile, linecount)

    # Initialize ROS node (for publishing/subscribing)
    rospy.init_node('quad_trajectory', anonymous=True)

    ### Trajectory Publishers ###
    # Position and acceleration setpoint publisher
    pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Position error publisher (for recording to bagfiles)
    puberr = rospy.Publisher('omnipos/error', PoseStamped, queue_size=1)

    # Local pose subscriber, to check when vehicle reaches a setpoint
    sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback)

    # Set publish/subscribe rate
    rate = rospy.Rate(50)  # 50hz


    # Publish preliminary setpoints before arming to enable PX4 OFFBOARD mode
    for k in range(50):
        pub.publish(sp)
        rate.sleep()
    rospy.loginfo("Finished publishing preliminary setpoints")

    # Arm the vehicle and switch to OFFBOARD mode
    setArm(True)
    rospy.loginfo('Arming')
    setOffboard()


    while not rospy.is_shutdown():
        # Publish setpoints
        pub.publish(sp)

        # Get current position, setpoint position, and error betwen them
        x = omnipose.position.x
        y = omnipose.position.y
        z = omnipose.position.z
        spx = sp.position.x
        spy = sp.position.y
        spz = sp.position.z
        errx = spx - x
        erry = spy - y
        errz = spz - z

        # Save errors to position error message and then publish
        omerr.pose.position.x = errx
        omerr.pose.position.y = erry
        omerr.pose.position.z = errz
        puberr.publish(omerr)

        # Debug outputs
        rospy.loginfo('Traj file length: %s, linecount: %s', length, linecount)
        rospy.loginfo('X error: %s m', errx)
        rospy.loginfo('Y error: %s m', erry)
        rospy.loginfo('Z error: %s m', errz)

        # Check if vehicle has reached a setpoint to within the acceptance radius
        # and get the next setpoint if so. If the vehicle reaches the end of the
        # trajectory file, repeat from the beginning, ignoring the first setpoint
        if not attonly:
            if ((abs(errx) < acc_rad) and (abs(erry) < acc_rad) and (abs(errz) < acc_rad)):
                if (linecount == length):
                    linecount = 1
                else:
                    linecount = linecount+1
                rospy.loginfo('Arrived at setpoint')
                sp = getSetpoint(trajfile, linecount)
        else:
            if (attcount == 25):
                if (linecount == length):
                    linecount = 1 #change to 1 or 0 if there is no lead in into the trajectory, right now is 6 if there is a leadin to the trajectory
                else:
                    linecount = linecount+1
                rospy.loginfo('Setpoint changed')
                sp = getSetpoint(trajfile, linecount)
                attcount = 0
            else:
                attcount = attcount + 1
        # Sleep for the amount of time specified in the rate command above
        rate.sleep()



def setArm(armed):
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        armService(armed)
    except rospy.ServiceException:
        rospy.loginfo("Service arming call failed")

def setOffboard():
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rospy.wait_for_service('/mavros/set_mode')
    try:
        base_mode = 0
        custom_mode = "OFFBOARD"
        out = change_mode(base_mode, custom_mode)
        if out:
            rospy.loginfo("Successfully changed to offboard mode")
    except rospy.ServiceException:
        rospy.loginfo("Service call failed")

def callback(data):
    global omnipose
    omnipose = data.pose

def getSetpoint(trajfile, linecount):
    data = trajfile[linecount] #This is where you read in textfile with setpoint data
    datasplit = data.split(",")
    datasplit[2] = datasplit[2].rstrip()
    datarray = np.array(map(float, datasplit))
    sp.position.x = 0
    sp.position.y = 0
    sp.position.z = 1
    sp.velocity.x = 0
    sp.velocity.y = 0
    sp.velocity.z = 1
    sp.acceleration_or_force.x = 0
    sp.acceleration_or_force.y = 0
    sp.acceleration_or_force.z = 1
    # sp.pose.orientation.x = 0
    # sp.pose.orientation.y = 0
    # sp.pose.orientation.z = 0
    # sp.pose.orientation.w = 1
    #sp.pose.yaw = datarray[3]

    return sp

if __name__ == '__main__':
    attitude_flag = True
    testfile = open(r'/home/omni/PX4-Autopilot/launch/quad_traj/scripts/traj_move_up.txt')
    lines = testfile.readlines()
    linelen = len(lines) - 1
    try:
        traj_pub(lines, linelen)
    except rospy.ROSInterruptException:
        pass
