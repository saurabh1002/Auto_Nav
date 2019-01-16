#!/usr/bin/env python

###################################################################################################################
# CODE TO DRIVE THE ROBOT VIA TELEOPERATION
# PRESS 'w' TO MOVE FORWARD
# PRESS 's' TO MOVE BACKWARD
# PRESS 'k' TO TURN LEFT ON SPOT (DEFAULT : 45 DEGREES)
# PRESS 'l' TO TURN RIGHT ON SPOT (DEFAULT : 45 DEGREES)
# PRESS 'e' TO SET TURN ANGLE TO 15 DEGREES
# PRESS 'q' TO INCREMENT TURN ANGLE BY 15 DEGREES
# PRESS 'a' TO ENABLE AUTONOMOUS NAVIGATION MODE
# PRESS 't' TO DISABLE AUTONOMOUS NAVIGATION MODE
# PRESS 'c' TO RESET ALL VARIABLE
###################################################################################################################

import rospy
from std_msgs.msg import String
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)                                         # read key pressed on keyboard
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    rospy.init_node('base',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('teleop_key', String, queue_size=10)          # publisher for teleop_key
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            key = getKey()
            pub.publish(key)
            rate.sleep()
        except:
            continue
