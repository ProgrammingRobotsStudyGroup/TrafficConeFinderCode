#!/usr/bin/env python
#
# Node using cone_finder/location messages and
# Publishes /mavros_msgs/OverrideRCIn messages
#
import sys, select, termios, tty
from numpy import interp
import rospy
from mavros_msgs.msg import OverrideRCIn
from cone_finder.msg import location_msgs as Locations

msg = """
cone_seeker: reading cone_finder location messages and 
             publishing OverrideRCIn messages
------------------------------------------------------
Throttle:
    e    increase 5%
    d    set to zero
    c    decrease 5%
    w    increase max throttle 5%
    x    decrease max throttle 5%
    q    increase min throttle 5%
    z    decrease min throttle 5%

Steering:
    j    increase servo L 5%
    k    set steering to neutral
    l    increase servo R 5%
    u    increase max L servo 5%
    m    decrease max L servo 5%
    o    increase max R servo 5%
    .    decrease max R servo 5%

any other key sets throttle and steering neutral

CTRL-C to quit
------------------------------------------------------------
"""

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1000, 1500, 2000]  # middle is neutral
throttle_limits = [1200, 1500, 1800]  # middle is neutral
 
steering = steering_limits[1]  # start neutral
throttle = throttle_limits[1]  # start neutral

# We will get angle between +90 to -90 for steering
STEER_SIZE = 2   # normalized step size for adjustments for steering
STEP_SIZE  = 0.5 # normalized step size for adjustments for throttle

def seek_cone(loc):
    # Sort the poses by y distance to get the nearest cone
    poses = sorted(loc.poses, key=lambda loc: loc.y)
    cone_loc = poses[0]
    # If cone is to the right
    steering = steering_limits[1] + STEER_SIZE*cone_loc.theta
    y = cone_loc.y - 80
    throttle = throttle_limits[1]
    if(y > 0):
        throttle = throttle + STEP_SIZE*(y)
    
    rospy.loginfo('Values = %d %d' % (throttle, steering))
    rc = OverrideRCIn()
    rc.channels = [int(steering), 0, int(throttle), 0, 0, 0, 0, 0]
    pub.publish(rc)

if __name__=="__main__":
    try:
        pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.init_node('cone_seeker', anonymous=True)
        rospy.Subscriber('/cone_finder/locations', Locations, seek_cone)
        rospy.loginfo('cone_seeker init.')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
