#!/usr/bin/env python
#
# Node using cone_finder/location messages and
# Publishes /mavros_msgs/OverrideRCIn messages
#
import sys, argparse, rospy
from mavros_msgs.msg import OverrideRCIn
from cone_finder.msg import location_msgs as Locations

msg = """
cone_seeker: reading cone_finder location messages and 
             publishing OverrideRCIn messages
"""

#settings = termios.tcgetattr(sys.stdin)
hard_limits = [1000, 1500, 2000]  # microseconds for servo signal
steering_limits = [1000, 1500, 2000]  # middle is neutral
throttle_limits = [1200, 1500, 1800]  # middle is neutral
 
# We will get angle between +pi/2 to -pi/2 for steering
# We will get 480 pixels range for throttle but should limit this
class Args(object):
    throttle_step = 0.5
    steering_step = 60

args = Args()

def seek_cone(loc):
    # Sort the poses by y distance to get the nearest cone
    poses = sorted(loc.poses, key=lambda loc: loc.y)
    cone_loc = poses[0]
    # If cone is to the right
    steering = steering_limits[1] + args.steering_step*cone_loc.theta
    y = cone_loc.y - 80
    throttle = throttle_limits[1]
    if(y > 0):
        throttle = throttle + args.throttle_step*(y)
    
    rospy.loginfo('Values = %d %d' % (steering, throttle))
    rc = OverrideRCIn()
    rc.channels = [int(steering), 0, int(throttle), 0, 0, 0, 0, 0]
    pub.publish(rc)

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Drive to cone found')
    parser.add_argument('--throttle_step', '-t', type=int, help='Throttle step size')    
    parser.add_argument('--steering_step', '-s', type=int, help='Steering step size')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
        pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.init_node('cone_seeker', anonymous=True)
        rospy.Subscriber('/cone_finder/locations', Locations, seek_cone)
        rospy.loginfo('cone_seeker init.')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        
