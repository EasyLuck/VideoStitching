#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys, select, termios, tty

# msg = """
# Control The Robot!
# ---------------------------
# Moving around:
#    u    i    o
#    j    k    l
#    m    ,    .

# q/z : increase/decrease max speeds by 10%
# w/x : increase/decrease only linear speed by 10%
# e/c : increase/decrease only angular speed by 10%
# space key, k : force stop
# anything else : stop smoothly

# CTRL-C to quit
# """

msg ="""
uav1 :
           move          yaw         up down
            w           z   x        q    e
        a       d
            s 
uav2 :
           move          yaw         up down
            i           n   m        u    o
        j       l
            k 
"""

uav1SpeedBindings = {
        'd':(1,0,0),
        'a':(-1,0,0),
        'w':(0,1,0),
        's':(0,-1,0),
        'q':(0,0,1),
        'e':(0,0,-1)
}
uav1AttitudeBindings = {
        'z':(0,0,0.2),
        'x':(0,0,-0.2)
}

uav2SpeedBindings = {
        'l':(1,0,0),
        'j':(-1,0,0),
        'i':(0,1,0),
        'k':(0,-1,0),
        'u':(0,0,1),
        'o':(0,0,-1)
}
uav2AttitudeBindings = {
        'n':(0,0,0.2),
        'm':(0,0,-0.2)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('uav_teleop')
    uav1SetVelocity_pub =  rospy.Publisher("uav1/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=1)
    uav2SetVelocity_pub =  rospy.Publisher("uav2/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=1)
    count = 0
    uav1_vx = 0
    uav1_vy = 0
    uav1_vz = 0
    uav1_yaw = 0 
    uav2_vx = 0
    uav2_vy = 0
    uav2_vz = 0
    uav2_yaw = 0 
    try:
        print msg
        # print vels(speed,turn)
        while(1):
            key = getKey()
            if key in uav1SpeedBindings.keys():
                print key
                uav1_vx = uav1SpeedBindings[key][0]
                uav1_vy = uav1SpeedBindings[key][1]
                uav1_vz = uav1SpeedBindings[key][2]
                count = 0
                print("uav1 publihsed: vx: {0}, vy: {1}, vz: {2}".format(uav1_vx, uav1_vy, uav1_vz))
            elif key in uav1AttitudeBindings.keys():
                print key
                uav1_yaw = uav1AttitudeBindings[key][2]
                count = 0
                print("uav1 publihsed: yaw: {0}".format(uav1_yaw))      
            elif key in uav2SpeedBindings.keys():
                print key
                uav2_vx = uav2SpeedBindings[key][0]
                uav2_vy = uav2SpeedBindings[key][1]
                uav2_vz = uav2SpeedBindings[key][2]
                count = 0
                print("uav2 publihsed: vx: {0}, vy: {1}, vz: {2}".format(uav2_vx, uav2_vy, uav2_vz))
            elif key in uav2AttitudeBindings.keys():
                print key
                uav2_yaw = uav2AttitudeBindings[key][2]
                count = 0
                print("uav2 publihsed: yaw: {0}".format(uav2_yaw))                   

            elif key == ' ' or key == 'k' :
                uav1_vx = 0
                uav1_vy = 0
                uav1_vz = 0
                uav1_yaw = 0
                uav2_vx = 0
                uav2_vy = 0
                uav2_vz = 0
                uav2_yaw = 0 
            else:
                count = count + 1
                if count > 4:
                    uav1_vx = 0
                    uav1_vy = 0
                    uav1_vz = 0
                    uav1_yaw = 0
                    uav2_vx = 0
                    uav2_vy = 0
                    uav2_vz = 0
                    uav2_yaw = 0 
                if (key == '\x03'):
                    break

            uav1SetVelocity = TwistStamped()
            uav1SetVelocity.twist.linear.x = uav1_vx
            uav1SetVelocity.twist.linear.y = uav1_vy
            uav1SetVelocity.twist.linear.z = uav1_vz
            uav1SetVelocity.twist.angular.z = uav1_yaw
            uav1SetVelocity_pub.publish(uav1SetVelocity)

            uav2SetVelocity = TwistStamped()
            uav2SetVelocity.twist.linear.x = uav2_vx
            uav2SetVelocity.twist.linear.y = uav2_vy
            uav2SetVelocity.twist.linear.z = uav2_vz
            uav2SetVelocity.twist.angular.z = uav2_yaw
            uav2SetVelocity_pub.publish(uav2SetVelocity)
            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print 111

    finally:
        uav1SetVelocity = TwistStamped()
        uav1SetVelocity.twist.linear.x = uav1_vx
        uav1SetVelocity.twist.linear.y = uav1_vy
        uav1SetVelocity.twist.linear.z = uav1_vz
        uav1SetVelocity.twist.angular.z = uav1_yaw
        setVelocity_pub.publish(uav1SetVelocity)
        uav2SetVelocity = TwistStamped()
        uav2SetVelocity.twist.linear.x = uav2_vx
        uav2SetVelocity.twist.linear.y = uav2_vy
        uav2SetVelocity.twist.linear.z = uav2_vz
        uav2SetVelocity.twist.angular.z = uav2_yaw
        uav2SetVelocity_pub.publish(uav2SetVelocity)
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
