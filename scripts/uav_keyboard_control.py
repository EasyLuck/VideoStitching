#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point

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

dx = 0.5
dy = 0.5
dz = 0.3

uav1SetVelocity = TwistStamped()
uav2SetVelocity = TwistStamped()

uav1Position = [0,0,3]  # [x y z]
uav2Position = [3,0,3]  # [x y z]

uav1AttitudeBindings = {
        'z':(0,0,0.2),
        'x':(0,0,-0.2)
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
    uav1SetPosition_pub =  rospy.Publisher("uav1/setTarget_position",Point,queue_size=1)
    uav2SetVelocity_pub =  rospy.Publisher("uav2/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=1)
    uav2SetPosition_pub =  rospy.Publisher("uav2/setTarget_position",Point,queue_size=1)


    count = 0
    uav1_yaw = 0 
    uav2_yaw = 0 

    try:
        print msg
        # print vels(speed,turn)
        while(1):
            key = getKey()
            if key == 'a' :
                count = 0
                uav1Position[0] = uav1Position[0] - dx
            elif key == 'd' :   
                count = 0
                uav1Position[0] = uav1Position[0] + dx
            elif key == 'w' :
                count = 0
                uav1Position[1] = uav1Position[1] + dy
            elif key == 's' :
                count = 0                  
                uav1Position[1] = uav1Position[1] - dy
            elif key == 'q' :
                count = 0
                uav1Position[2] = uav1Position[2] + dz
            elif key == 'e' :  
                count = 0 
                uav1Position[2] = uav1Position[2] - dz
            elif key in uav1AttitudeBindings.keys():
                print key
                uav1_yaw = uav1AttitudeBindings[key][2]
                count = 0
                uav1SetVelocity.twist.angular.z = uav1_yaw
                uav1SetVelocity_pub.publish(uav1SetVelocity)   

            elif key == 'j' :
                count = 0
                uav2Position[0] = uav2Position[0] - dx
            elif key == 'l' :   
                count = 0
                uav2Position[0] = uav2Position[0] + dx
            elif key == 'i' :
                count = 0
                uav2Position[1] = uav2Position[1] + dy
            elif key == 'k' :
                count = 0                  
                uav2Position[1] = uav2Position[1] - dy
            elif key == 'u' :
                count = 0
                uav2Position[2] = uav2Position[2] + dz
            elif key == 'o' :  
                count = 0 
                uav2Position[2] = uav2Position[2] - dz
            elif key in uav2AttitudeBindings.keys():
                print key
                uav2_yaw = uav2AttitudeBindings[key][2]
                count = 0
                uav2SetVelocity.twist.angular.z = uav2_yaw
                uav2SetVelocity_pub.publish(uav2SetVelocity)
                                   

            elif key == ' ' or key == 'k' :
                uav1_yaw = 0
                uav2_yaw = 0 
            else:
                count = count + 1
                if count > 4:
                    uav1_yaw = 0
                    uav2_yaw = 0 
                if (key == '\x03'):
                    break
            if count == 0 or count > 20:
                print "-----------------------------------------------"
                print("uav1 target position: x: {0}, y: {1}, z:{2}".format(uav1Position[0], uav1Position[1],uav1Position[2]))
                print("uav1 target yaw : {0}".format(uav1_yaw))  
                print("uav2 target position: x: {0}, y: {1}, z:{2}".format(uav2Position[0], uav2Position[1],uav2Position[2]))
                print("uav2 target yaw : {0}".format(uav2_yaw))  
                count = 0
            uav1SetPosition = Point()
            uav1SetPosition.x = uav1Position[0]
            uav1SetPosition.y = uav1Position[1]
            uav1SetPosition.z = uav1Position[2]
            uav1SetPosition_pub.publish(uav1SetPosition)

            uav2SetPosition = Point()
            uav2SetPosition.x = uav2Position[0]
            uav2SetPosition.y = uav2Position[1]
            uav2SetPosition.z = uav2Position[2]
            uav2SetPosition_pub.publish(uav2SetPosition)





            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print 111

    finally:
        uav1SetVelocity = TwistStamped()
        uav1SetVelocity.twist.angular.z = uav1_yaw
        setVelocity_pub.publish(uav1SetVelocity)
        uav2SetVelocity = TwistStamped()
        uav2SetVelocity.twist.angular.z = uav2_yaw
        uav2SetVelocity_pub.publish(uav2SetVelocity)
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
