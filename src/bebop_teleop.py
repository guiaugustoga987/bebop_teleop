#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

count_tilt = 0
tilt_exceed = False

count_pan = 0
pan_exceed = False


msg = """
Reading from the keyboard and Publishing to Bebop Drone!
--------------------------------------------------------
Moving around:                   Camera control:
   u    i    o				t
   j    k    l			     f  g  h
   m    ,    .				b

1 : Take off
2 : Landing
3 : Raise up (+z)
4 : Raise down (-z)

anything else : stop/reset

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Simultaneously press 'shift' button while operating camera
will set camera to max value : (h = -80/80, v = -50/50)

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        '3':(0,0,1,0),
        '4':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

cameraBindings={
        'T':(80,0),
        'B':(-80,0),
        'F':(0,-50),
        'H':(0,50),
	'G':(0,0),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    pub_land = rospy.Publisher('bebop/land', Empty, queue_size = 1)
    empty_msg = Empty()

    pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size = 1)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    tilt = 0
    pan = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

	    elif key in cameraBindings.keys():
                tilt = cameraBindings[key][0]
                pan = cameraBindings[key][1]

	    elif key == 't':

		if count_tilt == -80:
			tilt_exceed = False

		if tilt_exceed == False:
			count_tilt += 1

		tilt = count_tilt

		if count_tilt >= 80:
			tilt_exceed = True

	    elif key == 'b':

		if count_tilt == 80:
			tilt_exceed = False

		if tilt_exceed == False:
			count_tilt -= 1

		tilt = count_tilt

		if count_tilt <= -80:
			tilt_exceed = True

	    elif key == 'f':

		if count_pan == 50:
			pan_exceed = False

		if pan_exceed == False:
			count_pan -= 1

		pan = count_pan

		if count_pan <= -50:
			pan_exceed = True

	    elif key == 'h':

		if count_pan == -50:
			pan_exceed = False

		if pan_exceed == False:
			count_pan += 1

		pan = count_pan

		if count_pan >= 50:
			pan_exceed = True

	    elif key == '1':
                pub_takeoff.publish(empty_msg)
		print("Drone has been take off, fly carefully!")
    	    elif key == '2':
                pub_land.publish(empty_msg)
		print("Drone has landing successfully!")

            else:
                x = 0
                y = 0
                z = 0
                th = 0
		tilt = 0
		pan = 0
		count_tilt = 0
		count_pan = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
	    pub.publish(twist)

	    cam_twist = Twist()
	    cam_twist.angular.y = tilt; cam_twist.angular.z = pan;
	    pub_camera.publish(cam_twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	pub.publish(twist)
	
	cam_twist = Twist()
	cam_twist.angular.y = 0; cam_twist.angular.z = 0;
	pub_camera.publish(cam_twist)
        

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
