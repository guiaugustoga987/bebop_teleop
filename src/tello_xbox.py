#!/usr/bin/env python3

from __future__ import print_function
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import pygame
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

count_tilt = 0
tilt_exceed = False

count_pan = 0
pan_exceed = False


pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

clock = pygame.time.Clock()
#textPrint = TextPrint()

moveBindings = {
    'fwd':(0,1,0,0),
    'bk':(0,-1,0,0),
    'lft':(-1,0,0,0),
    'rgt':(1,0,0,0),
    'UP':(0,0,1,0),
    'DOWN':(0,0,-1,0),
    'CLOCKWISE':(0,0,0,1),
    'COUNTER_CLOCKWISE':(0,0,0,-1),
    }


joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    print(joystick.get_name())


def command(char):
    x = moveBindings[char][0]
    y = moveBindings[char][1]
    z = moveBindings[char][2]
    th = moveBindings[char][3]
    return x,y,z,th

def vely(char):
    y = moveBindings[char][1]
    return y 

def velx(char):
    x = moveBindings[char][0]
    return x

def velz(char):
    z = moveBindings[char][2]
    return z

def velth(char):
    th = moveBindings[char][3]
    return th

def increase_speed(vel):
    speed = vel+0.1
    if speed > 1:
        speed = 1
    if speed < 0:
        speed = 0
    
    return speed

def decrease_speed(vel):
    speed = vel-0.1
    if speed > 1:
        speed = 1
    if speed < 0:
        speed = 0
    
    return speed

if __name__=="__main__":
    
    settings = termios.tcgetattr(sys.stdin)
    
    pub = rospy.Publisher('tello/cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    pub_takeoff = rospy.Publisher('tello/takeoff', Empty, queue_size = 1)
    pub_land = rospy.Publisher('tello/land', Empty, queue_size = 1)
    empty_msg = Empty()
    
    
    #speed = rospy.get_param("~speed", 0.5)
    #turn = rospy.get_param("~turn", 1.0)
    speed = 0.5
    turn = 0.5

    try :
        while(1):
            #joystick_count = pygame.joystick.get_count()
            x = 0
            y = 0
            z = 0
            th = 0


            events = pygame.event.get()
            for event in events:
                A = j.get_button(0)
                B = j.get_button(1)
                X = j.get_button(2)
                Y = j.get_button(3)

                LB = j.get_button(4)
                RB = j.get_button(5)
                LT = j.get_button(6)
                RT = j.get_button(7)
                start = j.get_button(9)
                L = j.get_button(11)
                R = j.get_button(12)

                fwd = j.get_button(15)
                bk = j.get_button(16)
                lft = j.get_button(13)
                rgt = j.get_button(14)
                '''
                if event.type == pygame.JOYBUTTONDOWN:
                    if j.get_button(1):
                        #print("x")
                        a = 1
                    elif j.get_button(2):
                        b = 2
                elif event.type == pygame.JOYBUTTONUP:
                    a = 0
                    b = 0
                    #print("button released")
                '''

            #print('fwd',fwd)
            #print('bk',bk)
            #print('lft',lft)
            #print('rgt',rgt)


            if X == 1:
                pub_takeoff.publish(empty_msg)
                print("Drone has been take off, fly carefully!")
            if Y == 1:
                pub_land.publish(empty_msg)
                print("Drone has landing successfully!")
            if fwd == 1:
                y = vely('fwd')
            if bk == 1:
                y = vely('bk')
            if lft == 1:
                x = velx('lft')
            if rgt == 1:
                x = velx('rgt')
            if B == 1:
                z = velz('UP')
            if A == 1:
                z = velz('DOWN')
            if LB == 1:
                th = velth('COUNTER_CLOCKWISE')
            if RB == 1:
                th = velth('CLOCKWISE')
            if R == 1:
                speed = decrease_speed(speed)
                print("Decrease Speed : ",speed)
            if L == 1:
                speed = increase_speed(speed)
                print("Increase Speed : ",speed)
            if start == 1:
                print("Shutting down the node!")
                pygame.quit()
                sys.exit()


            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

            #print('x',x)
            #print('y',y)
            #print('z',z)
            #print('th',th)  
            clock.tick(20)

    except Exception as e:
        print(e)


    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0 
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0 
        twist.angular.z = 0
        pub.publish(twist)



        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#pygame.quit()

        

