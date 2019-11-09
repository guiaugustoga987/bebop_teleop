# teleop_twist_keyboard
ROS Keyboard Teleop for Parrot Bebop 1 and 2

# Launch
Installation
'''
cd /catkin_ws
git clone 
catkin_make
'''

Run.
```
rosrun bebop_teleop bebop_teleop.py
```

With custom values.
```
rosrun rosrun bebop_teleop bebop_teleop.py _speed:=0.9 _turn:=0.8
```

# Usage
```
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
```

