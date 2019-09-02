#!/usr/bin/env python
##@ University of Michigan benchmark test for odometry error correction
## UMBmark test for AMR
# Findout more at " https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf "

import rospy
import math
from geometry_msgs.msg import Twist  # used for cmd_vel
from nav_msgs.msg import Odometry  # used for odom
from tf.transformations import euler_from_quaternion, quaternion_from_euler
roll =0.0
pitch =0.0
yaw =0.0
quater =0.0 
cangle =0.0
position = 0.0
positxj=0.0
jkl=0.0
errorcollect=[]
gopy=0.0

##@ Our Apporach
# Asking AMR to go straight for 4 distance (calling gostraight function)
# Asking AmR to turn 90 degrees.


def callback(msg):
    global roll, pitch, yaw, quater, cangle,positxj,jkl,gopy
    #positxj= msg.pose.pose.position.x 
    positxj= float(format((msg.pose.pose.position.x),'.2f'))
    gopy= msg.pose.pose.position.y
    jkl= float(format((gopy),'.2f'))
   
    #print ("callback posit x is "+ str(positxj) + "  posit y is " + str(jkl))
    tuioooooo,    
    quater=msg.pose.pose.orientation
    orientation_list = [quater.x, quater.y, quater.z, quater.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    cangle= float(format((yaw *180/math.pi),'.2f'))
    #print(" cangle is " + str(cangle))
## @Turn function
# it rotates the robot to exactly around 90 degrees using closed loop control function
# function world by keeping track of angular odometry, converting quaternion into euler angles in degree & then publishing angular velocity 
# it find the the angular rotation to be moved, by measuring current angle and substracting it from goal angle  and setting it as an error in publishing cmd_vel angular 
# 
def turn(a):
    global roll, pitch, yaw, quater, cangle,position, positxj,positxj
    currentangle = cangle
    goalangle = currentangle + (90*a)
    if goalangle >= 180.00:
        goalangle = 0 - (180 - (goalangle-180))
    if goalangle <= -180.00:
        goalangle = 0 + (180 +(goalangle+180))
        

    print("current angle is " + str(currentangle))         # getting current angle  in degree
        
    pub= rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    rospy.loginfo("turning")
    print("current angle = {} goal angle ={}".format(currentangle,goalangle))
    
    while True:
        #if (currentangle <= mingoalangle or currentangle >= maxgoalangle ):
        rjk=float(format(goalangle-currentangle,'.2f'))
        #print("rjk value is {}".format(rjk))
        if (rjk>0):
            currentangle=cangle
            move_cmd.angular.z =(goalangle-currentangle)*3*math.pi/180
            pub.publish(move_cmd)
            print("if1 target = {}  currentangle= {} Wspeed= {} rjk={}" .format(goalangle,currentangle,move_cmd.angular.z,rjk))
            
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            
        if rjk<0 :
            currentangle=cangle
            #print("turning in oppostite direction")
            move_cmd.angular.z = (goalangle-currentangle)*3 *math.pi/180
            pub.publish(move_cmd)
            print(" if2 target = {}  currentangle= {} wpeed= {} rjk={}".format(goalangle,currentangle,move_cmd.angular.z,rjk))
            
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            
        if rjk==0 :
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            print("turned now breaking")
            break

## @ GO straight function
# it is a closed loop function
# it guides amr to move upto certain distance by publishing cmd-_vel and keeping track of how far it has traversed
# it measure distance treversed using euclidean distance method where one point is initial position and final point is robot current position

def gostraight():

    #global roll, pitch, yaw, quater, cangle,position, positxj,positxj    
    px,py=positxj, jkl
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)   #Making  a publisher
    pub_nothing = Twist()               
    pub_nothing.linear.x = 0
    pub_nothing.angular.z= 0
    
    move_cmd = Twist()
    move_cmd.linear.x = 0.4
    move_cmd.linear.z = 0.0
    while (pub.get_num_connections() == 0):
        print("")
    
    Distance = 0
    
    while 1:
        #gpn=jkl
        #print(gpn)
        cx,cy = positxj,jkl
        if (Distance>2):
            
            move_cmd.angular.x = - 0.3
            pub.publish(move_cmd)
            cx,cy = positxj,jkl
            print("cx value is " + str(positxj) + "cy value is " + str(jkl))
            print()
            
            Distance =float(format(( math.sqrt((px-cx)**2 + (py-cy)**2)),'.1f'))
            print("last pos px,py ={},{} current pos cx,cy = {},{} disp = {} ".format(px,py,cx,cy,Distance))
        
        if (Distance<2):
            move_cmd.angular.x = 0.3
            pub.publish(move_cmd)
            cx,cy = positxj,jkl
            Distance =float(format(( math.sqrt((px-cx)**2 + (py-cy)**2)),'.1f'))
            print("last pos px,py ={},{} current pos cx,cy = {},{} disp = {} ".format(px,py,cx,cy,Distance))
        
        if (Distance==2):
            move_cmd.angular.x =0
            pub.publish(move_cmd)
            print("target position reached")
            break
        
    
    pub.publish(pub_nothing)
    pub.publish(pub_nothing)

rospy.init_node('umbv1', anonymous=True) #making node 
rospy.Subscriber('odom',Odometry,callback)      # making a subscriber of odometry
print("starting up ")
xpos_list=[]
ypos_list=[]
xerror=0
yerror=0
## @ calling sqaure fucntion 5 time  
# calling a square making fucntion to make square while turning in cw direction
for i in range (0,1):
    xpos_list.append(positxj)
    ypos_list.append(jkl)
    for j in range (0,4):
        gostraight()
        turn(1)         # for cw direction

for i in range(0,len(xpos_list)-1):
    xerror += xpos_list[i+1]-xpos_list[i]
    print("errors is x direction = {}" .format(xerror))
print ("total error in x direction is = {}".format(xerror))
cwxerror = xerror
for i in range(0,len(ypos_list)-1):
    yerror += ypos_list[i+1]-ypos_list[i]
    print("errors is y direction = {}" .format(yerror))
print ("total error in y direction is = {}".format(yerror))
cwyerror = yerror

## @ calling sqaure function again 5 time  
# calling a square making fucntion to make square while turning in counter clockcwise direction
for i in range (0,1):
    xpos_list.append(positxj)
    ypos_list.append(jkl)
    for j in range (0,4):
        gostraight()
        turn(-1)   # for ccw direction

for i in range(0,len(xpos_list)-1):
    xerror += xpos_list[i+1]-xpos_list[i]
    print("errors is x direction = {}" .format(xerror))
print ("total error in x direction is = {}".format(xerror))
ccwxerror = xerror
for i in range(0,len(ypos_list)-1):
    yerror += ypos_list[i+1]-ypos_list[i]
    print("errors is y direction = {}" .format(yerror))
print ("total error in y direction is = {}".format(yerror))
ccwyerror=yerror

print("cw x eror is {}  y error is {}".format(cwxerror,cwyerror))
print("ccw x eror is {}  y error is {}".format(ccwxerror,ccwyerror))

xcwcg=cwxerror/5
ycwcg=cwyerror/5

xccwcg=ccwxerror/5
yccwcg=ccwyerror/5

rcw= math.sqrt(xcwcg**2 +ycwcg**2)
rccw= math.sqrt(xccwcg**2 +yccwcg**2)

print(rcw)
print(rccw)