#!/usr/bin/env python


import os
import rospy
from enum import Enum
import numpy as np
import time
import threading
import math
from math import asin
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from controllers import PIDController
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
 


class WallFollowerNode:

    def __init__(self):
        rospy.init_node('wall_follower')
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan,self.laser_callback, queue_size=10)
        self.drive_publisher = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        #flag
        self.right = 0
        self.left = 0
        #inizializzo Pid
        self.steering_controller = PIDController(kP=1.2, kD=0.0)
        #creo messaggio
        self.drive_msg = AckermannDriveStamped()
        # self.scan = LaserScan()
        self.goal_distance = 1.2
        self.color=[]
        self.diffr= 0
   

 
    def laser_callback(self,msg):
        #fascio frontale
        cur_dist = min(msg.ranges[int(0.45*len(msg.ranges)):int(0.55*len(msg.ranges))])
        # laterale a -2.47  dx dietro
        side_dist_r1 = min(msg.ranges[int(0*len(msg.ranges)):int(0.01*len(msg.ranges))])
        #   dx  avanti
        side_dist_r2 = min(msg.ranges[int(0.33*len(msg.ranges)):int(0.34*len(msg.ranges))])  
        #  sx avanti
        side_dist_l1 = min(msg.ranges[int(0.65*len(msg.ranges)):int(0.8*len(msg.ranges))]) 
        # sx dietro
        side_dist_l2 = min(msg.ranges[int(0.8*len(msg.ranges)):int(len(msg.ranges))])
        # di fronte
        front_dist = msg.ranges[int(len(msg.ranges)/2)] 
        # angolo 0
        destra = msg.ranges[int(18)]

        #self.color = msg.intensities[int(len(msg.intensities)/2)]

        # color = min(ms[int(0.5*len(msg.ranges)):int(0.5*len(msg.ranges))])
        
        steering_output = self.steering_controller.output(cur_dist, self.goal_distance)

        if side_dist_r1 > 1.4:
         self.right = 0
         self.drive_msg.drive.steering_angle = 0
        if side_dist_l2 > 1.4:
         self.left = 0
         self.drive_msg.drive.steering_angle = 0
        if side_dist_r1 < 1.4:
         self.right = 1
        if side_dist_l2 < 1.4:
         self.left = 1
        
        # trova corridoio
        if destra > side_dist_r2:
         if destra > side_dist_r1:
             if destra > 3.5  and destra < 4:  
                 #gira a dx per 1 secondo                                              
                 self.turn()

        self.diffr = side_dist_r2-side_dist_r1 
        # procedi dritto muro lontano 
        if cur_dist > 1.4 :
         self.drive_msg.drive.speed = steering_output
         self.drive_msg.drive.steering_angle = 0
        #gira a dx
        if self.diffr > 0.1:
         self.drive_msg.drive.steering_angle = -1.2     
     
        
        # muro avanti gira a sx
        if cur_dist < 1.4 :          
         self.drive_msg.drive.steering_angle = +1.2
         self.drive_msg.drive.speed = 0.5      
        

         # muro a sinistra vicino gira a dx
         if side_dist_l2 < 1.4:
             self.drive_msg.drive.steering_angle = -1.2
             self.left = 1                         
       
  

   
        
        ###################  vari print   ################################


        #print("angle: %s" % self.steering_angle.output(self.laser_data))             
        #print("Distance from wall: %s" % front_dist)                
        #print("Colore: %s" % self.color)
        #print("LEFT: %s" % side_dist_l2)
        print("RIGHT1: %s" % side_dist_r1)
        print("destra: %s" % destra) 
        print("RIGHT2: %s" % side_dist_r2)
        print("ZERO: %s" % msg.ranges[int(0)])
        #print("diffr: %s" % self.diffr)
        #print("angle_increment: %s" % msg.angle_increment)        
        #print("min_angle: %s" % msg.angle_min)  



        ##################################################################



 

       
        #pubblica messaggio
        self.drive_publisher.publish(self.drive_msg)

    def turn (self):
        # gira a dx per 1 secondo        
        self.drive_msg.drive.steering_angle = -1.8
        self.drive_msg.drive.speed = 1
        self.drive_publisher.publish(self.drive_msg)
        rospy.sleep(1)
        cur_dist = min(msg.ranges[int(0.45*len(msg.ranges)):int(0.55*len(msg.ranges))])
        if cur_dist < 1:
          rospy.spin()
      
       
 

if __name__ == '__main__':
    
    s = WallFollowerNode()
    rospy.spin()