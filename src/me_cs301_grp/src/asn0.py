#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl

import numpy

'''
MotorIDstrings for Hexapod follow the convention legN_jM, where N can be 1,2,3,4,5,6 (for each of the 6 legs) and M can 1,2,3. M=1 is the joint closest to the body and M=3 is the joint farthest from the body.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['front', 'left', 'right'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians

Note that, this list of API functions could potentially grow. You will be notified via Canvas if anything is updated

'''

class HexapodControl(RobotControl):
    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        #for alternating actions. delete when not needed
        count = 0

        #main control loop
        while not rospy.is_shutdown(): 

            #to do the folding sim
            '''
            if (count%2 == 0):
                self.hold_neutral()
            else:
                self.fold_legs()
            count += 1
            '''
            #if the robot sees something in front of front_sensor, it waves hi

            front_sensor_reading = self.front_reading()

            if (front_sensor_reading != -1000):
                print('\nObject detected in front. Waving a leg\n')
                self.wave_leg(5)
            else:
                print('\nNo object detected in front\n')

            #get back to neutral state
            self.hold_neutral()
            time.sleep(2)

            #if the robot sees something in front of left sensor, it
            #gets into a crab pose. the closer the object, the higher
            #the robot raises its arms
            left_sensor_reading = self.left_reading()

            #disable this loop to get lab readings            
            if (left_sensor_reading != -1000):

                print('Object detected on my left side. Setting up for a crab pose:')
                #legs need to be out before they can go up
                self.setMotorTargetJointPosition('leg1_j3',0)
                self.setMotorTargetJointPosition('leg4_j3',0)
                time.sleep(1)

                print('\nCrab posing:')

                #keep posing as long as the object is there
                while (left_sensor_reading != -1000):

                    #crab pose based on the value read from left sensor
                    self.crab_pose(left_sensor_reading)
                    #check the value of left sensor reading again
                    left_sensor_reading = self.left_reading()
        
                print('\nReadjusting to a neutral state:')

                #get back to a neutral state
                self.setMotorTargetJointPosition('leg1_j3',0)
                self.setMotorTargetJointPosition('leg4_j3',0)
            
            else: #if no object detected at left
                print('No object detected at left')
            

            time.sleep(1)
            self.hold_neutral()

            #print some sim readings
            print('\nTime:')
            print(self.getCurrentSimTime())
            print('\nSensor readings - front, left, right:')
            print(self.front_reading())
            print(self.left_reading())
            print(self.right_reading())

            time.sleep(3) # change the sleep time to whatever is the appropriate control rate for simulation

    def hold_neutral(self):
        # --- simple example of a behavior ---- #
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg1_j3', 2.09)

        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j3', 2.09)

        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j3', 2.09)

        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j3', 2.09)

        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j3', 2.09)

        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j3', 2.09)


    #simplifying the API functions for getting a sensor value
    def front_reading(self):
        return self.getSensorValue('front')

    def left_reading(self):
        return self.getSensorValue('left')

    def right_reading(self):
        return self.getSensorValue('right')

    ###

    def fold_legs(self):
	    #a behavior: fold up all the legs at once
        self.setMotorTargetJointPosition('leg1_j3', 3.14)
        self.setMotorTargetJointPosition('leg2_j3', 3.14)
        self.setMotorTargetJointPosition('leg3_j3', 3.14)
        self.setMotorTargetJointPosition('leg4_j3', 3.14)
        self.setMotorTargetJointPosition('leg5_j3', 3.14)
        self.setMotorTargetJointPosition('leg6_j3', 3.14)

    def wave_leg(self, leg_num):
        #a behavior: turn one leg upwards and move it side to side
        #assume the robot is in its resting position before this is called

        #turn leg num into a working string we can use for actions
        string_prefix = 'leg' + str(leg_num)

        #all the names for joints we rotate
        joint1 = string_prefix + '_j1'
        joint2 = string_prefix + '_j2'
        joint3 = string_prefix + '_j3'

        #rotate leg upward at joints 2 and 3
        self.setMotorTargetJointPosition(joint2, -1.2)
        self.setMotorTargetJointPosition(joint3, -0.9)

        #pause to give joints 2 and 3 time to adjust
        time.sleep(1.8)

        #
        #for testing, print out the current position of joints
        # print('Joint 2 posn:', self.getMotorCurrentJointPosition('hexa_leg5_j2'))
        # print('Joint 3 posn:', self.getMotorCurrentJointPosition('hexa_leg5_j3'))
        #

        #set number of iterations of wave
        num_waves = 5

        #each iteration, leg moves left, pauses, then moves right
        for ii in range(num_waves):

            self.setMotorTargetJointPosition(joint1, 0.4)
            time.sleep(0.3)
            self.setMotorTargetJointPosition(joint1, -0.4)
            time.sleep(0.3)

            #for testing only. change time.sleep from 0.3 to 1 for testing
            # print('Joint 1 posn, wave_leg():', self.getMotorCurrentJointPosition('hexa_leg5_j1'))



    def crab_pose(self, sensor_reading):
        #a behavior: read in the current value of the left sensor and raise
        #legs 1 and 4, crab style, according to the proximity

        '''
        The math equations below allow the angles of joints 2 and 3 to vary with
        the sensor reading. Desired angles at the closest and farthest points in the range:

        prox = 0.5m; joint states (j2,j3) = (-0.5,-0.5)
        prox = 0.1m; joint states (j2,j3) = (-1.3, -1.6)
        '''

        j2 = 2 * (sensor_reading - 0.1) - 1.3
        j3 = 2.75 * (sensor_reading - 0.1) - 1.6

        #update the motor locations of opposite legs 1 and 4 to values
        #j2 and j3, as determined by proximity to an object
        self.setMotorTargetJointPosition('leg1_j2',j2)
        self.setMotorTargetJointPosition('leg1_j3',j3)

        self.setMotorTargetJointPosition('leg4_j2',j2)
        self.setMotorTargetJointPosition('leg4_j3',j3)

        #these should be instantaneous, so no time.sleep() if not testing

        #for testing only:
        # print('Sensor distance: ', sensor_reading)
        # print('Joint 2 posn, crab_pose(): ', self.getMotorCurrentJointPosition('hexa_leg1_j2'))
        # print('Joint 3 posn, crab_pose(): ', self.getMotorCurrentJointPosition('hexa_leg1_j3'))
        # print()
        # time.sleep(1)
##

if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()
