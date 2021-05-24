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

        #changed this so as to enable the walking gait
        #self.hold_neutral()
        self.walking_neutral()
        time.sleep(2.0)

        #for the derivative control aspect: previous and
        #current readings from left + right sensors
        prev_left = self.left_reading()
        curr_left = self.left_reading()
        prev_right = self.right_reading()
        curr_right = self.right_reading()

        #create a csv file for the purpose of recording data
        #different csv files we need: sensor readings, robot location,
        #and turn data
        location_file = open('location_output.csv', mode='a')
        rotation_file = open('rotation_output.csv', mode='a')
        sensor_file = open('sensor_output.csv', mode='a')


        #main control loop
        while not rospy.is_shutdown(): 
            
            #use this function to make robot walk
            self.robotic_gait(dt = 0.5)

            #update the previous and current L+R sensor readings
            prev_left = curr_left
            curr_left = self.left_reading()

            prev_right = curr_right
            curr_right = self.right_reading()

            curr_front = self.front_reading()

            #print out sensor readings from each side to test out pd control
            print("\nPrev and curr left:")
            print(prev_left)
            print(curr_left)
            print("\nPrev and curr right:")
            print(prev_right)
            print(curr_right)

            #correct any deviations from the course using PD control
            just_corrected = self.course_correct(prev_left, curr_left, prev_right, curr_right)
            #print(just_corrected)

            #get data for csv files
            sim_time = self.getCurrentSimTime()
            robot_location = self.getRobotWorldLocation()

            print("\nSim time:")
            print(sim_time)

            #format data: include a boolean for sensor data as for whether or not
            #a course correction happened at that timestep
            time_and_location = self.format_tl(sim_time, robot_location)
            str_sensors = str(curr_front) + ', ' + str(curr_left)  \
                                    + ', '  + str(curr_right)
            time_and_sensors = str(sim_time) + ', '  \
                    + str_sensors + ', ' + str(just_corrected) + '\n'

            #check if robot is in a corner
            corner = self.in_corner()
            if (corner):

                #store the time/location data before turn
                tl_before_rotation = self.format_tl(sim_time, robot_location, rotation_data = True)

                if (corner == 'left'):
                    print("\nI'm in a left corner! Turning right:")
                    self.turn_90(dt = 0.4, is_right_turn = True)

                elif (corner == 'right'):
                    print("\nI'm in a right corner! Turning left:")
                    self.turn_90(dt = 0.4, is_right_turn = False)

                elif (corner == 'dead end'):
                    print("\nI'm in a dead end! Turning around:")
                    self.turn_180(dt = 0.4, is_right_turn = True)

                #format data for gathering data about rotations
                st_after_rotation = self.getCurrentSimTime()
                rl_after_rotation = self.getRobotWorldLocation()
                
                #each row is a) which corner rotation, b) time and heading before rotation,
                #c) time and heading after rotation. the [:-1] is to cut off the \n from tl_b4_rot8
                tl_after_rotation = self.format_tl(st_after_rotation, rl_after_rotation, rotation_data = True)
                rotation_data = corner + ', ' + tl_before_rotation[:-1] + tl_after_rotation

                #print(rotation_data) 
                rotation_file.write(rotation_data)

            #write to CSV files
            location_file.write(time_and_location)
            sensor_file.write(time_and_sensors)

            #time.sleep(4) # change the sleep time to whatever is the appropriate control rate for simulation

        ###

        #once rospy has shut down, close the files
        print('\nShutting down and closing out of CSV files:')
        location_file.close()
        rotation_file.close()
        sensor_file.close()
        print('Successfully closed CSV files.\n') 



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

    def walking_neutral(self):
        '''Another "hold neutral pose" that allows for the walking movements
        of the robotic gait functions to occur more naturally.
        '''
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg1_j2', -0.2)
        self.setMotorTargetJointPosition('leg1_j3', 1.5)

        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j3', 2.09)

        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j2', -0.2)
        self.setMotorTargetJointPosition('leg3_j3', 2.5)

        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j2', -0.2)
        self.setMotorTargetJointPosition('leg4_j3', 2.5)

        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j3', 2.09)

        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j2', -0.2)
        self.setMotorTargetJointPosition('leg6_j3', 1.5)

        pass

    #simplifying the API functions for getting a sensor value
    def front_reading(self):
        return self.getSensorValue('front')

    def left_reading(self):
        return self.getSensorValue('left')

    def right_reading(self):
        return self.getSensorValue('right')

    #functions needed for this assignment

    def raise_legs(self, legs = 'odd'):
        '''a function for raising the legs up for the moving robotic gait.
        3 legs will increase in height by the same amount, at joints 2 and 3,
        so as to be able to move them forward in next step.

        Args:
         - legs: if 'odd', it means we're moving legs 1,3, and 5 to make the
            gait progress. if 'even', means we're moving legs 2,4,6
        '''

        #string names for legs, for use in the motor functions
        if (legs == 'odd'):
            (legA, legB, legC) = ('leg1', 'leg3', 'leg5')
        elif (legs == 'even'):
            (legA, legB, legC) = ('leg6', 'leg4', 'leg2')
        else:
            raise Exception("Legs must be either 'odd' or 'even.'")

        #determine these constants later
        J2_HEIGHT = -1

        #raise all three legs by desired height of J2
        self.setMotorTargetJointPosition(legA + '_j2', J2_HEIGHT)
        self.setMotorTargetJointPosition(legB + '_j2', J2_HEIGHT)
        self.setMotorTargetJointPosition(legC + '_j2', J2_HEIGHT)
       
    ###

    def swing_legs_for_walking(self, MAG_SWIVEL_ANGLE = 0.2, legs = 'odd'):
        '''a function to swing the legs around for the moving robotic gait.
        3 legs will move in the air at the same time, then drop back onto the ground 

        Args: 
        - 'legs': see raise_legs() for usage
        - rotate_90: if we're rotating about our own axis instead of moving forward,
            neglect the movement of joint J3
        - flip_direction: for use in rotation functions, where it's necessary to
            swing legs in the opposite direction as when walking
        
        '''

        #so as to be able to globally edit this variable later
        # global MAG_SWIVEL_ANGLE
        # MAG_SWIVEL_ANGLE = 0.2

        #string names for legs, for use in the motor functions
        #in order for J1 to rotate move forward, legs 1,3,5 have
        #an opposite ideal J1 angle as legs 6,4,2
        if (legs == 'odd'):
            (legA, legB, legC) = ('leg1', 'leg3', 'leg5')
            J1_SWIVEL_ANGLE = -MAG_SWIVEL_ANGLE

        elif (legs == 'even'):
            (legA, legB, legC) = ('leg6', 'leg4', 'leg2')
            J1_SWIVEL_ANGLE =  MAG_SWIVEL_ANGLE

        #swing all three legs around via J1
        self.setMotorTargetJointPosition(legA + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legB + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legC + '_j1', -J1_SWIVEL_ANGLE)

        #prepare J3 of legs A and B for their next walking movement: 
        LEG_A_J3_INIT = 1.5
        LEG_B_J3_INIT = 2.5
        
        #leg A will be extended at J3: H-\
        #leg C will be folded up at J3: <-H
        self.setMotorTargetJointPosition(legA + '_j3', LEG_A_J3_INIT)
        self.setMotorTargetJointPosition(legB + '_j3', LEG_B_J3_INIT)      

        pass
    
    ###

    def swing_legs_for_turning(self, MAG_SWIVEL_ANGLE = 0.24, legs = 'odd', flip_direction = False):
        '''a function to swing the legs around for the turning functions
        3 legs will move in the air at the same time, then drop back onto the ground 

        Args: 
        - 'legs': see raise_legs() for usage
        - flip_direction: for places where it's necessary to
            swing legs in the opposite direction as the defalt positive direction,
            in which "positive" is the direction legA would move when walking
        
        Differences from swing_legs_for_walking(): no manipulation of J3;
        moreover legC moves in the same direction as legA and legB in this function
        '''

        #so as to be able to globally edit this variable later
        # global MAG_SWIVEL_ANGLE
        # MAG_SWIVEL_ANGLE = 0.2

        #string names for legs, for use in the motor functions
        #in order for J1 to rotate move forward, legs 1,3,5 have
        #an opposite ideal J1 angle as legs 6,4,2
        if (legs == 'odd'):
            (legA, legB, legC) = ('leg1', 'leg3', 'leg5')
            J1_SWIVEL_ANGLE = -MAG_SWIVEL_ANGLE

        elif (legs == 'even'):
            (legA, legB, legC) = ('leg6', 'leg4', 'leg2')
            J1_SWIVEL_ANGLE =  MAG_SWIVEL_ANGLE

        #if, for our turning functions, it's required to flip the turning direction:
        if (flip_direction):
            J1_SWIVEL_ANGLE *= -1

        #swing all three legs around via J1
        self.setMotorTargetJointPosition(legA + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legB + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legC + '_j1',  J1_SWIVEL_ANGLE)

        pass

    ###

    def drop_legs(self, legs = 'odd'):
        '''Simply lowers the legs after the robot has swung its legs around 
        in the air. I've separated this into its own function so that
        I can more easily manipulate the time.sleep() factors in the main
        function of the code.
        '''

        #string names for legs, for use in the motor functions
        if (legs == 'odd'):
            (legA, legB, legC) = ('leg1', 'leg3', 'leg5')
        elif (legs == 'even'):
            (legA, legB, legC) = ('leg6', 'leg4', 'leg2')
        else:
            raise Exception("Legs must be either 'odd' or 'even.'")

        #1. J2, legs A and B need to be at -0.2 so that J3 can transition from 1.5 to 2.5
        #2. J2, leg C can stay at -0.5 b/c J3 doesn't change
        J2_HEIGHT_A = -0.5
        J2_HEIGHT_B = -0.2

        self.setMotorTargetJointPosition(legA + '_j2', J2_HEIGHT_B)
        self.setMotorTargetJointPosition(legB + '_j2', J2_HEIGHT_B)
        self.setMotorTargetJointPosition(legC + '_j2', J2_HEIGHT_A)

    ###

    def ground_legs_for_walking(self, MAG_SWIVEL_ANGLE = 0.2, legs = 'odd'):
        '''Moves the 3 legs that are on the ground at any given time in the
        walking stance. Main priority movement is movement about J1, but
        legs A and C also have movement about J3.

        Args:
        - legs: if 'odd', moves legs 1,3,5. If 'even', moves legs 6,4,2.

        '''

        #string names for legs, for use in the motor functions
        #in order for robot to move forward, legs 1,3,5 have
        #an opposite ideal J1 angle as legs 6,4,2
        if (legs == 'odd'):
            (legA, legB, legC) = ('leg1', 'leg3', 'leg5')
            J1_SWIVEL_ANGLE =  MAG_SWIVEL_ANGLE

        elif (legs == 'even'):
            (legA, legB, legC) = ('leg6', 'leg4', 'leg2')
            J1_SWIVEL_ANGLE = -MAG_SWIVEL_ANGLE

        else:
            raise Exception("Legs must be either 'odd' or 'even.'")

        #swing all three legs around via J1. given legB is on the opposite side of 
        #robot from legs A and C, apply a negative sign to swivel angle
        self.setMotorTargetJointPosition(legA + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legB + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legC + '_j1', -J1_SWIVEL_ANGLE)
        
        #move J3 of legs A and C backward to create some extra forward mvmt 
        LEG_A_J3_END = 2
        LEG_B_J3_END = 2
        
        #leg A will be folded up at J3: H->
        #leg B will be extended at J3: /-H
        self.setMotorTargetJointPosition(legA + '_j3', LEG_A_J3_END)
        self.setMotorTargetJointPosition(legB + '_j3', LEG_B_J3_END)    

        #add a change to the leg B J2 angle so as to keep robot balanced
        LEG_B_J2_ADJUST = -0.65
        self.setMotorTargetJointPosition(legB + '_j2', LEG_B_J2_ADJUST)

        pass

    ###

    def ground_legs_for_turning(self, MAG_SWIVEL_ANGLE = 0.24, legs = 'odd', flip_direction = False):
        '''Moves the 3 legs that are on the ground at any given time in the
        turning functions. Rotation is about joint J1.

        Args:
        - legs: if 'odd', moves legs 1,3,5. If 'even', moves legs 6,4,2.
        - flip_direction: for switching rotation direction. If true, reverses 
            the J1_SWIVEL_ANGLE to allow the joint to rotate in the other direction

        Differences from swing_legs_for_walking(): no manipulation of J3;
        moreover legC moves in the same direction as legA and legB in this function
        '''

        #string names for legs, for use in the motor functions
        #in order for robot to move forward, legs 1,3,5 have
        #an opposite ideal J1 angle as legs 6,4,2
        if (legs == 'odd'):
            (legA, legB, legC) = ('leg1', 'leg3', 'leg5')
            J1_SWIVEL_ANGLE =  MAG_SWIVEL_ANGLE

        elif (legs == 'even'):
            (legA, legB, legC) = ('leg6', 'leg4', 'leg2')
            J1_SWIVEL_ANGLE = -MAG_SWIVEL_ANGLE

        else:
            raise Exception("Legs must be either 'odd' or 'even.'")

        if (flip_direction):
            J1_SWIVEL_ANGLE *= -1

        #swing all three legs around via J1. given legB is on the opposite side of 
        #robot from legs A and C, apply a negative sign to swivel angle
        self.setMotorTargetJointPosition(legA + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legB + '_j1',  J1_SWIVEL_ANGLE)
        self.setMotorTargetJointPosition(legC + '_j1',  J1_SWIVEL_ANGLE)

    ###

    def robotic_gait(self, dt = 0.5):
        '''A combination of simple movements in order to allow the hexapod to walk.

        Args:
        - dt: controls the time in between movements. suggested values are dt = 1 second
            for testing/analysis, and dt = 0.2 or 0.3 for a smooth walking motion
        '''          

        self.raise_legs(legs = 'odd')
        time.sleep(dt)

        self.swing_legs_for_walking(legs = 'odd')
        self.ground_legs_for_walking(legs ='even')
        time.sleep(dt)

        self.drop_legs(legs = 'odd')
        time.sleep(dt)

        self.raise_legs(legs = 'even')
        time.sleep(dt)

        self.swing_legs_for_walking(legs = 'even')
        self.ground_legs_for_walking(legs = 'odd')
        time.sleep(dt)

        self.drop_legs(legs = 'even')
        time.sleep(dt)

    ###

    def turn_motion(self, dt = 0.4, MAG_SWIVEL_ANGLE = 0.24, is_right_turn = True):
        '''just one step in a turn. can apply to either a left turn or right turn

        Args:
        - is_right_turn: use this boolean to make this function execute either
            left or right turns. default for this turn motion is a right turn, 
            so flip_direction has defaults based on right turn
        '''
        
        #use an alias for this to shorten the lines of code
        MSA = MAG_SWIVEL_ANGLE

        #lift the odd legs, and move the even legs on the ground

        self.raise_legs(legs = 'odd')
        time.sleep(dt)

        self.swing_legs_for_turning(legs = 'odd', MAG_SWIVEL_ANGLE = MSA, flip_direction = (not is_right_turn))
        self.ground_legs_for_turning(legs = 'even', MAG_SWIVEL_ANGLE = MSA, flip_direction = (is_right_turn))
        time.sleep(dt)

        self.drop_legs(legs = 'odd')
        time.sleep(dt)

        #lift the even legs, and move the odd legs on the ground

        self.raise_legs(legs = 'even')
        time.sleep(dt)

        self.swing_legs_for_turning(legs = 'even', MAG_SWIVEL_ANGLE = MSA, flip_direction = (is_right_turn))
        self.ground_legs_for_turning(legs = 'odd', MAG_SWIVEL_ANGLE = MSA, flip_direction = (not is_right_turn))
        time.sleep(dt)

        self.drop_legs(legs = 'even')
        time.sleep(dt)

    ###

    def turn_90(self, dt = 0.4, is_right_turn = True):
        '''Function for turning 90 degrees left or right. Uses the turn_motion() function,
        and tells it to do n # of iterations of a turn.

        Args:
        - dt: the timestep between behaviors in the turn motion. Gets passed into turn_motion().
            defaults to 0.2 s
        - is_right_turn: boolean that lets us decide whether to turn right or left. Gets passed into
            turn_motion(). Defaults to True.
        '''

        #given swivel angle is approx. pi/16 and each turn_motion moves 2* swivel_angle, 
        #4 iterations will enable to move 90 degrees
        NUM_ITERATIONS = 4

        for ii in range(NUM_ITERATIONS):
            self.turn_motion(dt = dt, is_right_turn = is_right_turn)

        #get back to a neutral position
        self.walking_neutral()

    ###

    def turn_180(self, dt = 0.4, is_right_turn = True):
        '''Function for turning 180 degrees left or right. Uses the turn_motion() function,
        and tells it to do n # of iterations of a turn.

        Args:
        - dt: the timestep between behaviors in the turn motion. Gets passed into turn_motion().
            defaults to 0.2 s
        - is_right_turn: boolean that lets us decide whether to turn right or left. Gets passed into
            turn_motion(). Defaults to True.
        '''

        #given swivel angle is approx. pi/16 and each turn_motion moves 2* swivel_angle, 
        #4 iterations will enable to move 90 degrees
        NUM_ITERATIONS = 8

        for ii in range(NUM_ITERATIONS):
            self.turn_motion(dt = dt, is_right_turn = is_right_turn)

        #get back to a neutral position
        self.walking_neutral()

    ###

    def in_corner(self):
        '''Checks to see if the robot is in a corner, or in a dead end.
        Cutoff distance for a sensor to be considered "blocked" is a distance
        of 0.4m from the center of the robot.

        Returns the type of corner the robot is stuck in
        '''

        #may want to make the thresholds, as well as dt and MAG_SWIVEL_ANGLE,
        #args of this function as well


        #modify this value later as needed
        FRONT_THRESHOLD = 0.5
        SIDE_THRESHOLD = 0.5

        #readings from sensors
        left = self.left_reading()
        right = self.right_reading()
        front = self.front_reading()

        #all corners include blockage from the front
        if (front != -1000 and front < FRONT_THRESHOLD):

            if (left != -1000 and right != -1000 
                and left < SIDE_THRESHOLD and right < SIDE_THRESHOLD):
                return 'dead end'

            elif (left != -1000 and left < SIDE_THRESHOLD):
                return 'left'

            elif (right != -1000 and right < SIDE_THRESHOLD):
                return 'right'

        #else, return None b/c we're not in a corner
        return None

    ###

    def course_correct(self, prev_left, curr_left, prev_right, curr_right):
        '''In the event that the robot gets off course, this function
        aims to fix the deviation. Uses the turn_motion() function
        to make one small turn back to the wall the robot is following.

        Args:
         - prev_left, curr_left, prev_right, curr_right: the previous and
           current sensor readings, from the left and right sensors. This
           is used to calculate a derivative of the sensor reading to tell
           if deviation is outside a threshold and increasing, or decreasing.

        Returns: a boolean stating whether or not a course correction was 
            implemented in this timestep
        '''

        #let's start out with this defined within the function. this may
        #be a good variable to keep outside this function, then pass to it
        #as an arg, esp. since in_corner() uses this too
        MIN_SIDE_THRESHOLD = 0.35
        MAX_SIDE_THRESHOLD = 0.45

        #this will be the determiner for whether the sensor is within close range
        #of a wall: whether its reading is within range of the sensor
        SENSOR_RANGE = 0.75

        #local variable for MAG_SWIVEL_ANGLE
        MSA = 0.07

        #threshold for deviation. if |curr_reading - prev_reading| >= distance threshold,
        #and robot is outside the max/min side thresholds, we should correct
        DERIVATIVE_THRESHOLD = 0.01

        #readings from sensors
        left = self.left_reading()
        right = self.right_reading()

        #initialize boolean for data analysis
        just_corrected = False

        #test to decide if we're following the left or right wall
        #if we're between a left wall and a right wall, bot follows the left wall
        if (left != -1000 and left < SENSOR_RANGE):

            #if we're outside the max side threshold and distance increasing, turn toward the wall
            if (left > MAX_SIDE_THRESHOLD and (curr_left - prev_left >= DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting toward left wall")
                self.turn_motion(MAG_SWIVEL_ANGLE = MSA, is_right_turn = False)
                just_corrected = True
                
            #else, if we're inside the min threshold and distance decreasing, turn away from wall
            elif (left < MIN_SIDE_THRESHOLD and (curr_left - prev_left <= -DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting away from left wall")
                self.turn_motion(MAG_SWIVEL_ANGLE = MSA, is_right_turn = True)
                just_corrected = True

        elif (right != 1000 and right < SENSOR_RANGE):
            
            #if we're outside the max side threshold and distance increasing, turn toward the wall
            if (right > MAX_SIDE_THRESHOLD and (curr_right - prev_right >= DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting toward right wall")
                self.turn_motion(MAG_SWIVEL_ANGLE = MSA, is_right_turn = True)
                just_corrected = True

            #else, if we're inside the min side threshold and distance decreasing, turn away from the wall
            elif (right < MIN_SIDE_THRESHOLD and (curr_right - prev_right <= -DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting away from right wall")
                self.turn_motion(MAG_SWIVEL_ANGLE = MSA, is_right_turn = False)
                just_corrected = True
   

        #return a statement of whether or not the robot just course-corrected
        return just_corrected
    ###

    def format_tl(self, sim_time, robot_location, rotation_data = False):
        '''Takes in the time and location outputs given by the course API
        and turns them into a string of comma separated values.
        Format of string is time, pos'n x, y, z, orientation x, y, z, w

        Args:
        - sim_time: the current time within the sim, not real life
        - robot_location: a tuple of type geometry_msgs.msg.Point
        - rotation_data: a boolean for whether or not we're looking for the time
            and location of a rotation. if so, discard the "position" data b/c this stays
            mostly constant through a turn
        '''
        
        #each element is of type "point"
        posn = robot_location[0]
        orientation = robot_location[1]

        #determine if we need position data, or just orientation
        if (rotation_data):
            str_location = str(orientation)
        else:
            str_location = str(posn) + str(orientation)

        #all characters to replace in the string
        chars_to_replace = {
            ' ': '',
            '\n': ' ',
            'x:': ', ',
            'y:': ', ',
            'z:': ', ',
            'w:': ', '
        }

        #iterate through each char and translate
        for key, value in chars_to_replace.items():
            str_location = str_location.replace(key,value)

        str_location += ', \n'
        time_and_location = str(sim_time) + str_location
        
        return time_and_location

    ###

##

if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()
