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

#for the mapping commands
from map import *

#for storing the map we create
import pickle

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

class Stack:
    """A last in first out (LIFO) stack representation where elements are pushed and
    popped from the top. Think of a stack of plates, where you can't remove or add a
    plate in the middle, only take from, or add to, the top

    Attributes:
        the_stack - the list that holds the elements of our stack
    """
    ###################################################
    ###CREDIT TO THIS CODE GOES TO SARA SOOD FROM CS150
    ###I DID NOT WRITE AND DO NOT OWN ANY OF THIS CODE
    ###################################################

    def __init__(self, initial = []):
        """Constructor for a stack, set the stack up with the given list if any is
        provided otherwise empty

        Args:
            initial - optional list of elements to fill the stack with
        """

        # can't have lists (mutable objects in general) as default values as the default
        # is shared among all instances. need to copy here to avoid issues with aliases
        self.the_stack = initial[:]

    def __str__(self):
        """String representation of the stack"""
        return "The stack contains: " + self.the_stack

    def is_empty(self):
        """Check if stack has no elements

        Returns:
            True if stack has no elements, False otherwise
        """
        return len(self.the_stack) == 0

    def push(self, elt):
        """Add element (elt) to top of stack

        Args:
            elt - an item to add to the stack
        """
        self.the_stack.append(elt)

    def pop(self):
        """Remove and return the top item in the stack (corresponds to the last item in
        the list)

        Returns:
            the most recently added element
        """
        return self.the_stack.pop()

class HexapodControl(RobotControl):
    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")

        ################ MEMBER DATA ##############

        #mechanics of turning 
        self.gait_timestep = 0.4
        self.turn_timestep = 0.4
        self.swivel_angle_walking = 0.2
        self.swivel_angle_90 = 0.23 #bumped this up while sim is slow. was 0.23
        self.swivel_angle_180 = 0.24 #was 0.24

        #course correct data from sensors
        self.prev_left = -1000
        self.curr_left = -1000
        self.prev_right = -1000
        self.curr_right = -1000
        self.curr_front = -1000
        self.just_corrected = False

        #maze data: heading, position, world location
        self.heading = DIRECTION.South
        self.maze_position = [0,0]
        self.prev_x = 0
        self.prev_y = 0
        self.curr_x = 0
        self.curr_y = 0

        #for knowing when the robot should ignore
        #a side. see self.decide_follow()
        self.follow_left = True
        self.follow_right = True

        #for making a map of an unknown course:
        #make lists that keep track of recent movements and all squares visited
        self.posns_visited = []
        self.recent_posns = []

        #so as to enable the walking gait
        self.walking_neutral()
        time.sleep(2.0)


        ###

        ##PART A: NAVIGATING AN EXISTING MAZE
        #initialize empty map
        # map = CSMEMap()

        #map.clearCostMap()
        #self.navigate_existing_maze(map)

        ###

        #PART B: Making a map from the world

        #initialize empty map
        map = CSMEMap()
        map.clearObstacleMap()

        #bring in the map we made in our earlier test
        # map = self.load_map()

        #make the robot's initial position editable
        start_posn = self.get_user_input_start()
        self.maze_position = list(start_posn)

        map = self.gen_obstacle_map(map)
        map.printObstacleMap()
        self.save_map(map)
        print("\nSuccessfully closed out of all files.")


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

    #simplifying the API functions for getting a sensor value
    def front_reading(self):
        return self.getSensorValue('front')

    def left_reading(self):
        return self.getSensorValue('left')

    def right_reading(self):
        return self.getSensorValue('right')

    #functions needed for robotic gait

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
       
    def swing_legs_for_walking(self, legs = 'odd'):
        '''a function to swing the legs around for the moving robotic gait.
        3 legs will move in the air at the same time, then drop back onto the ground 

        Args: 
        - 'legs': see raise_legs() for usage
        - rotate_90: if we're rotating about our own axis instead of moving forward,
            neglect the movement of joint J3
        - flip_direction: for use in rotation functions, where it's necessary to
            swing legs in the opposite direction as when walking
        
        '''

        MAG_SWIVEL_ANGLE = self.swivel_angle_walking

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
    
    def swing_legs_for_turning(self, legs, MAG_SWIVEL_ANGLE, flip_direction = False):
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

    def ground_legs_for_walking(self, legs = 'odd'):
        '''Moves the 3 legs that are on the ground at any given time in the
        walking stance. Main priority movement is movement about J1, but
        legs A and C also have movement about J3.

        Args:
        - legs: if 'odd', moves legs 1,3,5. If 'even', moves legs 6,4,2.

        '''

        MAG_SWIVEL_ANGLE = self.swivel_angle_walking

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

    def ground_legs_for_turning(self, legs, MAG_SWIVEL_ANGLE, flip_direction = False):
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

    def robotic_gait(self):
        '''A combination of simple movements in order to allow the hexapod to walk.

        Args:
        - dt: controls the time in between movements. suggested values are dt = 1 second
            for testing/analysis, and dt = 0.2 or 0.3 for a smooth walking motion
        '''          

        #add this so as if we press CTRL-C, we can exit out easily
        if (rospy.is_shutdown()):
            return

        # #for sensor testing
        # print("\nCurr and prev left:")
        # print(self.curr_left)
        # print(self.prev_left)
        # print("\nCurr and prev right:")
        # print(self.curr_right)
        # print(self.prev_right)

        dt = self.gait_timestep

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

    def turn_motion(self, angle, is_right_turn = True):
        '''just one step in a turn. can apply to either a left turn or right turn

        Args:
        - is_right_turn: use this boolean to make this function execute either
            left or right turns. default for this turn motion is a right turn, 
            so flip_direction has defaults based on right turn
        '''
        
        #add this so as if we press CTRL-C, we can exit out easily
        if (rospy.is_shutdown()):
            return

        #use the dt from member data
        dt = self.turn_timestep

        #lift the odd legs, and move the even legs on the ground

        self.raise_legs(legs = 'odd')
        time.sleep(dt)

        self.swing_legs_for_turning('odd', angle, flip_direction = (not is_right_turn))
        self.ground_legs_for_turning('even', angle, flip_direction = (is_right_turn))
        time.sleep(dt)

        self.drop_legs(legs = 'odd')
        time.sleep(dt)

        #lift the even legs, and move the odd legs on the ground

        self.raise_legs(legs = 'even')
        time.sleep(dt)

        self.swing_legs_for_turning('even', angle, flip_direction = (is_right_turn))
        self.ground_legs_for_turning('odd', angle, flip_direction = (not is_right_turn))
        time.sleep(dt)

        self.drop_legs(legs = 'even')
        time.sleep(dt)

        #if we pres

    def turn_90(self, is_right_turn = True):
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
            self.turn_motion(self.swivel_angle_90, is_right_turn)

        #get back to a neutral position
        self.walking_neutral()

    def turn_180(self, is_right_turn = True):
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
            self.turn_motion(self.swivel_angle_180, is_right_turn)

        #get back to a neutral position
        self.walking_neutral()

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

    def course_correct(self):
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

        #get updated versions of members of the hexapod data class
        self.prev_left = self.curr_left
        self.prev_right = self.curr_right
        self.curr_left = self.left_reading()
        self.curr_right = self.right_reading()

        #turn these into local versions
        prev_left = self.prev_left
        prev_right = self.prev_right
        curr_left = self.curr_left
        curr_right = self.curr_right

        #range that robot has to be inside
        MIN_SIDE_THRESHOLD = 0.42 #previously: 0.35 and 0.45
        MAX_SIDE_THRESHOLD = 0.50

        #this will be the determiner for whether the sensor is within close range
        #of a wall: whether its reading is within range of the sensor
        SENSOR_RANGE = 0.75

        #threshold for deviation. if |curr_reading - prev_reading| >= distance threshold,
        #and robot is outside the max/min side thresholds, we should correct
        DERIVATIVE_THRESHOLD = 0.01
        deriv_left = curr_left - prev_left
        deriv_right = curr_right - prev_right

        #local variable for MAG_SWIVEL_ANGLE
        #MSA = 0.09

        #changed this so that angle of turn is dependent on value of derivative
        MSA_left = 0.8411 * abs(deriv_left) + 0.08159
        MSA_right = 0.8411 * abs(deriv_right) + 0.08159

        #initialize boolean for data analysis
        just_corrected = False

        #test to decide if we're following the left or right wall
        #if we're between a left wall and a right wall, bot follows the left wall
        if (curr_left != -1000 and prev_left != -1000 \
                    and curr_left < SENSOR_RANGE and self.follow_left):

            #if we're outside the max side threshold and distance increasing, turn toward the wall
            if (curr_left > MAX_SIDE_THRESHOLD and (deriv_left >= DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting toward left wall")
                self.turn_motion(angle = MSA_left, is_right_turn = False)
                just_corrected = True
                
            #else, if we're inside the min threshold and distance decreasing, turn away from wall
            elif (curr_left < MIN_SIDE_THRESHOLD and (deriv_left <= -DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting away from left wall")
                self.turn_motion(angle = MSA_left, is_right_turn = True)
                just_corrected = True


        elif (curr_right != -1000 and prev_right != -1000  \
                   and curr_right < SENSOR_RANGE and self.follow_right):
            
            #if we're outside the max side threshold and distance increasing, turn toward the wall
            if (curr_right > MAX_SIDE_THRESHOLD and (deriv_right >= DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting toward right wall")
                self.turn_motion(angle = MSA_right, is_right_turn = True)
                just_corrected = True

            #else, if we're inside the min side threshold and distance decreasing, turn away from the wall
            elif (curr_right < MIN_SIDE_THRESHOLD and (deriv_right <= -DERIVATIVE_THRESHOLD) ):
                print("\nCourse correcting away from right wall")
                self.turn_motion(angle = MSA_right, is_right_turn = False)
                just_corrected = True

        #return a statement of whether or not the robot just course-corrected
        return just_corrected

    #functions for formatting data

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

    def get_data(self, has_corrected):
        '''A handy way to package the data so it can be stored in CSV files.
        Important details:
            - course_correct() must happen before this function so as to store
                data on whether or not we course corrected due to sensor input
            - CSV files must exist before this function is called. Handled
                in the asn2 code by putting file open/close before and after the main
                loop
        '''

        location_file = open('location_output.csv', mode='a')
        sensor_file = open('sensor_output.csv', mode='a')

        #get data for csv files
        sim_time = self.getCurrentSimTime()
        robot_location = self.getRobotWorldLocation()

        #print sensor data at current timestep
        # print("\n\nSim time:")
        # print(sim_time)
        # print("\nPrev and curr left:")
        # print(self.prev_left)
        # print(self.curr_left)
        # print("\nPrev and curr right:")
        # print(self.prev_right)
        # print(self.curr_right)

        #format data: include a boolean for sensor data as for whether or not
        #a course correction happened at that timestep
        time_and_location = self.format_tl(sim_time, robot_location)
        str_sensors = str(self.curr_front) + ', ' + str(self.curr_left)  \
                                + ', '  + str(self.curr_right)
        time_and_sensors = str(sim_time) + ', '  \
                + str_sensors + ', ' + str(has_corrected) + '\n'

        #add data to our CSV files. these were made global so as not
        #to have to add them to the class or pass them as arguments
        location_file.write(time_and_location)
        sensor_file.write(time_and_sensors)

        location_file.close()
        sensor_file.close()

    def get_user_input_start(self):
        '''Asks the user for the start posn, goal posn, and current heading
        of the robot. Performs error checking to see if the user input is an
        invalid format for either the position or heading.

        Returns:
         - two tuples: start_posn and end_posn
         - assigns self.heading but doesn't return it as a variable
        '''

        #get info on start posn
        while True:
            try:
                start_posn = input("\nEnter a start posn in the format i, j: ")
                i = start_posn[0]
                j = start_posn[1]

                if (i < 0 or i > 7 or j < 0 or j > 7):
                    print("i and j must be between 0 and 7.")
                    continue
                else:
                    break
            except:
                #if anything is weird about the data, just try again
                print("Please make sure to give two inputs separated by a comma.")
                continue
        
        #get input for heading        
        while True:                      
            try:
                heading_input = input("\nEnter a number for the current heading, \
                                        \nwhere N=1, E=2, S=3, W=4: ")
                if (heading_input == 1):
                    self.heading = DIRECTION.North
                    break
                elif (heading_input == 2):
                    self.heading = DIRECTION.East
                    break
                elif (heading_input == 3):
                    self.heading = DIRECTION.South
                    break
                elif (heading_input == 4):
                    self.heading = DIRECTION.West
                    break
                else:
                    print("Heading must be an integer between 1 and 4.")
                    continue
            
            except:
                print("Please enter an integer. Values must be between 1 and 4.")
                continue

        #return starting data
        return start_posn

    def get_user_input_end(self):
        #get info for goal posn
        while True:
            try:
                goal_posn = input("\nEnter a goal posn in the format i, j: ")
                i = goal_posn[0]
                j = goal_posn[1]

                if (i < 0 or i > 7 or j < 0 or j > 7):
                    print("i and j must be between 0 and 7.")
                    continue
                else:
                    break
            except:
                #if anything weird, just try again
                print("Please make sure to give two inputs separated by a comma.")
                continue

        #get input for the final heading we want robot to be at      
        while True:                      
            try:
                heading_input = input("\nEnter a number for the final heading, \
                                        \nwhere N=1, E=2, S=3, W=4: ")
                if (heading_input == 1):
                    final_heading = DIRECTION.North
                    break
                elif (heading_input == 2):
                    final_heading = DIRECTION.East
                    break
                elif (heading_input == 3):
                    final_heading = DIRECTION.South
                    break
                elif (heading_input == 4):
                    final_heading = DIRECTION.West
                    break
                else:
                    print("Heading must be an integer between 1 and 4.")
                    continue
            
            except:
                print("Please enter an integer. Values must be between 1 and 4.")
                continue

        #return end data
        return goal_posn, final_heading
        
    def print_better_cost_map(self, map):
        # *****************************
        # Function Name : printBetterCostMap
        # Description   : Print a cost map using syntax and walls from printObstacleMap()
        # Input         : none
        # Output        : none
        # Return        : none; prints to standard output
        # *****************************

        print("\nCost map with obstacles: ")
        for i in range(8):
            for j in range(8):
                if (map.horizontalWalls[i][j] == 0):
                    if i == 0:
                        sys.stdout.write(" ---")
                    else:
                        sys.stdout.write("    ")
                else:
                    sys.stdout.write(" ---")

            print(" ")
            for j in range(8):
                
                #so as to make each int take up 2 chars in string
                cost = "{0:2d}".format(map.costMap[i][j])
                if (map.verticalWalls[i][j] == 0):
                    if j == 7:
                        sys.stdout.write(" " + cost + " |")
                    elif j == 0:
                        sys.stdout.write("|" + cost + " ")
                    else:
                        sys.stdout.write(" " + cost + " ")
                else:
                    if j == 7:
                        sys.stdout.write("|" + cost + " |")
                    else:
                        sys.stdout.write("|" + cost + " ")
            print(" ")
        for j in range(8):
                sys.stdout.write(" ---")
        print(" ")

    def save_map(self, map):
        '''Uses pickling to save a map as a .pkl file. Uses 'wb' flag to write
        in binary and HIGHEST_PROTOCOL, which is used specifically for saving
        classes/objects. Let fp = filepath.
        '''
        with open("map.pkl","wb") as fp:
            pickle.dump(map, fp, pickle.HIGHEST_PROTOCOL)

    def load_map(self):
        '''Opens up the same map we just pickled. Returns that map.'''

        with open("map.pkl", "rb") as fp:
            map = pickle.load(fp)
            return map

    #functions for navigating maze

    def move_one_space(self, dir_travel, map):
        '''Used to move one space on the map, corresponding to 1 meter in CoppeliaSim.
        Robotic gait is repeated a certain number of iterations to approximately reach
        the center of the next square.

        If the current heading (self.heading) is not the same as the direction in which 
        we need to travel, the robot will rotate by however many degrees is necessary 
        to get to dir_travel

        Args: 
         - dt: the timestep between movements in the robotic gait function. the smaller
            the dt, the faster the robot moves
         - dir_travel: the direction you want the robot to move. Defaults to South.

        '''

        #if the current heading of the robot is not the same as the direction,
        #make a turn
        if (self.heading != dir_travel):
            self.adjust_heading(dir_travel)

        #get current heading and position, and use these to find the neighboring
        #cell in that direction. to be used in self.decide_follow()
        curr_posn = self.maze_position
        neighbor_posn = self.get_neighbor_coords(map,curr_posn,dir_travel)        

        #try to counteract the effects of edges of walls
        self.decide_follow(map,curr_posn)

        #for testing: see which walls the robot is following
        print("\nAm I currently following: left wall, right wall")
        print(self.follow_left)
        print(self.follow_right)

        #the current approximation for how many steps equals
        #approximately one step            
        NUM_STEPS = 13

        #make a stepping motion and check to see if we need to course correct
        for ii in range(NUM_STEPS):

            #for easy exit
            if (rospy.is_shutdown()):
                return

            self.robotic_gait()
            has_corrected = self.course_correct()
            self.get_data(has_corrected)

            #at the middle of square, decide whether to follow walls based
            #on walls in next square
            if (ii == NUM_STEPS//2 - 1):
                self.decide_follow(map, neighbor_posn)

                #for testing: see which walls the robot is following
                print("\nAm I currently following: left wall, right wall")
                print(self.follow_left)
                print(self.follow_right)

        #update the position of the robot based on which direction we turned
        #position is a tuple (i,j) where i = south/north and j is east/west
        if (dir_travel == DIRECTION.South):
            self.maze_position[0] += 1
        elif (dir_travel == DIRECTION.North):
            self.maze_position[0] -= 1
        elif (dir_travel == DIRECTION.East):
            self.maze_position[1] += 1
        elif (dir_travel == DIRECTION.West):
            self.maze_position[1] -= 1

        print("\nCurrent position:")
        print(self.maze_position)

    def adjust_heading(self, dir_travel):
        '''Takes in the direction we need to be travelling in, compares
        it to the member "heading" of the hexapod, and adjusts to meet that
        direction.

        Args:
         - dir_travel: the direction we want to be moving in. Type DIRECTION.<x>,
            where <x> is North, South, East or West
        '''
        
        #new way to generalize this code
        if ( (self.heading - dir_travel)%4 == 1):
            print("\nTurning left:")
            self.turn_90(is_right_turn = False)

        elif ( (self.heading - dir_travel)%4 == 3):
            print("\nTurning right:")
            self.turn_90(is_right_turn = True)

        elif ( (self.heading - dir_travel)%4 == 2):
            print("\nTurning around:")
            self.turn_180()

        elif ( (self.heading - dir_travel)%4 == 0):
            print("\nDid not adjust heading. Code shouldn't get here though.")

        else:
            raise Exception("adjust_heading: invalid direction of travel")

        #check this manually, but by this point the heading should now be 
        #equal to dir_travel after the change
        self.heading = dir_travel

    def move_in_line(self, num_spaces, dir_travel, map):
        '''Carries out the move_one_space() function several times, so as to move
        several spaces in a line

        Args:
         - num_spaces: the number of spaces the robot will move in a line
         - dir_travel: the direction in which the robot will move in a line
        '''

        #for each number of spaces we have to move, execute move_one_space()
        for ii in range(num_spaces):

            #for easy exit
            if (rospy.is_shutdown()):
                return

            self.move_one_space(dir_travel, map)

    def decide_follow(self, map, curr_posn):
        '''It's not advantageous for the robot to be following left and right walls
        all the time. For one thing, there are the thin ends of walls that the robot
        might pass by and accidentally pivot around to try to course_correct() to. 
        For another thing, in the creation of a new map, the robot should not try to
        follow any walls if it doesn't have any data about walls that exist around it

        Given a position on the map and the current heading of the robot, the code 
        checks to see if the left and right walls from the robot at that position are 
        blocked, and if not, tells the robot not to follow these walls.
        '''

        #thin walls around a robot depend on the position and direction
        #robot is travelling
        (m,n) = curr_posn
        heading = self.heading

        #members to say if the robot should avoid following wall on either side
        self.follow_right = True
        self.follow_left = True

        #check to see if directions around the wall are blocked. if so, then the
        #wall is part of a structure we can follow
        #(m,n) = self.get_neighbor_coords(map, curr_posn, heading)
        west_blocked = map.getNeighborObstacle(m,n,DIRECTION.West)
        east_blocked = map.getNeighborObstacle(m,n,DIRECTION.East)
        north_blocked = map.getNeighborObstacle(m,n,DIRECTION.North)
        south_blocked = map.getNeighborObstacle(m,n,DIRECTION.South)

        #if the left and right walls relative to the robot's current position/heading
        #don't exist or don't exist yet, don't follow them
        if (heading == DIRECTION.South):

            #if there are thin walls south and (east or west) from posn:
            if (not west_blocked):
                self.follow_right = False
            if (not east_blocked):
                self.follow_left = False

        elif (heading == DIRECTION.North):

            #if there are thin walls north and (east or west) from posn:
            if (not east_blocked):
                self.follow_right = False
            if (not west_blocked):
                self.follow_left = False

        elif (heading == DIRECTION.West):

            #if there are thin walls west and (north or south) from posn:
            if (not north_blocked):
                self.follow_right = False
            if (not south_blocked):
                self.follow_left = False

        elif (heading == DIRECTION.East):

            #if there are thin walls east and (north or south) from posn:
            if (not south_blocked):
                self.follow_right = False
            if (not north_blocked):
                self.follow_left = False
        
    #functions for planning path

    def get_neighbor_coords(self, map, curr_posn, dir):
        '''Just a helper function to put in a current cell and a direction, and
        get out the coords (i,j) of the closest cell in that direction.
        '''

        #out-of-bounds checking is done at the next step
        (i,j) = curr_posn

        if dir == DIRECTION.North:
            neighbor_coords = (i-1, j)
        elif dir == DIRECTION.South:
            neighbor_coords = (i+1, j)
        elif dir == DIRECTION.West:
            neighbor_coords = (i, j-1)
        elif dir == DIRECTION.East:
            neighbor_coords = (i, j+1)
        else:
            print('dir equals:')
            print(dir)

        return neighbor_coords

    def min_cost_and_coords(self, map, curr_posn, goal_posn):
        '''Calculates the min cost of any cell surrounding the current position,
        then returns both the cost and coords of that position. Used in several
        functions, calculate_cell_cost() and gen_path_coords().
        '''
        
        #just to make sure we're comparing tuples, not lists
        #from testing in HW2 pt2
        curr_posn = tuple(curr_posn)
        goal_posn = tuple(goal_posn)

        (i,j) = curr_posn
        all_dirs_list = [DIRECTION.East, DIRECTION.South, DIRECTION.West, DIRECTION.North]
        dirs_to_check = []

        #if an obstacle exists in that direction, skip it
        #else, it's a free direction, so see if we should add to stack

        for dir in all_dirs_list:
            #check all 4 positions around the current cell
            is_blocked = map.getNeighborObstacle(i,j,dir)
            
            #if any of our getNeighborObstacles() failed, exit
            if (is_blocked == -1):
                raise Exception("calculate_cell_cost: getNeighborObstacle() failed")

            #1 corresponds to blocked, 0 is clear
            elif (is_blocked):
                continue

            else:
                dirs_to_check.append(dir)

        #now that we have a list of unblocked directions, see what the cost is
        #in those directions. goal is to be able to assign a cost to the current cell
        min_cost_yet = 1000
        min_cost_coords = curr_posn

        for dir in dirs_to_check:

            #print("Min cost yet: ", min_cost_yet)
            neighbor_cost = map.getNeighborCost(i,j,dir)
            neighbor_coords = tuple(self.get_neighbor_coords(map,curr_posn,dir))

            # print("\nMCAC: neighbor_cost and neighbor_coords")
            # print(neighbor_cost)
            # print(neighbor_coords)

            #couple of different cases: cost is 0 b/c it's the goal posn 
            #cost is 0 b/c uninitialized, cost is 1000 b/c outer wall,
            #or cost is a nonzero int

            if (goal_posn == neighbor_coords):
                min_cost_yet = 0
                min_cost_coords = neighbor_coords
                break
            
            elif (neighbor_cost == 0):
                continue #need to skip the < check

            elif (neighbor_cost < min_cost_yet):
                min_cost_yet = neighbor_cost
                min_cost_coords = neighbor_coords

        return min_cost_yet, min_cost_coords

    def calculate_cell_cost(self, map, curr_posn, goal_posn):
        '''Takes in a map object, which holds data about obstacles in space and
        the currently calculated costs of squares, and calculates the cost of
        one cell based on neighboring cells.

        Args:
         - map: a map of type CSMEMap(). Has an obstacle map, at self.horizontalWalls
            and self.verticalWalls, and a cost map at self.costMap
         - curr_posn: a tuple (i,j) of the position of the cell on the map
         - goal_posn: a tuple (i,j) of the position we're trying to reach

        Returns: nothing; alters costMap to include the cost calculated within function
        '''

        #for debug: trying to ensure only tuples are getting compared
        curr_posn = tuple(curr_posn)
        goal_posn = tuple(goal_posn)

        (i,j) = curr_posn
        #case for the first cell: it's gonna stay at a cost of 0, so just skip
        #all the computation
        if (curr_posn == goal_posn):
            return

        min_cost_yet, _ = self.min_cost_and_coords(map,curr_posn, goal_posn)

        #now assign the cost to the current cell. Ideally at least one cell around
        #the current cell is either the goal position or nonzero
        cell_cost = min_cost_yet + 1
        result = map.setCost(i,j,cell_cost)

        #error check the cost setting step
        if (result == -1):
            raise Exception("calc cell cost: setCost failed")

    def add_neighbors_to_stack(self, map, curr_posn, goal_posn, posn_stack):
        '''Now that we've found the cost of the current cell, compare this cell
        to its neighbors. Options for neighboring cells are that they're blocked,
        an outer wall of the scene, cost==0 because they're uninitialized, cost==0
        because it's the goal position, or cost is nonzero.

        Args:
         - map: a map of type CSMEMap(). Contains info about which cells are blocked
            around a cell and what the cost of cells are.
         - curr_posn: a tuple (i,j) of the current position of the cell we're studying
         - goal_posn: a tuple (i,j) of the goal position of the map
         - posn_stack: a stack of type stack_and_queue.Stack that contains a set of 
            positions to check next. Positions are taken out in a LIFO algorithm. We
            want to take the current position and see where to go next, hence add to stack.
        '''

        #for easy exit. need to return the posn_stack because None.is_empty()
        #doesn't work
        if (rospy.is_shutdown()):
            return

        (i, j) = curr_posn
        curr_cost = map.getCost(i,j)

        #get the cost of all neighboring cells
        all_dirs_list = [DIRECTION.East, DIRECTION.South, DIRECTION.West, DIRECTION.North]
        dirs_to_check = []

        #find out which cells are blocked or not. we don't want to try to 
        #build a path through a wall

        for dir in all_dirs_list:
            #check all 4 positions around the current cell
            is_blocked = map.getNeighborObstacle(i,j,dir)
            
            #if any of our getNeighborObstacles() failed, exit
            if (is_blocked == -1):
                raise Exception("add_neighbors: getNeighborObstacle() failed")

            #1 corresponds to blocked, 0 is clear
            elif (is_blocked):
                continue

            else:
                dirs_to_check.append(dir)

        ##within the subset of N/S/E/W that isn't blocked, compare the cost
        #of the current cell to that of the neighboring cells

        for dir in dirs_to_check:

            neighbor_coords = self.get_neighbor_coords(map, curr_posn, dir)
            neighbor_cost = map.getNeighborCost(i,j,dir)

            #if a cell is the goal posn, skip--don't add it as a position to start from
            if (goal_posn == neighbor_coords):
                continue

            #if cell value is 0 and it's not the goal posn, it's a direction in which to travel
            #add it to the stack of directions to check next
            elif (neighbor_cost == 0 ):
                if (neighbor_coords not in posn_stack.the_stack):
                    posn_stack.push(neighbor_coords)

            #if cell value is 1000, means it's a wall so skip
            elif (neighbor_cost == 1000):
                continue

            #if the next cell has a cost thatis > than (curr_cost + 1), we need to 
            #overwrite it because we've found a more efficient cost route 
            elif (neighbor_cost > curr_cost + 1):
                if (neighbor_coords not in posn_stack.the_stack):
                    posn_stack.push(neighbor_coords)

    def gen_cost_map(self, map, goal_posn):
        '''Takes in an obstacle map and a goal position of where you want to
        navigate to, and generates a cost map based on distances from that 
        goal position.

        Args:
         - map: an obstacle map that shows walls around each square.
            Argument is of type CSMEMap().
         - goal_posn: a tuple containing (i, j) coordinates of the square
           we want to generate cost based on.
        '''

        #for easy exit
        if (rospy.is_shutdown()):
            return

        #initialize a stack so that we can use a last in, first out (LIFO)
        #algorithm to check paths branching away from the goal posn
        posn_stack = Stack()

        #add the first posn to the list: our goal posn
        posn_stack.push(goal_posn)

        #as long as there are still valid positions to check, keep checking for
        #new directions to travel to
        while (not posn_stack.is_empty()):

            #for easy exit
            if (rospy.is_shutdown()):
                return

            #pop an element from the stack, then calculate what its current cost is
            curr_posn = posn_stack.pop()
            self.calculate_cell_cost(map, curr_posn, goal_posn)

            #look around the current cell and see what the costs of neighboring cells are
            self.add_neighbors_to_stack(map, curr_posn, goal_posn, posn_stack)

    def gen_path_coords(self, map, start_posn, goal_posn):
        '''Takes a cost map and finds the most efficient route to get from 
        a start position to an end position.

        Args:
         - map: an instance of type CSMEMap(). Contains info about the obstacles
            and cost map of a world
         - start_posn: a tuple (i,j) of where you want the traveller to start
         - end_posn: a tuple (i,j) of where you want the traveller to end 

         Returns: a list of coordinates [(a,b), (c,d), ... ] that will need
            to be travelled to, in sequence, from the start posn 
        '''

        #find cost of current cell
        curr_posn = tuple(start_posn)
        (i,j) = curr_posn
        curr_cost = map.getCost(i,j)

        coord_list = [start_posn]

        #keep going until you reach a cost of 0
        while (curr_cost != 0 and curr_cost < 1000):

            if (rospy.is_shutdown()):
                return coord_list            

            _, min_cost_coords = self.min_cost_and_coords(map,curr_posn, goal_posn)

            #min_cost_coords gives us the direction of the lowest cost,
            coord_list.append(min_cost_coords)

            #move on to next cell
            curr_posn = min_cost_coords
            (i,j) = curr_posn
            curr_cost = map.getCost(i,j)

        #return list of coords to travel to get to goal posn
        return coord_list

    def coords_to_dirs(self, coord_list):
        '''Takes in a list of coordinates in tuple form (i,j), and 
        returns the directions that need to be travelled in order to
        travel from start to end.
        '''
        #for easy exit
        if (rospy.is_shutdown()):
            return []

        #iterate through list up until the last two tuples to compare
        dir_list = []
        for ii in range(len(coord_list) - 1):

            curr = coord_list[ii]
            next = coord_list[ii+1]

            #compare the difference between elements of curr and next.
            #determine whether we travel east/west or n/s
            if (next[0] > curr[0]):
                dir = DIRECTION.South
            elif (next[0] < curr[0]):
                dir = DIRECTION.North
            elif (next[1] > curr[1]):
                dir = DIRECTION.East
            elif (next[1] < curr[1]):
                dir = DIRECTION.West
            else:
                continue #to skip adding a dir that doesn't exist

            dir_list.append(dir)

        #now that dir_list has directions in which to travel, consolidate
        #multiple motions in one direction into a single line of travel
        jj = 0
        consolidated_dirs = []

        while (jj < len(dir_list)):

            num_steps = 1
            curr_dir = dir_list[jj]

            #look forward in the list to see if next movement is in same dir
            while ( (jj + 1 < len(dir_list)) and (dir_list[jj+1] == curr_dir ) ):
                num_steps += 1
                jj += 1

            #now we have the number of steps in each direction
            consolidated_dirs.append((num_steps,curr_dir))
            jj += 1

        # return list of directions. this will be hard to read, but interpret 
        # 2 as east, 3 as south, 4 as west, and 1 as north
        return consolidated_dirs

    def navigate_existing_maze(self, map):
        '''The set of complex instructions needed to tell the robot to follow an existing
        maze, with obstacle information already known.
        '''
        #get user input as to where robot starts and ends.
        #if any bad input, return to ask questions 

        start_posn = self.get_user_input_start()
        goal_posn, final_heading = self.get_user_input_end()
        self.maze_position = list(start_posn)

        #so as to enable the walking gait
        self.walking_neutral()
        time.sleep(2.0)

        #generate a cost map
        self.gen_cost_map(map,goal_posn)
        self.print_better_cost_map(map)

        #find coords for robot to travel to get to end
        coord_list = self.gen_path_coords(map, start_posn, goal_posn)
        dir_list = self.coords_to_dirs(coord_list)
        
        print("\nList of coords:")
        print(coord_list)
        print("\nList of (num_spaces, dir_num) to travel:")
        print(dir_list)

        #for each directional movement that we have to make, execute a 
        #move_in_line() function
        for num_dir in dir_list:

            #for easy exit
            if (rospy.is_shutdown()):
                return

            num_spaces = num_dir[0]
            dir = num_dir[1]
            self.move_in_line(num_spaces, dir, map)
        
        #if our current heading does not matched the desired final heading, adjust
        if (self.heading != final_heading):
            self.adjust_heading(final_heading)

        ###

    #functions for making a map

    def assign_wall(self, map, position, dir):
        '''Takes in an obstacle map, a position, and a direction, and puts a wall at
        that position relative to the robot.

        Args:
         - map: an instance of CSMEMap(), which has attributes of a cost map and an
            obstacle map
         - position: a tuple (i,j) of the position where the robot is when it tries
            to assign a wall
         - dir: a data type DIRECTION, which has four different aspects: N = 1, 
            E = 2, S = 3, W = 4

        Returns: none, just modifies the obstacle map of "map"
        '''

        (i, j) = position

        #walls defined in map.py such that the north wall of a cell is at hWalls[i][j],
        #south wall hWalls[i+1][j], west vWalls[i][j], east vWalls[i][j+1]

        #if a wall detected in each direction, modify the map to include this wall
        if (dir == DIRECTION.North):
            map.horizontalWalls[i][j] = 1
        elif (dir == DIRECTION.South):
            map.horizontalWalls[i+1][j] = 1
        elif (dir == DIRECTION.West):
            map.verticalWalls[i][j] = 1
        elif (dir == DIRECTION.East):
            map.verticalWalls[i][j+1] = 1

    def assign_all_dirs(self, map, posn_dir_stack):
        '''Once the robot is at an unfamiliar square, it needs to survey the 
        three directions in front of it. Assume the data from the 4th direction, from
        behind the robot, is already known because the robot came from that direction.

        For each direction, front + left + right, there are two possibilities:
         1. The direction is blocked. In this case, we want to note that there is a
            wall in that direction, relative to the current square, using self.assign_wall()
         2. The direction is open. This means this direction is potentially a direction for
            the robot to survey, as long as:
             - the pair of (position, direction) is not already in the stack to check
             - the square in that direction isn't in the set of squares we've already checked
        '''

        #get current position of robot. make a copy of list to prevent aliasing
        position = self.maze_position[:]
        heading = self.heading

        #define the directions relative to the current heading of the robot
        #given that directions are modeled as N = 1, E = 2, S = 3, W = 4
        left_dir = max( (heading - 1), ((heading + 3)%5) )
        right_dir = max( (heading - 3), ((heading + 1)%5) )
        front_dir = heading

        #survey 3 directions around robot at this square. assume the 4th direction
        #is the one you came from, so no need to define it
        front_value = self.front_reading()
        left_value = self.left_reading()
        right_value = self.right_reading()

        #if a wall exists in each of these three directions, modify the obstacle map and 
        #put a wall in the given direction relative to current cell
        #else, if the direction is open and we haven't been to the cell in that direction
        #before, add posn and direction to stack of places to check next

        #we also want to change the cost of the next cell so that the robot has a way to
        #get back to a previous position
        if (right_value != -1000):
            self.assign_wall(map, position, right_dir)
        else:
            next_posn_dir = (position, right_dir)
            neighbor_coords = self.get_neighbor_coords(map, position, right_dir)
            if (neighbor_coords not in self.posns_visited):                
                posn_dir_stack.push(next_posn_dir)

        if (left_value != -1000):
            self.assign_wall(map, position, left_dir)
        else:
            next_posn_dir = (position, left_dir)
            neighbor_coords = self.get_neighbor_coords(map, position, left_dir)
            if (neighbor_coords not in self.posns_visited):                
                posn_dir_stack.push(next_posn_dir)


        #we want the front direction to be the next thing to check if possible,
        #so add it to the stack last so it's first out (if applicable)
        if (front_value != -1000):
            self.assign_wall(map, position, front_dir)
        else:
            next_posn_dir = (position, front_dir)
            neighbor_coords = self.get_neighbor_coords(map, position, right_dir)
            if (neighbor_coords not in self.posns_visited):  
                posn_dir_stack.push(next_posn_dir)

    def navigate_to_next(self, map, goal_posn, goal_heading):
        '''Part of this code involves keeping track of all the different branches
        along the robot's path where the robot can travel to explore unknown walls.
        As such, in the depth-first search (using a LIFO algorithm) the robot will
        go down certain dead ends and need to backtrack to get to the next branch
        to check. This function enables the robot to do that.
        '''
        #compare to current position and heading
        position = self.maze_position
        heading = self.heading

        #if current position is not the same as the one we need to be at, 
        #pull the list of recent positions and get a path back to that position
        if (position != goal_posn):

            print('\nNavigate_to_next: position and goal_posn')
            print(position)
            print(goal_posn)

            #make a list of positions to visit to get to goal
            posns_to_goal = []

            #append current square to recent_posns
            self.recent_posns.append(self.maze_position)

            #generate a cost map to get to the goal posn
            #takes into account walls in scene
            map.clearCostMap()
            self.gen_cost_map(map, goal_posn)

            #for visualizing
            self.print_better_cost_map(map)
            posns_to_goal = self.gen_path_coords(map, position, goal_posn)

            #now we have a list of positions to visit, change this to a list of
            #instructions for the robot
            dir_list = self.coords_to_dirs(posns_to_goal)

            #tell robot to navigate to the position it needs to be at
            for num_dir in dir_list:

                if (rospy.is_shutdown()):
                    return

                num_spaces = num_dir[0]
                dir = num_dir[1]
                self.move_in_line(num_spaces, dir, map)

        #if we need to adjust position:
        if (heading != goal_heading):
            self.adjust_heading(goal_heading)

    def first_nonrepeat_pd(self, map, posn_dir_stack):
        '''When the robot walks through a maze, there is likely more than one way
        to get to certain positions on the map. When the robot is deciding which is
        the next (posn,dir) to navigate to for analysis, we want to remove any repetitive
        (posn,dir)'s from the stack. 

        'Repetitive' in this respect is defined as such: if the neighboring cell relative
        to that position of the map, in the direction given, is in the list of cells we've
        already visited, there's no need to visit that position and travel in that direction;
        we've already gathered data on what's on that branch.
        '''

        #if our stack is empty, cannot do anything so return
        if (posn_dir_stack.is_empty()):
            return

        #pop a pair of data from the stack
        posn_dir = posn_dir_stack.pop()
        posn = posn_dir[0]
        dir = posn_dir[1]

        #find neighbor coords, and compare them to existing coordinates we've visited

        #posns_visited is a list of lists, and output of g_n_c() is a tuple, 
        #so we need to convert neighbor_coords back into a list for proper comparison
        neighbor_coords= list(self.get_neighbor_coords(map, posn, dir))

        #keep iterating to find the first (posn,dir) set that doesn't produce
        #a neighboring position that's already in the map
        while (neighbor_coords in self.posns_visited):

            if (rospy.is_shutdown()):
                return posn_dir

            #if our stack is empty, cannot do anything further
            if (posn_dir_stack.is_empty()):
                break

            #pop a pair of data from the stack
            posn_dir = posn_dir_stack.pop()
            posn = posn_dir[0]
            dir = posn_dir[1]

            #find neighbor coords, and compare them to existing coordinates we've visited
            neighbor_coords = list(self.get_neighbor_coords(map, posn, dir))

            #if the neighboring position would be out of bounds, remove it
            #as a possible option for travel
            if (neighbor_coords[0] == map.getObstacleMapSize(xDim = True) \
                    or neighbor_coords[0] == 0 or neighbor_coords[1] == 0 \
                    or neighbor_coords[1] == map.getObstacleMapSize(xDim = False)):

                #repeat the process
                posn_dir = posn_dir_stack.pop()
                posn = posn_dir[0]
                dir = posn_dir[1]
                neighbor_coords = list(self.get_neighbor_coords(map, posn, dir))

        #return the most recent posn_dir pair that will not tell the robot to 
        #go to a position that it's already visited
        return posn_dir

    def gen_obstacle_map(self, map):
        '''For use when the robot is placed into an unfamiliar environment and 
        needs to create an obstacle map. Tells the robot to scan all possible
        directions of travel from a square, then travel all of those directions
        by adding them to a Stack. New directions to travel get added to the stack,
        and the robot keeps surveying positions until no more directions exist
        in the stack.

        Can assume the robot always starts at (0,0), facing south, and that the world
        is an 8x8 square

        Returns: a map
        '''

        #initialize an instance of the Stack data type to store new positions/
        #directions to check
        posn_dir_stack = Stack()

        #check all 3 directions at current posn_dir. add walls to obstacle map
        #if they exist, and add open positions to the stack
        self.assign_all_dirs(map, posn_dir_stack)

        #if stack is empty, i.e. no more directions to check, break out of loop
        while not posn_dir_stack.is_empty():
            
            #for easy exit
            if (rospy.is_shutdown()):
                return map

            print("\nContents of the stack:")
            print(posn_dir_stack.the_stack)

            #pop an open direction from the stack
            posn_dir = self.first_nonrepeat_pd(map, posn_dir_stack)
            goal_posn = posn_dir[0]
            goal_heading = posn_dir[1]

            #tell robot to navigate to the next position 
            self.navigate_to_next(map,goal_posn, goal_heading)

            #after navigating to that position, add current position to recent posns visited
            #and all posns visited
            if (goal_posn not in self.posns_visited):
                self.posns_visited.append(goal_posn)
            self.recent_posns.append(goal_posn)

            #we know that the direction the robot is facing is now an open
            #space, so move forward 
            self.move_one_space(goal_heading, map)

            #this is so that the code is less likely to assign walls
            #after the code has already exited out
            if (rospy.is_shutdown()):
                return map

            #check all 3 directions at current posn_dir. add walls to obstacle map
            #if they exist, and add open positions to the stack
            self.assign_all_dirs(map, posn_dir_stack)

        print("\nReached the end of posn_dir_stack.")
        return map


#some things I edited that I may want to come back to later:
# - the sensor range and angle span
# - the range of acceptable sensor readings in course_correct()
# - the gait timestep: usually is 0.4, may increase to 0.5 if misbehaving
# - num_steps: increased to 13
##

#for navigating an existing maze, sensor range 0.75, 30deg and
#a front sensor of 0.75 range are better for the job

if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()
