#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
import requests
from mitad_test.mitad import getInstructions, addNode, inBucket, approximateCords, convertBlockToCords, getEuler

pub = rospy.Publisher('/new_on_the_block', LaserScan, queue_size=10)
scann = LaserScan()
cmdvel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

BLOCK_SIZE = .2
INITIAL_POSITION = None

POSITION = None
ANGULAR_POSITION = None
MODE = 'init'
INSTRUCTION_SET = []
VISITEDNODES = []
START = {'x': 0, 'y': 0, 'z': 0}
# [0=i] is row
# [1=j] is column

DISTANCES = [0, -1]

DESTINATION = convertBlockToCords([1, 1], BLOCK_SIZE), # original format for cords was: {'x': 4, 'y': 0, 'z': 0}

INDEX = None
BLOCKERS = {}
VISITED = {}
PREVIOUS_BLOCKED_VERSION = [None, None]
PREVIOUS_POSITION = None
RESET_WAIT = 20
WAIT = RESET_WAIT
TURN_CONSTANT_MESSAGES = True
INIT_CONSTANT_MESSAGES = True
TIMESTEPS = 20
PATH = []
PRINTED_DONE = False
VECTOR_POSITION = [0, 0]
LOCAL_URL="http://129.12.41.185:9000/"
URL="https://turtlebotrender.momodoubah1.repl.co/"

LATEST_YAW = 0
LATEST_TURNING = 0


def send():
    try:
        currP = POSITION
        p_version = approximateCords(currP, BLOCK_SIZE)
        myobj = {'facingAngle': LATEST_YAW, 'targetAngle': LATEST_TURNING, 'blockers': BLOCKERS,
                 'visits': VISITEDNODES, 'path': PATH, 'robotPosition': p_version}
        requests.post(URL, json=myobj)
    # rospy.loginfo(f"data sent! {VECTOR_POSITION}")
    except:
        # rospy.loginfo(f'{x}')
        pass

def convertCordsToBlock(cords, block_size):
    return [math.floor(x/block_size) for x in cords][::-1]

def calculateXY(hypotenuse, angleDegrees, relativePosition, robot_theta_radians):
    radiansAngle = (angleDegrees+1) * (math.pi / 180)
    vals = [math.cos(radiansAngle)*hypotenuse,
            math.sin(radiansAngle)*hypotenuse]

    vals0a = (math.cos(robot_theta_radians)*vals[0])
    vals0b = (math.sin(robot_theta_radians)*vals[1])
    vals[0] = vals0a - vals0b

    vals1a = (math.sin(robot_theta_radians)*vals[0])
    vals1b = (math.cos(robot_theta_radians)*vals[1])
    vals[1] = vals1a + vals1b

    return vals

def response_to_odom(msg):
    global POSITION
    global INITIAL_POSITION
    global ANGULAR_POSITION

    if POSITION == None:
        POSITION = {}

    POSITION['x'] = msg.pose.pose.position.x
    POSITION['y'] = msg.pose.pose.position.y
    POSITION['z'] = msg.pose.pose.position.z

    if ANGULAR_POSITION == None:
        ANGULAR_POSITION = {}

    ANGULAR_POSITION['x'] = msg.pose.pose.orientation.x
    ANGULAR_POSITION['y'] = msg.pose.pose.orientation.y
    ANGULAR_POSITION['z'] = msg.pose.pose.orientation.z
    ANGULAR_POSITION['w'] = msg.pose.pose.orientation.w

    if INITIAL_POSITION == None:
        INITIAL_POSITION = {}
        INITIAL_POSITION['x'] = msg.pose.pose.position.x
        INITIAL_POSITION['y'] = msg.pose.pose.position.y
        INITIAL_POSITION['z'] = msg.pose.pose.position.z

def response_to_scan(msg):
    
    global INSTRUCTION_SET
    global INDEX
    global MODE
    global DESTINATION
    global INITIAL_POSITION
    global WAIT
    global RESET_WAIT
    global VISITED
    global BLOCKERS
    global INIT_CONSTANT_MESSAGES
    global TURN_CONSTANT_MESSAGES
    global PREVIOUS_BLOCKED_VERSION
    global PREVIOUS_POSITION
    global TIMESTEPS
    global DISTANCES
    global VECTOR_POSITION
    global VISITEDNODES
    global PATH

    notvalidtouse = INSTRUCTION_SET == None or (
        not type(INSTRUCTION_SET) == list) or len(INSTRUCTION_SET) <= 0 or INDEX == None or INDEX < 0 or INDEX >= len(INSTRUCTION_SET)

    if POSITION==None or ANGULAR_POSITION==None or INITIAL_POSITION==None:
        return

    identifyBlockers(msg.ranges)
    
    if MODE == 'init':
        DESTINATION = { 'x':POSITION['x']+DISTANCES[1], 'y':POSITION['y']+DISTANCES[0], 'z':0 }
        rospy.loginfo(f'Moving {DISTANCES[1]} something FORWARD and {DISTANCES[0]} something SIDE WAYS')
        MODE = None
        return

    if MODE == None:

        # try:
        transformCords = True
        if type(DESTINATION) == tuple:
            DESTINATION = DESTINATION[0]
        # rospy.loginfo(f'Good so far! {POSITION}')

        if inBucket(approximateCords(DESTINATION, BLOCK_SIZE), BLOCKERS):
            pass
                #rospy.loginfo(
                 #    f"Destination avoided {approximateCords(DESTINATION, BLOCK_SIZE)}")

        (velocityMovement, path, visitedNodes) = getInstructions(
            POSITION, DESTINATION, [20, 20], {}, {}, rospy, BLOCK_SIZE, transformCords)
                # BLOCKERS
                # VISITED
        INSTRUCTION_SET = velocityMovement
        VISITEDNODES = visitedNodes
        PATH = path
        # return
        
        notvalid = INSTRUCTION_SET == None or (
            not type(INSTRUCTION_SET) == list) or len(INSTRUCTION_SET) <= 0

        currP = POSITION
        nextD = DESTINATION
        p_version = approximateCords(currP, BLOCK_SIZE)
        d_version = approximateCords(nextD, BLOCK_SIZE)
        VECTOR_POSITION = p_version

        if notvalid:

            if INIT_CONSTANT_MESSAGES:
                if p_version[0] == d_version[0] and p_version[1] == d_version[1]:
                    rospy.loginfo(
                            "Robot already at destination (If you would like more precision, reduce the blockSize)")
                else:
                    rospy.loginfo(
                            f"Unable to find path, this is probably because too many blockades surround the robot {BLOCKERS}")
            INIT_CONSTANT_MESSAGES = False
            return

        else:
            rospy.loginfo(
                f"TURTLEBOT MOVING towards {INSTRUCTION_SET[0]['directionName']} TO GET TO {INSTRUCTION_SET[0]['end']}")
            INDEX = 0
            MODE = 'transition'
        # except Exception as e:
        #     rospy.loginfo(f"{e}")

    elif notvalidtouse:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        cmdvel_pub.publish(twist)
        INDEX = None
        INIT_CONSTANT_MESSAGES = True

def convertAngleToVector(theta):
    # radiansAngle = theta_degrees * (math.pi / 180)
    return [math.sin(theta), math.cos(theta)]


def identifyBlockers(ranges):
    (a, b, yaw) = getEuler(ANGULAR_POSITION)
    degrees_yaw = round((yaw * 180.0) / math.pi)
    translator = convertAngleToVector(yaw)
    # rospy.loginfo(
    #     f"The yaw is {degrees_yaw} with size{len(ranges)} vector is {translator}")
    max_distance = -1
    quadrant = 0
    index_tracker = 0
    for k in ranges:
        if k == float('inf'):
            index_tracker += 1
            continue
        max_distance = max(max_distance, k)
        object_direction = (index_tracker) % 360
        # posY = object_direction <= 180
        # posX = object_direction <= 45 or object_direction >= 270
        # rospy.loginfo(f"Obj {object_direction} {posX} {posY} {k}")
        position = calculateXY(
            k, object_direction, POSITION, yaw)

        converted = convertCordsToBlock(position, BLOCK_SIZE)
        # rospy.loginfo(f"converted {position} {converted}")
        addNode(converted, BLOCKERS)

        index_tracker += 1



def listener():
    
    rospy.init_node('new_on_the_block', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, response_to_odom)
    
    scan_Sub = rospy.Subscriber('/scan', LaserScan, response_to_scan)

    rospy.loginfo("Started!!!!")
    rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("Started!!! 2 !")
    #listener()


rospy.init_node('new_on_the_block', anonymous=True)
odom_sub = rospy.Subscriber('/odom', Odometry, response_to_odom)
scan_Sub = rospy.Subscriber('/scan', LaserScan, response_to_scan)
rospy.loginfo("Started!!!!")
    #rospy.spin()

twist = Twist()
twist.linear.x = .0
twist.angular.z = 0

COUNT = 0

while not rospy.is_shutdown():

    send()

    if INDEX == None:
        twist.linear.x = .0
        twist.angular.z = 0
    elif INDEX >= len(INSTRUCTION_SET):
        if not PRINTED_DONE:
            rospy.loginfo("I think we have reached our destination")
            PRINTED_DONE = True

    elif INDEX < len(INSTRUCTION_SET):

        newPos = POSITION
        blocked_version = approximateCords(newPos, BLOCK_SIZE)

        if PREVIOUS_BLOCKED_VERSION[0] != blocked_version[0] and PREVIOUS_BLOCKED_VERSION[1] != blocked_version[1]:
            rospy.loginfo(f"New block {PREVIOUS_BLOCKED_VERSION}")

        if blocked_version[0] == INSTRUCTION_SET[INDEX]['end'][0] and blocked_version[1] == INSTRUCTION_SET[INDEX]['end'][1]:
            INDEX += 1
        else:
            incX = INSTRUCTION_SET[INDEX]['relativeEnd']['x'] - POSITION['x']
            incY = INSTRUCTION_SET[INDEX]['relativeEnd']['y'] - POSITION['y']
            
            angleToTheGoal = LATEST_TURNING if MODE=='turn' else math.atan2(incY, incX)
            (a, b, yaw) = getEuler(ANGULAR_POSITION)

            if abs(angleToTheGoal - yaw) > 1.5:
                MODE = 'turn'
                twist.linear.x = 0
                twist.angular.z = 2
            else:
                MODE = 'move'
                twist.angular.z = 0
                twist.linear.x = .3
            LATEST_YAW = yaw
            LATEST_TURNING = angleToTheGoal
            cmdvel_pub.publish(twist)

        PREVIOUS_BLOCKED_VERSION = blocked_version.copy()

    cmdvel_pub.publish(twist)

twist.angular.z = 0
twist.linear.x = 0
cmdvel_pub.publish(twist)
