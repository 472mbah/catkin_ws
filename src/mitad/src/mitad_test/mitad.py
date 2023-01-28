import math
import numpy as np
from tf.transformations import euler_from_quaternion

"""
This code was taken from stackoverflow:
https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
"""

RESET_MOVE_BUFFER = 1
MOVE_BUFFER = RESET_MOVE_BUFFER




def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def angle_between_translated(v1, v2):
    computed = angle_between(v1, v2)
    # return computed
    return math.pi - computed if computed > math.pi else computed


def getEuler(quaternion):
    lst = [quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']]
    data = euler_from_quaternion(lst)
    return data


def inferSpeedFromDistance(currentPoint, destination, rospy,  maxSpeed=0.6):
    if currentPoint < 1:
        currentPoint = 1
    if destination == 0:
        destination = 1
    ratio = (destination - currentPoint) / destination
    return math.sin(math.pi*ratio) * maxSpeed


def turnToAngle(currentAngle, destinationAngle, rate=0.5):
    return (destinationAngle - currentAngle) * rate


def addNode(node, store):
    store[f"{node[0]}:{node[1]}"] = None


def checkNode(node, store):
    return f"{node[0]}:{node[1]}" in store


def pythagoras(start, end):
    return math.sqrt((end[0]-start[0])**2 + (end[1] - start[1])**2)


def inBucket(item, obj):
    return f"{item[0]}:{item[1]}" in obj

# Ensure the cordinates do not go out of bound


def inRange(cords, dimensions):
    minY = min(dimensions[0], -dimensions[0])
    minX = min(dimensions[1], -dimensions[1])
    maxY = max(dimensions[0], -dimensions[0])
    maxX = max(dimensions[1], -dimensions[1])
    return cords[0] < minY or cords[0] >= maxY or cords[1] < minX or cords[1] >= maxX


def runMitad(start, end, dimensions, blockers, visited, rospy):

    visited_ = {}
    blockers_ = {}
    reserves = []
    #end = [9, 9]
    #dimensions = [10, 10]
    #start = [0, 0]
    path = []
    # Ensure options are valid before continuing

    def filterOptions(cords):
        # if inRange(cords, dimensions):
        #     return False
        if inBucket(cords, visited_):
            return False
        if inBucket(cords, blockers_):
            return False
        return True

    def mitad(current):

        if current[0] == end[0] and current[1] == end[1]:
            return True

        options = [
            # [current[0]-1, current[1]-1],  # diagonal top left
            # [current[0]-1, current[1]+1],  # diagonal top right
            # [current[0]+1, current[1]+1],  # diagonal bottom right
            # [current[0]+1, current[1]-1],  # diagonal bottom left
            [current[0]-1, current[1]],  # top
            [current[0], current[1]+1],  # right
            [current[0]+1, current[1]],  # bottom
            [current[0], current[1]-1],  # left
        ]

        options = list(filter(filterOptions, options))
        options_size = len(options)
        hasReserves = len(reserves) > 0
        hasOptions = options_size > 0

        if not hasOptions and hasReserves:
            options.append(reserves.pop(0))
            options_size = 1
        elif not hasOptions and not hasReserves:
            # print("Complete blockade, no more options to look at")
            rospy.loginfo("No more options!")
            return False

        # rospy.loginfo(f"Options found, {options} {blockers}")

        bestFitness = float('inf')
        bestNodeIndex = None

        # Identify which option has the best fitness using pythagoras
        for index in range(options_size):
            pythag = pythagoras(options[index], end)
            if pythag < bestFitness:
                bestFitness = pythag
                bestNodeIndex = index

        # put remainder of items into the reserves for later use
        for index in range(options_size):
            if index != bestNodeIndex:
                reserves.append(options[index])

        visited_[f"{options[bestNodeIndex][0]}:{options[bestNodeIndex][1]}"] = None
        pathResponse = mitad(options[bestNodeIndex])
        if pathResponse:
            addNode(options[bestNodeIndex], visited_)
            path.append(options[bestNodeIndex])
            return True
        return False

    mitad(start)
    path.append(start)
    return (path[::-1], visited_)


def identifyDirectionBetweenNodes(first, second):
    # top
    if first[1] == second[1] and (second[0] - first[0]) == 1:
        # COMPLETE, DO NOT MODIFY
        return (math.pi/2, "North")
        # return 0
        """
		Example:
		first    second	
		[ 9, 9 ] [ 8, 9 ]
		"""
    # left
    if first[0] == second[0] and (first[1] - second[1]) == 1:
        # COMPLETE, DO NOT MODIFY
        return (math.pi, "West")
        """
		Example:
		first    second	
		[ 9, 9 ] [ 9, 8 ]
		"""
    # bottom
    if first[1] == second[1] and (first[0] - second[0]) == 1:
        # COMPLETE, DO NOT MODIFY
        # return math.pi
        return (-math.pi/2, "South")
        """
		Example:
		first    second	
		[ 8, 8 ] [ 9, 8 ]
		"""
    # right
    if first[0] == second[0] and (second[1] - first[1]) == 1:
        # COMPLETE, DO NOT MODIFY
        return (0, "East")
        """
		Example:
		first    second	
		[ 8, 8 ] [ 8, 9 ]
		"""

    # top left
    if (second[0] - first[0]) == 1 and (first[1] - second[1]) == 1:
        # return math.pi/4
        return (3*math.pi/4, "North West")
        """
		Example:
		first    second	
		[ 9, 9 ] [ 8, 8 ]
		"""

    # top right
    if (second[0] - first[0]) == 1 and (second[1] - first[1]) == 1:
        # COMPLETE, DO NOT MODIFY
        return (math.pi/4, "North East")
        """
		Example:
		first    second	
		[ 9, 9 ] [ 8, 10 ]
		"""

    # bottom right
    if (first[0] - second[0]) == 1 and (second[1] - first[1]) == 1:
        # COMPLETE, DO NOT MODIFY
        return (-math.pi/4, "South East")
        """
		Example:
		first    second	
		[ 9, 9 ] [ 10, 10 ]
		"""

    # bottom left
    if (first[0] - second[0]) == 1 and (first[1] - second[1]) == 1:
        # return 3*math.pi/4
        return (-3*math.pi/4, "South West")
        """
		Example:
		first    second	
		[ 9, 9 ] [ 10, 8 ]
		"""

    return None


def convertBlockToCords(cords, blockSize):
    relative = [x*blockSize for x in cords]
    return {'x': relative[1], 'y': relative[0], 'z': 0}


"""

	1st step in path processing
	This function collapses the paths into units where the robot could move in a straight line to 
	reduce instructions at the end

"""


def generateVelocityMovement2(path, blockSize, rospy):

    size = len(path)
    if size <= 1:
        return None
    instructions = []
    for index in range(1, size):
        directionData = identifyDirectionBetweenNodes(
            path[index-1], path[index])

        if directionData == None:
            rospy.loginfo("Unable to extract the exact direction")
            return None

        (nextDirection, directionName) = directionData

        instructions.append({
            'type': 'movement',
            'start': path[index-1],
            'relativeEnd': convertBlockToCords(
                path[index], blockSize),
            'end': path[index],
            'directionName': directionName,
            'direction': nextDirection
        })

    # rospy.loginfo('result is:')
    # [rospy.loginfo(f"{k}\n") for k in instructions]

    return instructions


def generateVelocityMovement(path, blockSize, rospy):
    # identify where you need to go top, left, right, diagonal etc
    instructions = []
    index = 1
    size = len(path) - 1

    if size <= 0:
        return None

    """
		t for top, l for left, r for right, b for bottom and d diagonal and d_ where _
		references what we have already mentioned above
	"""

    while index < size+1:
        prevInstructIndex = len(instructions) - 1
        directionData = identifyDirectionBetweenNodes(
            path[index-1], path[index])

        if directionData == None:
            rospy.loginfo("Unable to extract the exact direction")
            return None

        (nextDirection, directionName) = directionData

        # rospy.loginfo(
        #     f"Next direction {nextDirection} {directionName} between {(path[index-1], path[index])}")
        sameDirection = True if index == 1 else nextDirection == instructions[
            prevInstructIndex]['direction']

        if index == 1:
            instructions.append(
                {'type': 'movement', 'start': path[0], 'relativeEnd': {'x': 0, 'y': 0, 'z': 0}, 'end': None, 'directionName': directionName, 'direction': nextDirection})

        if not sameDirection:
            instructions[prevInstructIndex]['end'] = path[index-1]
            instructions[prevInstructIndex]['relativeEnd'] = convertBlockToCords(
                path[index-1], blockSize)

            instructions[prevInstructIndex]['direction'] = instructions[prevInstructIndex]['direction']
            instructions.append(
                {'type': 'movement', 'start': path[index-1], 'end': None, 'direction': nextDirection, 'directionName': directionName})

        #previousDirection = nextDirection
        index += 1

    #print("Final instructions")
    if len(instructions):
        instructions[len(instructions)-1]['end'] = path[len(path)-1]
        instructions[len(
            instructions)-1]['relativeEnd'] = convertBlockToCords(path[len(path)-1], blockSize)

    rospy.loginfo('result is:')
    [rospy.loginfo(f"{k}\n") for k in instructions]

    return instructions


"""
	Second step in processing of the path
	This will add in 'turn' features to rotate the robot when turning is necessary
"""


def identifyTurningPoints(path):
    size = len(path)

    if size <= 1:
        return path

    instructions = []
    for index in range(1, size):
        previous = path[index-1]
        current = path[index]
        rotation = (
            (current['direction'] - previous['direction'])/360) * (2 * math.pi)
        instructions.append(previous)
        instructions.append({'type': 'rotation', 'rotation': rotation})
        instructions.append(current)
    return instructions

# Use sine rule to approximate velocities gradually over a range of time


def generateVelocities(maxSpeed=.6, unitDifference=0.01):
    summed = 0
    speeds = []
    while (summed <= 1):
        speeds.append(math.sin(math.pi*summed) * maxSpeed)
        summed += unitDifference
    return speeds

# calculate height of line which corresponds to y position of node given an angle


def calculateSOH(hypotenuse, angleInDegrees):
    """
    sin(theta) = o / h
    o = sin(theta) * h
    """
    radiansAngle = angleInDegrees * (math.pi / 180)
    return math.sin(radiansAngle) * hypotenuse


# calculate width of line which corresponds to x position of node given an angle
def calculateCAH(hypotenuse, angleInDegrees):
    """
    cos(theta) = a / h
    a = sin(theta) * h
    """
    radiansAngle = angleInDegrees * (math.pi / 180)
    return math.cos(radiansAngle) * hypotenuse


"""
	Standalone math.floor will round figures like -2.0009 to -3 
	instead of our proposed -2. So we build a customized version to handle negative values
"""


def mitadFloor(number):
    pos = abs(number)
    isPositive = pos == number
    floored = math.floor(pos)
    return floored if isPositive else (floored * -1)


"""
Going from cords to block unit -> Math.floor(value / block dimension)
"""


def approximateCords(cords, blockSize):
    return [mitadFloor(cords['y']/blockSize), mitadFloor(cords['x']/blockSize)]


def getInstructions(start, end, dimensions, blockers, visited, rospy, blockSize=1, transformCords=True):
    new_end = end.copy()
    start_ = approximateCords(start, blockSize) if transformCords else start
    end_ = approximateCords(new_end, blockSize) if transformCords else new_end

    (path, visitedNodes) = runMitad(start_, end_, dimensions, blockers, visited, rospy)

    # rospy.loginfo(f'Initial vals start, end')
    # rospy.loginfo(f'{start_} {end_}')
    # rospy.loginfo(f"SET {path}")
    velocityMovement = generateVelocityMovement2(path, blockSize, rospy)
    return  (velocityMovement, path, visitedNodes)

# if __name__ == "__main__":
    # generateVelocities()
    #hyp = 20
    #angle = 20
    #print(" height  ", calculateSOH( hyp, angle  ))
    #print(" width  ", calculateCAH( hyp, angle  ))
    #print(getInstructions([3, 3], [9, 9], [15, 15], {}, {}))
    #path = runMitad([3, 3], [9, 9], [15, 15])
    #path = generateVelocityMovement (path)
    #path = identifyTurningPoints(path)
    #[ print(x) for x in path  ]
#visited = {}
#blocks = {}
#print(getInstructions({ 'x':0, 'y':0, 'z':0 }, { 'x':-10, 'y':0, 'z':0 }, [15, 15], blocks, visited, 1, True))
#print( "blocks", blocks )
#print("visited ", visited)