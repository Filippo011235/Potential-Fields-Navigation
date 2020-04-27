import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch

##### COEFFICIENTS:
# SCENE:
x_size = 10
y_size = 10
delta = 0.1   # scene(points) resolution
NoOfObstacles = 7

# For subparagraph d - Variation where Robot can only sense Obstacles in range d_r 
# CODE FOR THIS MODE IS INSIDE WhichCellNext FUNCTION!!!!
OnlyAreaInRangeKnown = True
d_r = 1

# ALGORITHMS:
k_p = 5    # Coefficient of final point attraction
# Repulsion force:
k_o = 100  #  Coefficient of obstacles' repulsion
d_0 = 5        # Border of obstacles' influence 
# F_rep_MaxValue - Restriction for repulsion force (prevents F_rep from closing to infinity)
# Makes plot much more readable for users, but might cause robot to start moving to obstacle,
# if the MaxValue had been breached
F_rep_MaxValue = 4*k_p*10*np.sqrt(2)
# = 3* max. attraction force;
PathLength = 500 # maximum length(number of points) for Path; breaks the loop if breached
# For tests/visualization, display only forces of... 
DisplayForce = "N"      # ...attraction: "A"; repulsion: "R"; net force: "N"

####### FUNCTIONS #######

Accuracy = int(np.log10(1/delta)) # used in ReturnRounded
def ReturnRounded(SingleCoord):
    '''Return SingleCoord(X or Y) rounded to Accuracy based on scene resulotion delta'''
    return round(SingleCoord, Accuracy)

def RandomCoordinate():
    """ Return random coordinate for an object rounded to accuracy of delta (float). """ 
    return ReturnRounded(np.random.uniform(-10,10))

def CalculateDistance(q1, q2):
    """Return distance between two 2D points"""
    return np.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)

def AttractionForce(q,q_k):
    """Return F_att for point q, from finish_point q_k"""
    return k_p*CalculateDistance(q,q_k)
    
def RepulsionForceFromObstacle(q,q_oi):
    """Return repulsion force on point q, from obstacle q_oi"""
    d_i = CalculateDistance(q,q_oi) # Distance to the obstacle
    if (d_i < d_0):     # Point within obstacle influence
        F_oi = k_o * (1/d_i - 1/d_0) /d_i**2    
        if (F_oi > F_rep_MaxValue):     # F_rep won't exceed set constant
            return F_rep_MaxValue
    else:   # Point without obstacle influence
        F_oi = 0
    return F_oi

def RepulsionForcesInAPoint(q,ObstVector):
    """Return net repulsion force acting on a point q, based on a list of obstacles ObstVector"""
    F_rep = 0
    for q_oi in ObstVector:             # For each obstacle...
        F_rep += RepulsionForceFromObstacle(q,q_oi)     #... add its repulsion to net F_rep
    return F_rep

def ForcesInAPoint(WhichForce, q,q_k,ObstVector):
    """Calculate net force on a point q"""
    """WhichForce determines what kind of forces: F_att "A"; F_rep "R", F_net "N"""
    return {
        'N': AttractionForce(q,q_k) + RepulsionForcesInAPoint(q, ObstVector),
        'A': AttractionForce(q,q_k),
        'R': RepulsionForcesInAPoint(q, ObstVector)
    }[WhichForce]

def ForceVectorComponents(q, SceneObject, isElementObstacle):
    ''' 
    Function returns Components of Force Vector(from Obstacle/Finish).
    Treat q as a center of local coordinate system -> (0,0). Each Vector will start at the center
    and Components will be in relation to it. In most cases, X and Y can be calculated using Force 
    value, equation for linear function, and Pitagoras theorem. 
    
    There are two special cases: SceneObject is directly above or below q (0,0).
    Then X equals 0, and Y simply Force Value, + or -, depends on SceneObject(Obs/Finish).
    '''
    if isElementObstacle:
        # Local => SceneObject has coord. in relation to q
        LocalObstacle = (SceneObject[0] - q[0], SceneObject[1] - q[1])
        if LocalObstacle[0] != 0: # Local X different than zero -> linear function
            aa = LocalObstacle[1]/LocalObstacle[0]  # "aa" as in "y = aa*x" function
            ForceSquared = RepulsionForceFromObstacle(q, SceneObject)**2
            X_Distance = np.sqrt(ForceSquared/(aa**2 + 1))
            if LocalObstacle[0] < 0:    # is west to the q
                X_force = X_Distance
            else:   # is east to the q
                X_force = -X_Distance
            Y_force = aa*X_force 
        else: # Local X = 0; special case
            X_force = 0
            Y_force = -RepulsionForceFromObstacle(q, SceneObject)
        
    else:# it's a attracting finish point 
        LocalFinish = (SceneObject[0] - q[0], SceneObject[1] - q[1])
        if LocalFinish[0] != 0: # Local X different than zero -> linear function
            aa = LocalFinish[1]/LocalFinish[0]  # "aa" as in "y = aa*x" function
            ForceSquared = AttractionForce(q, SceneObject)**2
            X_Distance = np.sqrt(ForceSquared/(aa**2 + 1))
            if LocalFinish[0] < 0:    # is west to the q
                X_force = - X_Distance
            else:   # is east to the q
                X_force = X_Distance
            Y_force = aa*X_force 
        else: # Local X = 0; special case
            X_force = 0
            Y_force = AttractionForce(q, SceneObject)

    return (X_force, Y_force) # end coordinates of given Force vector

def ResultantForceComponents(ListOfVectors):
    '''Resultant force components equal to sum of single vectors components.
        Return resultant force components/coordinates.'''
    X_sum = Y_sum = 0
    for SingleVector in ListOfVectors:
        X_sum += SingleVector[0]
        Y_sum += SingleVector[1]
    return (X_sum, Y_sum)

def ValueInRange(Min, Value, Max):
    '''return True if Value is between Min and Max, Max included'''
    if Min < Value and Value <= Max:
        return True
    else:
        return False

def AssingCell(ResultantForce):
    '''Choose which out of 8 neighbour cells corresponds best to the vectors' angle'''
    '''Returns tuple which will be added to the current cell'''
    ForceAngle = np.arctan2(ResultantForce[1],ResultantForce[0]) # in rad
    ForceAngle = ForceAngle/np.pi # for simplyfication
    # print("ForceAngle", ForceAngle)
    if ValueInRange(-1/8, ForceAngle, 1/8):
        return (delta, 0) # Right
    elif ValueInRange(1/8, ForceAngle, 3/8):
        return (delta, delta) # Right Up
    elif ValueInRange(3/8, ForceAngle, 5/8):
        return (0, delta) # Up
    elif ValueInRange(5/8, ForceAngle, 7/8):
        return (-delta, delta) # Left Up
    elif ValueInRange(7/8, ForceAngle, 1) or ValueInRange(-1, ForceAngle, -7/8): # special case
        return (-delta, 0) # Left
    elif ValueInRange(-7/8, ForceAngle, -5/8):
        return (-delta, -delta) # Left Down
    elif ValueInRange(-5/8, ForceAngle, -3/8):
        return (0, -delta) # Down
    elif ValueInRange(-3/8, ForceAngle, -1/8):
        return (+delta, -delta) # Right Down

def WhichCellNext(q, ObstVector, finish_point):
    '''Return which cell to move to, based on location'''
    SingleVectorsCoords = []

    SingleVectorsCoords.append(ForceVectorComponents(q, finish_point, False))

    for Obst in ObstVector:
        
        ################## SUBPARAGRAPH D) ##################
        if OnlyAreaInRangeKnown:   # Switch between all obst mode, and only in range
            if CalculateDistance(q, Obst) < d_r:
                SingleVectorsCoords.append(ForceVectorComponents(q, Obst, True))     
        ################## SUBPARAGRAPH D) ##################


        else:          # All Obstacles are known
            SingleVectorsCoords.append(ForceVectorComponents(q, Obst, True))

    F_res = ResultantForceComponents(SingleVectorsCoords)
    
    Direction = AssingCell(F_res)
    return Direction

def KeepWithinBorder(coord, sign, border):
    '''
    Prevent robot from reaching position values beyond the scene' border
    coord - coordinate x or y of a point
    sign - "<" or ">"
    border - max(or min) value which robot can reach
    '''
    if sign == "<":
        if coord < border:
            return coord
        else:
            return border
    elif sign == ">":
        if coord > border:
            return coord
        else:
            return border
    else:
        raise ValueError("Incorrect sign. Use: \"<\", \">\"")

def KeepWithinScene(SuggestedPoint):
    '''
    Prevent robot from reaching positions beyond the scene
    Returns corrected Point(if corrections were needed)
    SuggestedPoint - coordinate x or y of a point
    x_size, y_size - scene size; global name
    '''
    SuggestedPoint[0] = KeepWithinBorder(SuggestedPoint[0], "<", x_size)
    SuggestedPoint[0] = KeepWithinBorder(SuggestedPoint[0], ">", -x_size)
    SuggestedPoint[1] = KeepWithinBorder(SuggestedPoint[1], "<", y_size)
    SuggestedPoint[1] = KeepWithinBorder(SuggestedPoint[1], ">", -y_size)
    
    return SuggestedPoint

def CalculateNextPoint(Path, finish_point, obst_vect):
    '''Return processed Point to be added to the Path'''
    Direction = WhichCellNext(Path[-1], obst_vect, finish_point)
    RawPoint = list(map(lambda x, y: x + y, Path[-1], Direction))
    RawPoint = KeepWithinScene(RawPoint)
    RoundedNextPoint = (ReturnRounded(RawPoint[0]), ReturnRounded(RawPoint[1]))
    return RoundedNextPoint


def CreatePath(start_point, finish_point, obst_vect):
    '''Return list of Path points(tuples)'''
    Path = []   # list of path points (tuples)
    Path.append(start_point) # beginning
    i = 0 # auxiliary variable
    
    Path.append(CalculateNextPoint(Path, finish_point,obst_vect))

    while (Path[-1] != finish_point):
        NextPoint = CalculateNextPoint(Path, finish_point,obst_vect)

        if NextPoint == Path[-2]: # Robot might alternate between 2 positions
            # make a new move in a random direction
            Rand_X = np.random.choice([-delta, 0, delta])
            Rand_Y = np.random.choice([-delta, 0, delta])
            NewDir = (Rand_X, Rand_Y)
            RandomPoint = tuple(map(lambda x, y: x + y, NextPoint, NewDir))
    
            Path.append(RandomPoint)
        else: 
            Path.append(NextPoint)
        
        i += 1 
        if i > PathLength: # Path points limit
            print("Couldn't finish the Path in %s iterations :(" % PathLength)
            break
        # assert(i < PathLength), "Couldn't finish the Path in 100000 iterations :("
    return Path

#############################
x = y = np.arange(-10.0, 10.0, delta)
X, Y = np.meshgrid(x, y)

# START + FINISH points
start_point=(-x_size,RandomCoordinate())
finish_point=(x_size,RandomCoordinate())

# start_point=(-y_size,-10) # TESTS
# finish_point=(y_size,10) # TESTS

# OBSTACLES
obst_vect = []
# obst_vect.append((0, 0)) # TESTS
for i in range(0,NoOfObstacles):
    obst_vect.append((RandomCoordinate(), RandomCoordinate()))

# Z MATRIX - net force affecting the robot in a given point
Z = np.zeros(X.shape)
for i in range(0, Z.shape[0]):
    for j in range(0, Z.shape[1]):
        q = (x[i], y[j]) # current point
        Z[j,i] = ForcesInAPoint(DisplayForce, q, finish_point, obst_vect) # calculate net force

# PATH
RobotsPath = CreatePath(start_point, finish_point, obst_vect)

# PLOT
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111)
ax.set_title('Metoda potencjałów')
plt.imshow(Z, cmap=cm.RdYlGn,
           origin='lower', extent=[-x_size, x_size, -y_size, y_size])

plt.plot(start_point[0], start_point[1], "or", color='blue')
plt.plot(finish_point[0], finish_point[1], "or", color='blue')

for obstacle in obst_vect:
    plt.plot(obstacle[0], obstacle[1], "or", color='black')

for PathPoint in RobotsPath:
    plt.plot(PathPoint[0], PathPoint[1], "or", color='blue')

plt.colorbar(orientation='vertical')

plt.grid(True)
plt.show()

