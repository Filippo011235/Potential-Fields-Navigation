import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch

##### COEFFICIENTS:
# SCENE:
delta = 0.01   # scene(points) resolution
NoOfObstacles = 4

# ALGORITHMS:
k_p = 1    # Coefficient of final point attraction
# Repulsion force:
k_o = 10  #  Coefficient of obstacles' repulsion
d_0 = 2        # Border of obstacles' influence 
# F_rep_MaxValue - Restriction for repulsion force (prevents F_rep from closing to infinity)
# Makes plot much more readable for users, but might cause robot to start moving to obstacle,
# if the MaxValue had been breached
F_rep_MaxValue = 1/2*k_p*10*np.sqrt(2)
# = 1/2 of max. attraction force;

# For tests/visualization, display only forces of... 
DisplayForce = "N"      # ...attraction: "A"; repulsion: "R"; net force: "N"

def RandomCoordinate():
    """ Return random coordinate for an object (float). """ 
    return np.random.uniform(-10,10)

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

#############################
x_size = 10
y_size = 10

# Defining Start and Finish points
start_point=(-10,RandomCoordinate())
finish_point=(10,RandomCoordinate())

# Obstacles
obst_vect = []
# obst_vect.append((0, 0)) # Line for tests
for i in range(0,NoOfObstacles):
    obst_vect.append((RandomCoordinate(), RandomCoordinate()))

x = y = np.arange(-10.0, 10.0, delta)
X, Y = np.meshgrid(x, y)
# Matrix Z of net force affecting the robot in a given point
Z = np.zeros(X.shape)

for i in range(0, Z.shape[0]):
    for j in range(0, Z.shape[1]):
        q = (x[i], y[j]) # current point
        Z[j,i] = ForcesInAPoint(DisplayForce, q, finish_point, obst_vect) # record net force

### Plot
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111)
ax.set_title('Metoda potencjałów')
plt.imshow(Z, cmap=cm.RdYlGn,
           origin='lower', extent=[-x_size, x_size, -y_size, y_size])

plt.plot(start_point[0], start_point[1], "or", color='blue')
plt.plot(finish_point[0], finish_point[1], "or", color='blue')

for obstacle in obst_vect:
    plt.plot(obstacle[0], obstacle[1], "or", color='black')

plt.colorbar(orientation='vertical')

plt.grid(True)
plt.show()