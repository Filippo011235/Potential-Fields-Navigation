import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch


def RandomCoordinate():
    """ Return random coordinate for an object (float). """ 
    return np.random.uniform(-10,10)

def CalculateDistance(q1, q2):
    return np.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)

def AttractionForce(q,q_k):
    return k_p*CalculateDistance(q,q_k)
    
def RepulsionForceFromObstacle(q,q_oi):
    d_i = CalculateDistance(q,q_oi)
    if (d_i < d_0):                                     # <= is pointless???
        F_oi = k_0 * (1/d_i - 1/d_0) / d_i**2
        if (F_oi > Max_Attraction):     # F_rep won't exceed value of F_att, for max distance 
            return Max_Attraction
    else:
        F_oi = 0
    return F_oi

def RepulsionForcesInAPoint(q,ObstVector):
    F_rep = 0
    for q_oi in ObstVector:
        F_rep += RepulsionForceFromObstacle(q,q_oi)
    return F_rep

def ForcesInAPoint(q,q_k,ObstVector):
    # return AttractionForce(q,q_k)
    # return RepulsionForcesInAPoint(q, ObstVector)
    return AttractionForce(q,q_k) + RepulsionForcesInAPoint(q, ObstVector)

##### DELTA ZWIEKSZONA ######
delta = 0.01
#############################
x_size = 10
y_size = 10

# Defining Start and Finish points, and Obstacles
start_point=(-10,RandomCoordinate())
finish_point=(10,RandomCoordinate())

### OBSTACLES
obst_vect = []
NoOfObstacles = 4
for i in range(0,NoOfObstacles):
    obst_vect.append((RandomCoordinate(), RandomCoordinate()))
# obst_vect.append((0, 0)) # For tests


# Coefficients:
#### WNIOSKI
'''
d_0 nie jest tak ważne jak k_0 -> Duże d, ale małe k, liczy dla większej ilości punktów(koszty obliczeniowe programu)
 a daje coraz mniejszą różnicę. Wzrost d, w parze z k_0 powinien

'''
k_0 = 1
k_p = 1
Max_Attraction = AttractionForce((-x_size,-y_size),(x_size,y_size)) # -> RepulsionForceFromObstacle
d_0 = 20

x = y = np.arange(-10.0, 10.0, delta)
X, Y = np.meshgrid(x, y)
# Matrix Z of forces affecting the robot in a given point
Z = np.zeros(X.shape)
# Z = np.exp(-X**0)

Z_rows = Z.shape[0]
Z_cols = Z.shape[1]

for i in range(0, Z_rows):
    for j in range(0, Z_cols):
        q = (x[i], y[j])
        # print(q)
        Z[j,i] = ForcesInAPoint(q, finish_point, obst_vect)

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