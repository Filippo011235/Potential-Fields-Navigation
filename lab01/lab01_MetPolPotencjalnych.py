import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch

def RandomCoordinate():
    """ Return random coordinate for an object (float). """ 
    return np.random.uniform(-10,11)

def ForcesInPoint():

def AttractivePotential():
    k_p = 1
    

def RepulsivePotentialFromObstacle():

##### DELTA ZWIEKSZONA ######
delta = 0.1
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
# Coefficients:
k_0 = 1
d_0 = 20


x = y = np.arange(-10.0, 10.0, delta)
X, Y = np.meshgrid(x, y)
# Matrix Z of forces affecting the robot in given point
Z = np.exp(-X**0)


fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111)
ax.set_title('Metoda potencjałów')
plt.imshow(Z, cmap=cm.RdYlGn,
           origin='lower', extent=[-x_size, x_size, -y_size, y_size],
           vmax=1, vmin=-1)


plt.plot(start_point[0], start_point[1], "or", color='blue')
plt.plot(finish_point[0], finish_point[1], "or", color='blue')

for obstacle in obst_vect:
    plt.plot(obstacle[0], obstacle[1], "or", color='black')

plt.colorbar(orientation='vertical')

plt.grid(True)
plt.show()