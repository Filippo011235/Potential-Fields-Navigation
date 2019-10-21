import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from matplotlib.path import Path
from matplotlib.patches import PathPatch


delta = 0.01
x_size = 10
y_size = 10
obst_vect = [(1, 2), (1.5, 2.3), (0,1), (1,0)]
start_point=(-10,-3)
finish_point=(10,3)

x = y = np.arange(-10.0, 10.0, delta)
X, Y = np.meshgrid(x, y)
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