from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import csv
import sys

fig = plt.figure()
ax = p3.Axes3D(fig)
steps = 100
data = []

def gen(n):
    phi = 0
    while phi < 2*np.pi:
        yield np.array([np.cos(phi), np.sin(phi), phi])
        phi += 2*np.pi/n

def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])


with open(sys.argv[0], 'rb') as csvfile:
    posreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    l = (len(posreader))
    l = range(l)[0::int(l/steps)]
    for i in l:
        row = posreader[i];
        if i == 0:
            print(row)
        else:
            data.append([row[1], row[2], row[3]])
N = len(data)
line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])

# Setting the axes properties
ax.set_xlim3d([-2, 2])
ax.set_xlabel('X')

ax.set_ylim3d([-2, 2])
ax.set_ylabel('Y')

ax.set_zlim3d([0, 2])
ax.set_zlabel('Z')

ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)
#ani.save('matplot003.gif', writer='imagemagick')
plt.show()

#https://stackoverflow.com/questions/38118598/3d-animation-using-matplotlib
#TY ophir carmi
