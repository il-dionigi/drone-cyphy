import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import sys
import csv
import re

if len(sys.argv) < 3:
	print('''Error: this script takes a file with the regex-explained format d[0-9]_n?b_enter_name_here_pos.csv and then 's' or 'd' designating 'save figure' or 'display figure' ''')
	sys.exit(1)
elif not(sys.argv[2] in ['s', 'd']):
	print('''Error: this script takes either 's' or 'd' as the second input, designating 'save figure' or 'display figure' ''')
	sys.exit(1)

#r = re.search('(?<=b_)([a-z0-9]*(_[a-z0-9]*)*)_pos\.csv', sys.argv[1])

x = []
y = []
x_range = [0,0]
y_range = [0,0]
count = 0

def roundUp(x):
	return float(int(x) + 1) if x > 0 else -1 * float(abs(int(x)) + 1)

with open(sys.argv[1], 'r') as f:
	reader = csv.reader(f, delimiter=',')
	for row in reader:
		if row[0] == 'time':
			pass
		else:
			x_new = float(row[1])
			y_new = float(row[2])
			count = count + 1
			if (count >= 10):
				x.append(x_new)
				y.append(y_new)
				count = 0
			if x_new < x_range[0]:
				x_range[0] = roundUp(x_new)
			elif x_new > x_range[1]:
				x_range[1] = roundUp(x_new)
			elif y_new < y_range[0]:
				y_range[0] = roundUp(y_new)
			elif y_new > y_range[1]:
				y_range[1] = roundUp(y_new)

xlen = len(x)
ylen = len(y)

def highResPoints(x,y,factor=10):
    '''
    Take points listed in two vectors and return them at a higher
    resultion. Create at least factor*len(x) new points that include the
    original points and those spaced in between.

    Returns new x and y arrays as a tuple (x,y).
    '''

    # r is the distance spanned between pairs of points
    r = [0]
    for i in range(1,len(x)):
        dx = x[i]-x[i-1]
        dy = y[i]-y[i-1]
        r.append(np.sqrt(dx*dx+dy*dy))
    r = np.array(r)

    # rtot is a cumulative sum of r, it's used to save time
    rtot = []
    for i in range(len(r)):
        rtot.append(r[0:i].sum())
    rtot.append(r.sum())

    dr = rtot[-1]/(NPOINTS*RESFACT-1)
    xmod=[x[0]]
    ymod=[y[0]]
    rPos = 0 # current point on walk along data
    rcount = 1 
    while rPos < r.sum():
        x1,x2 = x[rcount-1],x[rcount]
        y1,y2 = y[rcount-1],y[rcount]
        dpos = rPos-rtot[rcount] 
        theta = np.arctan2((x2-x1),(y2-y1))
        rx = np.sin(theta)*dpos+x1
        ry = np.cos(theta)*dpos+y1
        xmod.append(rx)
        ymod.append(ry)
        rPos+=dr
        while rPos > rtot[rcount+1]:
            rPos = rtot[rcount+1]
            rcount+=1
            if rcount>rtot[-1]:
                break

    return xmod,ymod

def multicolored_lines():
    """
    http://nbviewer.ipython.org/github/dpsanders/matplotlib-examples/blob/master/colorline.ipynb
    http://matplotlib.org/examples/pylab_examples/multicolored_line.html
    """
    global x, y
    fig, ax = plt.subplots()
    lc = colorline(x, y, cmap='hsv')
    plt.colorbar(lc)
    plt.xlim(min(x), max(x))
    plt.ylim(min(y), max(y))
    plt.show()

def colorline(
        x, y, z=None, cmap='copper', norm=plt.Normalize(0.0, 1.0),
        linewidth=3, alpha=1.0):
    """
    http://nbviewer.ipython.org/github/dpsanders/matplotlib-examples/blob/master/colorline.ipynb
    http://matplotlib.org/examples/pylab_examples/multicolored_line.html
    Plot a colored line with coordinates x and y
    Optionally specify colors in the array z
    Optionally specify a colormap, a norm function and a line width
    """

    # Default colors equally spaced on [0,1]:
    if z is None:
        z = np.linspace(0.0, 1.0, len(x))

    # Special case if a single number:
    # to check for numerical input -- this is a hack
    if not hasattr(z, "__iter__"):
        z = np.array([z])

    z = np.asarray(z)

    segments = make_segments(x, y)
    lc = mcoll.LineCollection(segments, array=z, cmap=cmap, norm=norm,
                              linewidth=linewidth, alpha=alpha)

    ax = plt.gca()
    ax.add_collection(lc)

    return lc

def make_segments(x, y):
    """
    Create list of line segments from x and y coordinates, in the correct format
    for LineCollection: an array of the form numlines x (points per line) x 2 (x
    and y) array
    """

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    return segments

multicolored_lines()

#plt.plot(x[0:xlen/2], y[0:ylen/2], color='blue', marker='o', linestyle='None', markersize='1')
#plt.plot(x[xlen/2:xlen], y[ylen/2:ylen], color='red', marker='v', linestyle='None',  markersize='0.5')
#plt.axis([min(x), max(x), min(y), max(y)])
#plt.xlabel('x position, meters (m)', fontsize=14)
#plt.ylabel('y position, meters (m)', fontsize=14)

#if sys.argv[2] == 's':
#	plt.save("Idek.png")
#	#plt.savefig(r.group(1)+'_fig.png')
#else: # sys.argv[2] == 'd'
#	plt.show()
