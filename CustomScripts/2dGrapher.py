import matplotlib.pyplot as plt
import sys
import csv
import re

if len(sys.argv) < 3:
	print('''Error: this script takes a file with the regex-explained format d[0-9]_n?b_enter_name_here_pos.csv and then 's' or 'd' designating 'save figure' or 'display figure' ''')
	sys.exit(1)
elif not(sys.argv[2] in ['s', 'd']):
	print('''Error: this script takes either 's' or 'd' as the second input, designating 'save figure' or 'display figure' ''')
	sys.exit(1)

r = re.search('(?<=b_)([a-z]*(_[a-z]*)*)_pos\.csv', sys.argv[1])

x = []
y = []
x_range = [0,0]
y_range = [0,0]

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
			x.append(x_new)
			y.append(y_new)
			if x_new < x_range[0]:
				x_range[0] = roundUp(x_new)
			elif x_new > x_range[1]:
				x_range[1] = roundUp(x_new)
			elif y_new < y_range[0]:
				y_range[0] = roundUp(y_new)
			elif y_new > y_range[1]:
				y_range[1] = roundUp(y_new)

plt.plot(x, y, 'ro', markersize='0.5')
plt.axis([x_range[0], x_range[1], y_range[0], y_range[1]])
plt.xlabel('x position, meters (m)', fontsize=14)
plt.ylabel('y position, meters (m)', fontsize=14)

if sys.argv[2] == 's':
	plt.savefig(r.group(1)+'_fig.png')
else: # sys.argv[2] == 'd'
	plt.show()