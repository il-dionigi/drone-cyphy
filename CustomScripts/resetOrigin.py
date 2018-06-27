import sys
import csv

new_positions = []

new_origin = None

with open(sys.argv[1], 'r') as f:
	reader = csv.reader(f, delimiter=',')
	for row in reader:
		if row[0] == 'time':
			pass
		elif new_origin == None:
			new_origin = row
			for i in range(len(new_origin)):
				new_origin[i] = float(new_origin[i])
		else:
			new_positions.append([float(row[0]) - new_origin[0], float(row[1])-(new_origin[1]), float(row[2])-(new_origin[2]), float(row[3])-(new_origin[3])])
	f.close()

with open(sys.argv[1][:-4]+'_adj.csv', 'w') as f:
	fieldnames = ['time', 'x_pos', 'y_pos', 'z_pos']
	writer = csv.DictWriter(f, fieldnames=fieldnames)
	writer.writeheader()
	for row in new_positions:
		writer.writerow({'time':row[0], 'x_pos':row[1], 'y_pos':row[2], 'z_pos':row[3]})
	f.close()
