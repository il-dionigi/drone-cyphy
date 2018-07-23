from Tkinter import *
from random import randint
 
# these two imports are important
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time
import threading
 
continuePlotting = False
index = 0
xdataPoints = [] 
ydataPoints = []

def change_state():
    global continuePlotting
    if continuePlotting == True:
        continuePlotting = False
    else:
        continuePlotting = True
    
 
def data_points():
    global index, xdataPoints, ydataPoints
    f = open("data.txt", "r")
    data = f.readlines()
    f.close()
	fileLength = len(data)
    for i in range(index, fileLength):
		row = data[i].split(',')
		xy = [int(row[0]), int(row[1])]
        xdataPoints.append(xy[0])
		ydataPoints.append(xy[1])
		print(xy)
	index = fileLenth 
 
def app():
	global xdataPoints, ydataPoints
    # initialise a window.
    root = Tk()
    root.config(background='white')
    root.geometry("1000x700")
    
    lab = Label(root, text="Live Plotting", bg = 'white').pack()
    
    fig = Figure()
    
    ax = fig.add_subplot(111)
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.grid()
 
    graph = FigureCanvasTkAgg(fig, master=root)
    graph.get_tk_widget().pack(side="top",fill='both',expand=True)
 
    def plotter():
        while continuePlotting:
            ax.cla()
            ax.grid()
            ax.plot(xdataPoints, ydataPoints, marker='o', color='orange')
            graph.draw()
            time.sleep(1)
 
    def gui_handler():
        change_state()
        threading.Thread(target=plotter).start()
 
    b = Button(root, text="Start/Stop", command=gui_handler, bg="red", fg="white")
    b.pack()
    
    root.mainloop()
 
if __name__ == '__main__':
    app()