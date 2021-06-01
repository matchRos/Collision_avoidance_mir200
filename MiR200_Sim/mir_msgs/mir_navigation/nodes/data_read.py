#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np


with open("/home/ros_match/Documents/Masterarbeit/Energieverbrauch/Sim_En/AD/ac_x3.txt") as f:
    data1 = f.read()
    data1 = data1.split()
    #data = float(data)
with open("/home/ros_match/Documents/Masterarbeit/Energieverbrauch/Sim_En/AD/ac_y3.txt") as f:
    data2 = f.read()
    data2 = data2.split()
with open("/home/ros_match/Documents/Masterarbeit/Energieverbrauch/Sim_En/navfn/ac_x3.txt") as f:
    data3 = f.read()
    data3 = data3.split()
with open("/home/ros_match/Documents/Masterarbeit/Energieverbrauch/Sim_En/navfn/ac_y3.txt") as f:
    data4 = f.read()
    data4 = data4.split()


     


#x = np.loadtxt('file.txt', delimiter=',', unpack=True)
#plt.plot(x, label='Loaded from file!')
#with open("file.txt", 'r') as f:
    #score = [line.rstrip('\n') for line in f]

#plt.plot(score)
#plt.ylabel('ax')
#plt.show()

#plt.xlabel('x')
#plt.ylabel('y')
#plt.title('Interesting Graph\nCheck it out')
#plt.legend()
#plt.show()



x1 = [row.split(' ')[0] for row in data1]
x2 = [row.split(' ')[0] for row in data2]
x3 = [row.split(' ')[0] for row in data3]
x4 = [row.split(' ')[0] for row in data4]
#y = [row.split(' ')[1] for row in data]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Verteilung der linear x- und y-Beschleunigung (Z2-Z3)", fontsize=30)    
ax1.set_xlabel('Zeit (s)',fontsize=30)
ax1.set_ylabel('Beschleunigung (m/s^2)', fontsize=30)

ax1.plot(x1, c='r', label='x_Beschleunigung_AD')
ax1.plot(x2, c='g', label='y_Beschleunigung_AD')
ax1.plot(x3, c='b', label='x_Beschleunigung_navfn')
ax1.plot(x4, c='y', label='y_Beschleunigung_navfn')

ax1.xaxis.set_tick_params(labelsize=23)
ax1.yaxis.set_tick_params(labelsize=23)

#ax1.legend(prop=dict(size=60))

#plt.legend(fontsize=50)
leg = ax1.legend(fontsize=30)

plt.show()
