import serial
import matplotlib.pyplot as plt
import numpy as np
import re

plt.ion()
#fig=plt.figure()
fig, ax0 = plt.subplots()
refAngle = 1.2*np.pi 
#ax0.set_xlabel('time (s)')

ax0.grid()

ax0.set_xlabel('time')
plt.xlim([0,5])
ax0.set_ylabel('Angle (rad)')


ax0.axhline(refAngle/np.pi,linewidth=1,color = 'gray')


i  = 0
x  = list()
y  = list()
dt = 0.1

ser = serial.Serial('/dev/ttyACM0',9600)
ser.close()
ser.open()

while True:
    ser.flushInput()

    data = ser.readline()
    
    #x.append(i*dt)
    #y.append(data.decode())
    data = data.decode()
    data = [float(x) for x in data.split()]

    check1 = len( data )
    if check1 == 2: 
        ax0.plot(data[0], data[1]/np.pi, 'b.-')
    else:
        i = i



    i += 1
    plt.show()
    plt.pause(dt)  # Note this correction
