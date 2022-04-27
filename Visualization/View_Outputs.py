import serial
import matplotlib.pyplot as plt
import numpy as np
import re

plt.ion()
#fig=plt.figure()
fig, (ax0,ax1) = plt.subplots(2,1)
refAngle = np.pi 
#ax0.set_xlabel('time (s)')

ax0.grid()
ax1.grid()

ax1.set_xlabel('time (s)')

ax0.set_ylabel('theta')
ax1.set_ylabel('nn')

#ax0.axhline(refAngle/np.pi,linewidth=1,color = 'green')
#ax0.axhline(561,linewidth = 1,color = 'green')


i  = 0
x  = list()
y  = list()
dt = 0.1

ser = serial.Serial('/dev/ttyACM0',9600)
ser.close()
ser.open()

fname = "/home/david/Dropbox/RomePosdoc/PhaseLockedLoop/outputData/trial3.txt"
f = open(fname,'w') 
f.close() 

mm = 1
while mm < 100000:
    ser.flushInput()

    data = ser.readline()
    data0 = data
    #x.append(i*dt)
    #y.append(data.decode())
    data = data.decode()
    data = [float(x) for x in data.split()]

    check1 = len( data )
    if check1 == 3: 
        #ax0.plot(data[0], data[1],'b.-')
        #ax1.plot(data[0], data[2],'r.-')
        ax0.plot(mm, data[1],'b.-')
        ax1.plot(mm, data[2],'r.-')
        #data0 = str(data[0]) + "     " + str(data[1]) + "\n" 
        #f = open(fname,"a")
        #f.writelines(data0)
        mm += 1
        #f.close()

    else:
        i = i

    i += 1
    plt.show()
    plt.pause(dt)  # Note this correction
