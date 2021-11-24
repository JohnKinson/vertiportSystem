from numpy.core.function_base import linspace

from vertiport import vertiport
from drone import drone
import matplotlib.pyplot as plt
import time


#Example scenario of calling 3 drones to specific locations from a vertiport holding 4 drones

vertiport_parameters= {
    "dimentions": [2,4,3], #in meters
    "drone_max_capacity": 6,
    "starting_drones": 4,
    "waypoint_generation_method": "Experimental",

    #LANDINGPAD LOCATION GENERATION
    #"Standard" generates a standard 2x3 vertiport with the dimentions 2x4x3
    #"Experimental" generates landingpad locations using vertiport dimentions, drone dimentions and drone_max_capacity"
    "landingpad_location_generation": "Experimental",
    "drone_separation_distance(cm)": 25
}

drone_parameters= {
    "speed_control":1,
    "dimentions": [50,50,20], # in cm
    "dt":0.1
}

takeoff_data1 = {
    "number of drones": 1,
    "location": [200, 200, 100]
}

takeoff_data2 = {
    "number of drones": 1,
    "location": [500, 400, 250]
}

landing_data1 = {
    "number of drones": 1,
    "location": [300, 250, 340]
}

#Create vertiport and generate environment based on parameters
vertiport1 = vertiport(vertiport_parameters)
vertiport1.generateEnvironment()
vertiport1.generateWaypoints()

#take off request 1
vertiport1.takeoff(takeoff_data1)

#vertiport1.takeoff(takeoff_data2)



time.sleep(3)

vertiport1.landing(landing_data1)





#vertiportAnimation(vertiport1)

import numpy as np
from matplotlib import pyplot as plt

#print(vertiport1.landingPads)

########################### PLOT LANDINGPADS ON A 2D GRAPH #############
fig= plt.figure()
ax=plt.axes()
a = np.arange(0, len(vertiport1.landingPads), 1)
c=[]
for i in a:
    
    b = vertiport1.landingPads[i][0:2]
    c.append(b)
    
c = np.array(c)

x, y = c.T
ax.set_xlim(-100,600)
ax.set_ylim(-100,600)

#SHOW VERTIPORT TWO DIMENTIONAL SPACE
vDimX=np.multiply(vertiport_parameters["dimentions"][0],100)
vDimY=np.multiply(vertiport_parameters["dimentions"][1],100)

ax.plot(np.linspace(0, 0, 1000), np.linspace(0, vDimY, 1000))
ax.plot(np.linspace(vDimX, vDimX, 1000), np.linspace(0, vDimY, 1000))
ax.plot(np.linspace(0, vDimX, 1000), np.linspace(0, 0, 1000))
ax.plot(np.linspace(0, vDimX, 1000), np.linspace(vDimY, vDimY, 1000))

plt.scatter(x,y)
plt.grid()
plt.show()
##############################################################################