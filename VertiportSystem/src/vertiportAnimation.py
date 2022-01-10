from Scenario1 import drone1_takeoff
from Scenario1 import drone2_landing
from Scenario1 import vertiport1

from Scenario1 import vertiport_parameters
from Scenario1 import drone_parameters

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
 

takeoff_pos, time1 = drone1_takeoff

landing_pos, time = drone2_landing #drone1_takeoff
landingpads = vertiport1.landingPads

print("landing_pos===",landing_pos, "time==", time)
takeoff_pos = takeoff_pos[::10]
landing_pos = landing_pos[::10]
time = time[::10]
time1 = time1[::10]


# ANIMATION FUNCTION
def func(num, dataSet,  landingDots, takeoffDots):#landing_pads, line, "
    # NOTE: there is no .set_data() for 3 dim data...

    takeoffDots.set_data(dataSet[0:2, :num])    
    takeoffDots.set_3d_properties(dataSet[2, :num]) 

    landingDots.set_data(dataSet[0:2, :num])    
    landingDots.set_3d_properties(dataSet[2, :num]) 

    
    #landing_pads.set_data(dataSet[0:2, :num])
    #landing_pads.set_3d_properties(dataSet[0:2, :num])

    return  takeoffDots,landingDots   #line    

#takeoff animation points
x1=[]
pos1len=np.arange(0, len(takeoff_pos), 1)
for i in pos1len:
    xi1= takeoff_pos[i][0]
    x1.append(xi1)

y1=[]
for i in pos1len:
    yi1= takeoff_pos[i][1]
    y1.append(yi1)

z1=[]
for i in pos1len:
    zi1= takeoff_pos[i][2]
    z1.append(zi1)




#Landing animation points
x=[]
poslen=np.arange(0, len(landing_pos), 1)
for i in poslen:
    xi= landing_pos[i][0]
    x.append(xi)

y=[]
for i in poslen:
    yi= landing_pos[i][1]
    y.append(yi)

z=[]
for i in poslen:
    zi= landing_pos[i][2]
    z.append(zi)
 
# THE DATA POINTS
dataSet1= np.array([x1,y1,z1])
numDataPoints1 = round(len(time1)/10)

dataSet = np.array([x, y, z])
numDataPoints = time
 
# GET SOME MATPLOTLIB OBJECTS
fig = plt.figure()
ax = Axes3D(fig)

#landing_pads = plt.plot(landingpads[0], landingpads[1],landingpads[2], c='b', marker='o')[0]


takeoffDots = plt.plot(dataSet1[0], dataSet1[1], dataSet1[2], lw=2, c='b', marker='o', zorder=5)[0] # For scatter plot

landingDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='r', marker='o', zorder= 0)[0] # For scatter plot
# NOTE: Can't pass empty arrays into 3d version of plot()
#line = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='g')[0] # For line plot
 
# AXES PROPERTIES]
# ax.set_xlim3d([limit0, limit1])
ax.set_xlabel('X(t)')
#ax.set_xlim3d([-200, 400.0])
ax.set_ylabel('Y(t)')
#ax.set_ylim3d([-100, 500.0])
ax.set_zlabel('Z(t)')
#ax.set_zlim3d([0.0, 400.0])
ax.set_title('Vertiport animation')
ax.auto_scale_xyz([-200, 400.0], [-100, 500.0], [0.0, 400.0])




a = np.arange(0, len(vertiport1.landingPads), 1)
c=[]
for i in a:
    
    b = vertiport1.landingPads[i][0:2]
    c.append(b)
    
c = np.array(c)

an = np.linspace(0, 2 * np.pi, 100)
landingPadSize = drone_parameters["dimentions"][0:2]

for i in a:
    #circle = vertiport1.landingPads[i][0:2]
    ax.plot((landingPadSize[0]/2 * np.cos(an))+vertiport1.landingPads[i][0], (landingPadSize[1]/2 * np.sin(an))+vertiport1.landingPads[i][1])

vDimX=np.multiply(vertiport_parameters["dimentions"][0],100)
vDimY=np.multiply(vertiport_parameters["dimentions"][1],100)

ax.plot(np.linspace(0, 0, 1000), np.linspace(0, vDimY, 1000))
ax.plot(np.linspace(vDimX, vDimX, 1000), np.linspace(0, vDimY, 1000))
ax.plot(np.linspace(0, vDimX, 1000), np.linspace(0, 0, 1000))
ax.plot(np.linspace(0, vDimX, 1000), np.linspace(vDimY, vDimY, 1000))



# Creating the Animation object
anim = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(dataSet, takeoffDots, landingDots), interval=10, blit=False)

#writergif = animation.PillowWriter(fps=30)
#anim.save('Animation.gif',writer=writergif)
#anim.save(r'Animation.mp4', fps= 30, dpi= 80)
 
 
plt.show()
