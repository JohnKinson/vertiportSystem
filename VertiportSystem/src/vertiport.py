import numpy as np
import matplotlib.pyplot as plt
from drone import drone
import math

drone_parameters= {
    "speed_control":1,
    "dimentions": [50,50,20], # in cm
    "dt":0.1
}


class vertiport():

    def __init__(self, params):
        
        #Assign to self object
        self.droneN= params["starting_drones"]
        self.vertiportDimentions = params["dimentions"]
        self.drone_max_capacity = params["drone_max_capacity"]
        self.waypoint_generation_method=  params["waypoint_generation_method"]
        self.landingpad_location_generation = params["landingpad_location_generation"]
        self.drone_separation= params["drone_separation_distance(cm)"]
        #self.landingPad =[]# defining so it can be used later to iterate through and store generated landingpads
        self.landingPads =[] #landing pad location array
        self.waypoints_in = [] #landing path
        self.waypoints_out = [] #takeoff path
        self.UAVs=[] #array of UAV objects within the vertiport
        self.takeoff_in_progress = False
        self.landing_in_progess = False

        if self.droneN < 0 or self.droneN > self.drone_max_capacity:
            print("number of starting drones must be >= 0 and <= ", self.drone_max_capacity)
        #Argument validations
        #assert self.droneN >0, f"Starting drones for vertiport, {self.droneN} is less than zero"
        #assert self.droneN <=6, f"Starting drones for vertiport, {self.droneN} is greater than the maximum number of 6"
    
    def generateEnvironment(self):
        if self.landingpad_location_generation == "Experimental":
            drone_size = np.array(drone_parameters["dimentions"])
            drone_size = drone_size/100 #convert to m from cm
            droneX= drone_size[0]
            droneY= drone_size[1]
            vertiX = self.vertiportDimentions[0]
            vertiY= self.vertiportDimentions[1]
            
            self.drone_separation = self.drone_separation/100 #convert to m from cm

            #calculating how many drones can fit on the X axis while also accountng for drone separation distance (this also includes a separation from the edge of the vertiport boundaries)
            x_comparison =  (vertiX-self.drone_separation)/(droneX+self.drone_separation)
            x_comparison = math.floor(x_comparison) #round down to nearest integer provides the most drones that will fit along the x axis
            y_comparison = (vertiY-self.drone_separation)/(droneY+self.drone_separation)
            y_comparison = math.floor(y_comparison) #round down to nearest integer provides the most drones that will fit along the y axis
            
            if y_comparison < 1 or x_comparison <1:
                print("The current parameters are not feasable to hold any drones!! Please increase vertiport dimentions!, reduce drone size! or decrease drone separation distance!!!")
            
            elif y_comparison > 0 and x_comparison >0:
                max_possible_drones =x_comparison*y_comparison
                
                if self.drone_max_capacity > max_possible_drones:
                    print("Hello! Unfortunately the current parameters are not feasable to hold the number of drones specified! Currently the vertiport can only hold ", max_possible_drones ," drones. Please increase vertiport dimentions, reduce drone size, number of drones or decrease drone separation distance!!")
                
                elif self.drone_max_capacity <= max_possible_drones:
                    #Generate loactions for landing pads going right to left top to bottom
                    rows = []
                    x_comparison_array=np.arange(0, x_comparison, 1) # make iterable
                    
                    for i in x_comparison_array:
                        
                        row = (self.drone_separation)+(droneX/2)+(droneX*(i)+self.drone_separation*(i))
                        rows.append(row)
                    
                    columns = []
                    y_comparison_array=np.arange(0, y_comparison, 1) # make iterable
                    
                    for i in y_comparison_array:
                        
                        column = (vertiY-self.drone_separation)-(droneY/2)-(droneY*(i)+self.drone_separation*(i))
                        columns.append(column)
                    
                
                    possiblelandingPads = [] 
                    #finding location of all possible landing pads
                    for y in y_comparison_array:
                            for x in x_comparison_array:
                                self.landingPad = [rows[x], columns[y], 0]
                                possiblelandingPads.append(self.landingPad)
                                

                    #selecting landing pads from possible ones
                    drone_max_capacity_array=np.arange(0, self.drone_max_capacity, 1) # make iterable
                    for i in drone_max_capacity_array:
                        pad= possiblelandingPads[i]
                        
                        self.landingPads.append(pad)
                    self.landingPads = np.multiply(self.landingPads,100)# convert to cm
                    print("landing pads",self.landingPads)   

        
        
        elif self.landingpad_location_generation == "Standard":
            self.landingPads = [[50, 300, 0],[150, 300, 0],[50, 200, 0],[150, 200, 0],[50, 100, 0],[150, 100, 0]]
            
        self.UAVs= [] #np.zeros(len(self.landingPads))
        
        droneN_array= np.arange(0, self.droneN, 1)
        
        for i in droneN_array:
            UAV = drone(self.landingPads[i],drone_parameters)
            self.UAVs.append(UAV) #append uavs to array

        a = self.drone_max_capacity  
        while a > len(self.UAVs) :
            self.UAVs.append([])
            a+1
        


    def generateWaypoints(self):
        if self.waypoint_generation_method == "Experimental":
            x_in = (self.landingPads[0][0]) #furthest left x value
            y_in1 = (self.landingPads[0][1])
            y_in2 = (self.landingPads[-1][1])
            
            levels = np.arange(0, 6, 1)
            in_waypoints = []

            for i in levels:
                in_waypoint = (x_in, y_in1, 400-i*50)
                in_waypoints.append(in_waypoint)
                in_waypoint = (x_in, y_in2, 400-i*50)
                in_waypoints.append(in_waypoint)
            self.waypoints_in = in_waypoints    
            

            a = self.landingPads

            x_out = max(a, key=lambda x: x[1])[1] #find furthest right x value aka max value
            y_out1 = (y_in1)
            y_out2 = (y_in2)
            out_waypoints = []

            for i in levels:
                out_waypoint = (x_out, y_out1, 150+i*50)
                out_waypoints.append(out_waypoint)
                out_waypoint = (x_out, y_out2, 150+i*50)
                out_waypoints.append(out_waypoint)
            self.waypoints_out = out_waypoints    
            




        elif self.waypoint_generation_method == "Standard":
            self.waypoints_in = [
                            [0, 300, 400],[0, 100, 400], 
                            [0, 100, 350],[0, 300, 350],
                            [0, 300, 300],[0, 100, 300],
                            [0, 100, 250],[0, 300, 250],
                            [0, 300, 200],[0, 100, 200],
                            [0, 100, 150],[0, 300, 150]
                           ]

            self.waypoints_out = [
                            [200, 100, 150],[200, 300, 150],
                            [200, 300, 200],[200, 100, 200],
                            [200, 100, 250],[200, 300, 250],
                            [200, 300, 300],[200, 100, 300],
                            [200, 100, 350],[200, 300, 350],
                            [200, 300, 400],[200, 100, 400]
                           ]
            return self.waypoints_in, self.waypoints_out

    def takeoff(self, data):
        if self.takeoff_in_progress == True:
            print("A takeoff is already in progress please wait")

            #STORE REQUEST HERE TO EXECUTE AFTER TAKEOFF IS COMPLETE

        elif self.landing_in_progess == True:
            print("A landing is in progress please wait")

            #STORE REQUEST HERE TO EXECUTE AFTER LANDING IS COMPLETE

        elif self.takeoff_in_progress == False and self.landing_in_progess == False:
            self.takeoff_in_progress = True
            if self.droneN <= 0:
                print("No more drones available from ", self)

            elif self.droneN > 0:
                #requested_drones = data["number of drones"]
                #requested_drones = int(requested_drones)
                requested_location = data["location"]
                
        
            ocupied_array = []
            occupied_index_array = []

            for i in np.arange(0, len(self.UAVs), 1):
                if not self.UAVs[i]:
                    ocupied = False
                    

                else:
                    ocupied = True
                    occupied_index = i
                    occupied_index_array.append(occupied_index)
                ocupied_array.append(ocupied)
        
        
        current_takeoff_drone = self.UAVs[occupied_index_array[0]]# select the first available drone from the landingpads
        
        print("takeoff drone ",current_takeoff_drone)

        self.UAVs[occupied_index_array[0]]= [] #remove drone from the landingpads
        

        current_takeoff_drone_location = self.landingPads[occupied_index_array[0]]
        first_waypoint = np.add(current_takeoff_drone_location,[0, 0, 50])
        self.waypoints_out.insert(0, first_waypoint)
        xy_of_requested_location = list(requested_location) 
        xy_of_requested_location[2]= 400 # set it so all requested drones fly out at the same level
        self.waypoints_out.append(xy_of_requested_location)
        self.waypoints_out.append(requested_location)
        

        print("waypoints out", self.waypoints_out)


        pos, time = current_takeoff_drone.droneControl(self.waypoints_out)

        

    def landing(self, data):
        if self.droneN>=self.drone_max_capacity:
            print("There is no more capacity in the vertiport for any more drones")

        elif self.droneN<self.drone_max_capacity:
            drone_current_location = data["location"]
            l = self.UAVs.index([])
            goal_landingPad = self.landingPads[l]#find the first empty landing pad

            print("goal landinpad",goal_landingPad)

        print("waypoints in", self.waypoints_in)
        
    

class vertiportAnimation():
    def __init__(self,vertiport,**kwargs):
        self.vertiport = vertiport
        self.internal_time_step = 0
        self.figure = plt.figure()
        self.UAVs = []
