import numpy as np
import csv
import math




class drone():

    all_drones = []

    def __init__(self, start_position, drone_parameters):
        self.position=start_position
        self.velocity = np.zeros(3)
        self.t=0
        self.dt = drone_parameters["dt"]
        self.pos_array= []
        self.times_array = []
        self.speed_control = drone_parameters["speed_control"]
        self.dimentions = drone_parameters["dimentions"]

        drone.all_drones.append(self)

    def droneControl(self, waypoints):
        self.waypoints = waypoints

        for i in self.waypoints:

            # goal coordinates
            g_x=i[0]
            g_y=i[1]
            g_z=i[2]

            # current coordinates
            c_x=self.position[0]
            c_y=self.position[1]
            c_z=self.position[2]
            
            waypoint_reached = False
            
            if math.isclose(c_x, g_x, abs_tol = 0.5) and math.isclose(c_y, g_y, abs_tol = 0.5) and math.isclose(c_z, g_z, abs_tol = 0.5):
                print("Waypoint [", g_x, g_y, g_z, "] reached")
                waypoint_reached= True
                
            else:
                waypoint_reached = False


            while waypoint_reached == False:
                c_x= self.position[0]
                c_y= self.position[1]
                c_z= self.position[2]
                
                if math.isclose(c_x, g_x, abs_tol = 0.5) and math.isclose(c_y, g_y, abs_tol = 0.5) and math.isclose(c_z, g_z, abs_tol = 0.5):
                    waypoint_reached= True
                else:
                    dx = g_x-c_x
                    dy = g_y-c_y
                    dz = g_z-c_z
                    magnitude= np.sqrt(dx**2+dy**2+dz**2)

                    self.velocity = [dx, dy, dz]
                    
                    self.rescaled_velocity = self.velocity/(magnitude*self.speed_control)
                    
                    last_position = self.position
                    self.position = self.position+self.rescaled_velocity*self.dt
                    
                    self.t+=self.dt
                    

                    self.pos_array.append(self.position)
                    
                    self.times_array.append(self.t)

            


        return self.pos_array, self.times_array