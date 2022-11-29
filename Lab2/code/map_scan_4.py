import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from math import sin,cos,pi,radians
import pandas as pd

class rosbot():
    def __init__(self):
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.rate=rospy.Rate(1) #rate
        self.scan_ranges=[]
        self.scan_save=[]
      
    def scan_callback(self,msg):
        self.scan_ranges=msg.ranges
        self.scan=msg

    def laser_scan(self):
        while not rospy.is_shutdown():
            data=self.scan_ranges
            x_1,y_1=[],[] #for scatter plot
            x_2,y_2=[],[] #for line plot
            for angle in range(len(self.scan_ranges)):
                if self.scan_ranges[angle]=="inf":
                    pass
                else:
                    #for dot plot
                    x_new_1= -self.scan_ranges[angle]*sin(radians(angle)) #sin, cos will take value in radian
                    y_new_1= self.scan_ranges[angle]*cos(radians(angle)) #sin, cos will take value in radian
                    x_1.append(x_new_1)
                    y_1.append(y_new_1)
                    #for line plot
                    x_new_2= -self.scan_ranges[angle]*sin(radians(angle))
                    y_new_2= self.scan_ranges[angle]*cos(radians(angle))             
                    x_2.insert(len(x_2)-1,0) #to add origin co-ordinate
                    y_2.insert(len(y_2)-1,0) #to add origin co-ordinate
                    x_2.append(x_new_2)
                    y_2.append(y_new_2)

            self.map_plot(x_1, y_1,x_2,y_2) #send data for plotting

            self.scan_save.append(data)
            np.savetxt('/turtlebot3_ws/maps/laser_data_df.csv',np.array(self.scan_save), fmt='%s')   

    def map_plot(self, x_1, y_1,x_2,y_2):
        plt.clf()
        plt.title('Laser Scan')
        plt.scatter(x_1, y_1,c='r' ,s=3, zorder=2)
        plt.plot(x_2,y_2,c='g', linewidth=.1, zorder=0)
        plt.scatter(0,0, c='b', s=100, zorder=1)
        plt.pause(.2)
        plt.show
    
    

    def shutdownhook(self):
        self.ctrl_c=True
   
if __name__=="__main__":
   rospy.init_node('laser_scan')
   rosbot_obj=rosbot()
   rosbot_obj.laser_scan()
   # rosbot_obj.save_map()
   # try:
   #    rosbot_obj.map()
   # except rospy.ROSInterruptExecution:
   #    pass