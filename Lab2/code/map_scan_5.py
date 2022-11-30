import rospy
import threading
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from math import sin,cos,pi,radians,degrees
import tf.transformations as tftr

class rosbot():
    def __init__(self):
        self.lock = threading.Lock()
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        sub1 = rospy.Subscriber("/odom", Odometry, self.getAngle)
        self.rate=rospy.Rate(1) #rate
        self.scan_ranges=[]
        self.scan_save=[]
        self.cur_rot=0
      
    def scan_callback(self,msg):
        self.scan_ranges=msg.ranges
        self.scan=msg

    def laser_scan(self):
        while not rospy.is_shutdown():
            data=self.scan_ranges
            x_1,y_1=[],[] #for scatter plot
            x_2,y_2=[],[] #for line plot
            xr_1,yr_1=[],[] #for translated scatter plot
            xr_2,yr_2=[],[] #for translated line plot
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

            #For stable map plotting
            for i in range(2*len(self.scan_ranges)):#Translating all the plot points with respect to current position
                if i < 360:
                    xr1,yr1 = self.rotate_matrix(x_1[i],y_1[i],self.cur_rot)
                    xr_1.append(xr1)
                    yr_1.append(yr1)
                xr2,yr2 = self.rotate_matrix(x_2[i],y_2[i],self.cur_rot)
                xr_2.append(xr2)
                yr_2.append(yr2)

            self.map_plot(xr_1, yr_1,xr_2,yr_2) #send data for plotting

            # self.scan_save.append(data)
            # np.savetxt('/turtlebot3_ws/maps/laser_data_df.csv',np.array(self.scan_save), fmt='%s')

    def getAngle(self, msg):
        self.lock.acquire()
        cur_r = msg.pose.pose.orientation
        cur_pos1 = tftr.euler_from_quaternion((cur_r.x, cur_r.y, cur_r.z, cur_r.w))
        self.cur_rot = cur_pos1[2]
        self.lock.release()

    def map_plot(self, x_1, y_1,x_2,y_2):
        plt.clf()
        plt.title('Laser Scan')
        plt.scatter(x_1, y_1,c='r' ,s=3, zorder=2)
        plt.plot(x_2,y_2,c='g', linewidth=.1, zorder=0)
        marker, scale = self.gen_robot_marker(degrees(self.cur_rot)) #To plot orientation of robot
        markersize = 20 #Size of robot
        plt.scatter(0, 0, marker=marker, s=(markersize*scale)**2, color='k') #Plotting robot position and orientation
        plt.pause(.1)
        plt.show
    
    def gen_robot_marker(self,rot):
        #Creating marker for robot
        arr = np.array([[0, 1],[.3, .1], [-.3, .1], [.1, .3]])  # robot shape
        angle = rot / 180 * np.pi
        rot_mat = np.array([
            [np.cos(angle), np.sin(angle)],
            [-np.sin(angle), np.cos(angle)]
            ])
        arr = np.matmul(arr, rot_mat)  # rotates the arrow

        # scale
        x0 = np.amin(arr[:, 0])
        x1 = np.amax(arr[:, 0])
        y0 = np.amin(arr[:, 1])
        y1 = np.amax(arr[:, 1])
        scale = np.amax(np.abs([x0, x1, y0, y1]))
        codes = [mpl.path.Path.MOVETO, mpl.path.Path.LINETO,mpl.path.Path.LINETO, mpl.path.Path.CLOSEPOLY]
        arrow_head_marker = mpl.path.Path(arr, codes)
        return arrow_head_marker, scale

    def rotate_matrix (self, x, y, angle):

        # Shift to origin (0,0)
        # x = x - x_shift
        # y = y - y_shift

        # Rotation matrix multiplication to get rotated x & y
        xr = (x * cos(angle)) - (y * sin(angle)) #+ x_shift
        yr = (x * sin(angle)) + (y * cos(angle)) #+ y_shift

        return xr, yr

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