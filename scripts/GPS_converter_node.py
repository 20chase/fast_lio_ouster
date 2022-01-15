#! /usr/bin/python
import time
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


class GPSReceivor():
    def __init__(self, viz_enable=False):
        
        rospy.init_node("gnss_node")
        
        # visualization
        self.viz_enable = viz_enable
        self.confidence_sigma = 2
        if self.viz_enable:
            self.init_visualization()
        
        # earth parameters 
        self.a = 6378137.0 # Semi-major axis
        self.b = 6356752.3142 # Semi-minor axis
        self.f = (self.a - self.b) / self.a # inverse of reciprocal of flattening
        self.e_square = self.f * (2 - self.f) # First eccentricity squared
        
        # initial geodetic coordinates
        self.ref_geo_pos = NavSatFix()
        self.ref_ecef = np.zeros((3,))
        self.set_initial_position = False
        
        # gnss subscriber and publisher
        self._gnss_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self._gnss_cb)
        self._gnss_reset_srv = rospy.Service('/gnss_reset_srv', Empty, self._reinit_origin, buff_size=10)
        
        # result list
        self.path = list()
        self.ellipse_patch_list = list()

        # world to odom pose
        self.t_w2o = (0, 0, 0)
        
        time.sleep(1)
    
    def run(self):
        loop = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.viz_enable:
                self.draw_interactive()
            loop.sleep()
        
    def wgs84_to_ecef(self, geo_pos):
        """
            converts coordinates P1 from WGS84 to ECEF coordinates
        """
        
        lon = geo_pos.longitude * np.pi /180.0
        lat = geo_pos.latitude * np.pi /180.0
        alt = geo_pos.altitude

        N = self.a/np.sqrt(1 - self.e_square*np.sin(lat)*np.sin(lat))
        x = (N + alt)*np.cos(lat)*np.cos(lon)
        y = (N + alt)*np.cos(lat)*np.sin(lon)
        z = ((1-self.e_square)*N + alt)*np.sin(lat) 

        return np.array([x, y, z])
    
    def get_enu_position(self, geo_pos):
        """
            converts the target ECEF points in the ENU reference
            reference: https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
        """
        
        delta_ecef = self.wgs84_to_ecef(geo_pos) - self.ref_ecef
        enu = np.matmul(self.ref_R, delta_ecef)
        
        return enu
    
    def _gnss_cb(self, msg):
        """
            callback function of GNSS subscriber
        """
        if not self.set_initial_position:
            self.ref_geo_pos = msg
            self.ref_ecef = self.wgs84_to_ecef(self.ref_geo_pos)
            
            ref_lon = self.ref_geo_pos.longitude * np.pi/180.0
            ref_lat = self.ref_geo_pos.latitude * np.pi/180.0
        
            self.ref_R = np.array([[-np.sin(ref_lon), np.cos(ref_lon), 0], 
                      [-np.sin(ref_lat)*np.cos(ref_lon), -np.sin(ref_lat)*np.sin(ref_lon), np.cos(ref_lat)], 
                      [np.cos(ref_lat)*np.cos(ref_lon), np.cos(ref_lat)*np.sin(ref_lon), np.sin(ref_lat)]])
            
            self.set_initial_position = True
            rospy.loginfo('Set origin successfully!')
        
        current_enu = self.get_enu_position(msg)
        current_cov = np.resize(np.array(msg.position_covariance), (3, 3))
        cov_xy = np.sqrt([current_cov[0,0], current_cov[1, 1]])
        
        # remove z coordinate
        self.t_w2o = (current_enu[0], current_enu[1], 0.0)
        
        if self.viz_enable:
            self.path.append([self.t_w2o[0], self.t_w2o[1]])
            # actually, results from GPS are circle
            patch = Ellipse((self.t_w2o[0], self.t_w2o[1]), 
                            width=cov_xy[0] * 2 * self.confidence_sigma, 
                            height=cov_xy[1] * 2 * self.confidence_sigma,
                            facecolor='g', alpha=0.1)
            self.ellipse_patch_list.append(patch)
        
    def _reinit_origin(self, srv):
        """
            service handler of the GNSS origin (reference)
        """
        self.set_initial_position = False
        rospy.loginfo("Reset flag to zero and waiting for new gnss data...")

        return []
    
    def draw_interactive(self):
        if len(self.ellipse_patch_list)!=0: 
            plt.ion()
            np_path_list = np.array(self.path)
            self.ax.plot(np_path_list[:, 0], np_path_list[:, 1], color='r', linewidth=1.5)
            [self.ax.add_patch(patch) for patch in self.ellipse_patch_list]
            self.ellipse_patch_list = list()
            plt.pause(0.05)
            plt.ioff()
    
    def init_visualization(self):
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = plt.gca()
        self.ax.grid(linestyle='--')
        self.ax.set_aspect(1)
        
    
if __name__ == "__main__":
    GPS_node = GPSReceivor(viz_enable=True)
    GPS_node.run()