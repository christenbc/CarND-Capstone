#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import numpy as np
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add other member variables you need below
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.pose = None
        
        self.stopline_wp_idx = -1
        self.base_lane = None
        self.stop_wp_idx = -1

        # rospy.spin()
        self.loop() # instead, to have control of the publishing frequency
        
    def loop(self):
        """
        Instead of rospy.spin(), this method is used as an alternative in order to set
        a custom publishing rate
        """
        rate = rospy.Rate(30) # 30 Hz should be the minimum
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                # Get the closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
                
    def get_closest_waypoint_idx(self):
        """
        From the KDTree, the closest waypoint is extracted from the base waypoints given
        a pose. Such waypoint will always be ahead from the position of the car towards the
        highway direction
        """
    
        # Coordinates of our car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]
        
        # Check if the closest waypoint is ahead or behind vehicle
        closest_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        # Equation of the hyperplane through closest_coord
        val = np.dot(closest_vect - prev_vect, pos_vect - closest_vect)

        if val > 0: # if the waypoint is behind us
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d) # modulo to avoid trouble when finishing a lap
        return closest_idx
    
    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane(self):
        """
        In addition to the closest waypoint, a set of subsequent waypoints are added to the lane
        waypoints output. This method triggers as well the deceleration in case of given stop waypoints
        """
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx : farthest_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        """
        The velocity will be reduced in case the pose reaches a triggering waypoint prior
        to the stop waypoint line        
        """
        tmp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # 2 wp back from line so front of the car stops at the line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
                
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            tmp.append(p)
                    
        return tmp
                
         
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints): # Latch subscriber, it is called just once
        """
        The base waypoints are formated in a 2D list which is fed to the KDTree method to construct a KDTree which
        will provide closest waypoints information in a quite optimal way
        """
        # TODO: Implement
        self.base_lane = waypoints
        if not self.waypoints_2d: # to be sure that it is initialized before callback is performed
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints] # converted to a 2D list
            self.waypoint_tree = KDTree(self.waypoints_2d)        

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')