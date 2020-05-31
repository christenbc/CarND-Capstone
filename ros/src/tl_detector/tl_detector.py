#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree        # Added the KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')  # Initialization of the tl_detector node

        self.pose = None                # pose : x, y
        self.waypoints = None           
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None        # image from car-mounted camera
        self.lights = []
        # The tl_detector node subscribes to the topics: '/current_pose', '/base_waypoints', '/vehicle/traffic_lights', '/image_color'
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Define the Publisher topic: tl_detector publishes the '/traffic_waypoint' topic which publishes the upcoming red lights at camera frequency.
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()  # light_classifier is an object of the TLClassifier class to determine the classified image 
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    # Call back function for the '/current_pose' topic subscription
    def pose_cb(self, msg):
        self.pose = msg

    # Call back function for the '/base_waypoints' topic subscription
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    # Call back function for the '/vehicle/traffic_lights' topic subscription
    def traffic_cb(self, msg):
        self.lights = msg.lights   

    # Call back function for the '/image_color' topic subscription
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights() # Get the closest traffic light index and its state

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:     # if state is changed 
            self.state_count = 0    # set count to Zero
            self.state = state      # set the Node state to the new state & count +1 at afterwards 
        elif self.state_count >= STATE_COUNT_THRESHOLD:                 # if the counts are greater than the Threshold    
            self.last_state = self.state                                # Set the last state to the Node state 
            light_wp = light_wp if state == TrafficLight.RED else -1    # set the light_wp index to the closest traffic light index ONLY if its state was Red  
            self.last_wp = light_wp                                     # set the last_wp index to the light_wp index
            self.upcoming_red_light_pub.publish(Int32(light_wp))        # ---> Publish the (light_wp)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))    # if state did not change and still less than the threshold ---> Kepp Publishing the Last_wp index
        self.state_count += 1       # Increment the state_count + 1

    def get_closest_waypoint(self, x, y): # Changed the Parameters from pose to x,y 
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # Commented the light.state: It was only used for testing with no Image Classification.
        # return light.state
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # Return the classified Image 
        return self.light_classifier.get_classification(cv_image)
        
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists) ------------------------------------------------------- process traffic lights
            # We are going to iterate through the traffic light list to find the closest one
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            #rospy.logwarn('Closes light idx: {} \t state: {}'.format(line_wp_idx, state))
            return line_wp_idx, state   # line_wp_idx = light_wp which is called in the image_cb function
            
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')