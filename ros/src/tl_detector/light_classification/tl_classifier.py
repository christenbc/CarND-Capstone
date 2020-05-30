from styx_msgs.msg import TrafficLight
import rospy
import cv2
import tensorflow as tf
import numpy as np
import os
import yaml

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.session = None
        self.graph_model = None
        
        self.categories = {1: TrafficLight.GREEN,
                           2: TrafficLight.YELLOW,
                           3: TrafficLight.RED,
                           4: TrafficLight.UNKNOWN}
                           
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.get_model(os.path.dirname(os.path.realpath(__file__)) + self.config['classifier_model'])
        
    def get_model(self, path_model):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        
        self.graph_model = tf.Graph()
        
        with tf.Session(graph = self.graph_model, config = config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name = '')
                
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        image = cv2.resize(image, (300, 300))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        image_tensor = self.graph_model.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.graph_model.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.graph_model.get_tensor_by_name('detection_scores:0')
        detection_classes = self.graph_model.get_tensor_by_name('detection_classes:0')
        
        boxes, scores, classes = self.session.run([detection_boxes, detection_scores, detection_classes],
                                                  feed_dict = {image_tensor: np.expand_dims(image, axis = 0)})
                                                  
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        
        for item, box in enumerate(boxes):
            if scores[item] > 0.5:
                traffic_light_class = self.categories[classes[item]]
                rospy.loginfo("Inferred traffic light was {}".format(traffic_light_class))
                return traffic_light_class
            else:
                rospy.loginfo("Unknown detection")
        
        return TrafficLight.UNKNOWN