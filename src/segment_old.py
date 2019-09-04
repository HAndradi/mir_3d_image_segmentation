#! /usr/bin/env python
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from my_pcl_tutorial.msg import Clusters
from message_filters import ApproximateTimeSynchronizer, Subscriber

def clusters_cb(msg):
    clusters = np.zeros((msg.height, msg.width, 3), np.uint8)
    for cluster in msg.clusters:
        im = np.zeros((msg.height, msg.width),'uint8')
    	x_sum = y_sum = 0
        for px in cluster.cluster_pixels:
            im[px.y,px.x] = 255
            x_sum += px.x
            y_sum += px.y
        kernel = np.ones((3,3), np.uint8)
        im_opening = cv2.morphologyEx(im, cv2.MORPH_CLOSE, kernel) 
        ret, thresh = cv2.threshold(im_opening, 128, 255, cv2.THRESH_BINARY)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # create hull array for convex hull points
        #hull = [cv2.convexHull(contours[i], False) for i in range(len(contours))]
        
        conts = np.concatenate(contours)
        hull = [cv2.convexHull(conts, False)]

        red_val = int(255 * (x_sum/len(cluster.cluster_pixels)) / msg.height)
        blue_val = int(255 * (y_sum/len(cluster.cluster_pixels)) / msg.width)
        cv2.drawContours(clusters, hull, -1, (blue_val, 40, red_val), -1, 8)

        ret, mask = cv2.threshold(clusters[1], 20, 255, cv2.THRESH_BINARY)
        mask2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        #obj_img = cv2.bitwise_and(img, mask2)

    #cv2.imshow('clusters',clusters)
    #k = cv2.waitKey(30) & 0xff

    clusters_msg = bridge.cv2_to_imgmsg(clusters, encoding="bgr8")
    pub.publish(clusters_msg)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('cluster_to_image_node')
    pub = rospy.Publisher('cluster_image', Image, queue_size=1)
    rospy.Subscriber('/clusters', Clusters, clusters_cb, queue_size=1)
    rospy.spin()    
