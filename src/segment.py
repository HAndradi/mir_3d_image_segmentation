#! /usr/bin/env python
import cv2
import numpy as np
from datetime import datetime

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from my_pcl_tutorial.msg import Clusters
from message_filters import ApproximateTimeSynchronizer, Subscriber

def synchronized_msg_cb(img_msg, cluster_msg):
    global save_imgs_flag
    img = cv2.cvtColor(bridge.imgmsg_to_cv2(img_msg),cv2.COLOR_BGR2RGB)
    cluster_img = np.zeros((cluster_msg.height, cluster_msg.width, 3), np.uint8)
    for cluster in cluster_msg.clusters:
        thresh = np.zeros((cluster_msg.height, cluster_msg.width),'uint8')
    	x_sum = y_sum = 0
        for px in cluster.cluster_pixels:
            thresh[px.y,px.x] = 255
            x_sum += px.x
            y_sum += px.y
        kernel = np.ones((3,3), np.uint8)
        im_opening = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel) 
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # create hull array for convex hull points
        #hull = [cv2.convexHull(contours[i], False) for i in range(len(contours))]
        
        conts = np.concatenate(contours)
        hull = [cv2.convexHull(conts, False)]

        red_val = int(255 * (x_sum/len(cluster.cluster_pixels)) / cluster_msg.height)
        blue_val = int(255 * (y_sum/len(cluster.cluster_pixels)) / cluster_msg.width)
        cv2.drawContours(cluster_img, hull, -1, (blue_val, 40, red_val), -1, 8)

        if save_imgs_flag:
            obj_mask = np.zeros((cluster_msg.height, cluster_msg.width),'uint8') 
            cv2.drawContours(obj_mask, hull, -1, 255, -1, 8)
            resized_obj_mask = cv2.resize(obj_mask, (img.shape[1], img.shape[0]))
            resized_obj_mask_inv = 255 - cv2.resize(obj_mask, (img.shape[1], img.shape[0])) #
            obj_mask2 = cv2.cvtColor(resized_obj_mask, cv2.COLOR_GRAY2BGR)
            obj_mask2_inv = cv2.cvtColor(resized_obj_mask_inv, cv2.COLOR_GRAY2BGR) #
            white_bkgrnd = 255*np.ones((img.shape[0], img.shape[1], 3), np.uint8) #
            obj_masked_frgrnd = cv2.bitwise_and(img, obj_mask2) #
            obj_masked_bkgrnd = cv2.bitwise_and(white_bkgrnd, obj_mask2_inv) #
            obj_masked = obj_masked_frgrnd + obj_masked_bkgrnd #
            #obj_masked = cv2.bitwise_and(img, obj_mask2)
            x, y, w, h = cv2.boundingRect(hull[0])
            #obj_img = np.zeros((64, 64, 3), np.uint8) 
            obj_img = 255 * np.ones((64, 64, 3), np.uint8) 
            cropped_obj_img = obj_masked[3*y:3*(y+h),3*x:3*(x+w)]
            if cropped_obj_img.shape[0] > cropped_obj_img.shape[1]:
                resized_obj_img = cv2.resize(cropped_obj_img, (cropped_obj_img.shape[1]*64/cropped_obj_img.shape[0], 64)) 
                obj_img[:, int(32-resized_obj_img.shape[1]/2.0) : int(32+resized_obj_img.shape[1]/2.0), :] = resized_obj_img
            else:
                resized_obj_img = cv2.resize(cropped_obj_img, (64, cropped_obj_img.shape[0]*64/cropped_obj_img.shape[1])) 
                obj_img[int(32-resized_obj_img.shape[0]/2.0) : int(32+resized_obj_img.shape[0]/2.0), :, :] = resized_obj_img
            img_time_stamp = str(datetime.now())
            cv2.imwrite('/home/hk/catkin_ws_general/src/my_pcl_tutorial/src/imgs/IMG_' + img_time_stamp + '.png',obj_img)
            print 'saved image: IMG_' + img_time_stamp + '.png'

    ret, mask = cv2.threshold(cluster_img[:,:,1], 20, 255, cv2.THRESH_BINARY)
    resized_mask = cv2.resize(mask, (img.shape[1], img.shape[0]))
    mask2 = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)
    debug_img = cv2.bitwise_and(img, mask2)

    cv2.imshow('image', debug_img)
    k = cv2.waitKey(3) & 0xff
    if k == ord('s'):
        save_imgs_flag = True
    else: 
        save_imgs_flag = False
        #cv2.imwrite('opticalfb.png',frame2)

    cluster_img_ros = bridge.cv2_to_imgmsg(cluster_img, encoding="bgr8")
#    t1 = 0.000000001* img_msg.header.stamp.nsecs + img_msg.header.stamp.secs
#    t2 = 0.000000001* rospy.Time.now().nsecs + rospy.Time.now().secs 
#    cluster_out_msg.header.stamp.nsecs = 10**9*((t2 - t1) - int(t2 - t1))
    pub.publish(cluster_img_ros)

if __name__ == '__main__':
    save_imgs_flag = False
    bridge = CvBridge()
    rospy.init_node('cluster_to_image_node')
    pub = rospy.Publisher('cluster_image', Image, queue_size=1)

    img_sub = Subscriber('/camera/color/image_raw', Image)
    clusters_sub = Subscriber('/clusters', Clusters)
    ats = ApproximateTimeSynchronizer([img_sub, clusters_sub], queue_size = 5, slop=0.000001)
    ats.registerCallback(synchronized_msg_cb)

    rospy.spin()    
