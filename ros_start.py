#!/usr/bin/env python3

# this file will serve as an entryway for ROS
# so that you can run the CLI without ROS
# for development

# required so ROS will know where the python modules are
# import sys
# sys.path.append("/home/ubuntu/ToF_ws/src/ToF_ROS_CLI/ToF_ROS_CLI")

# maybe should think about removing some of these imports
import os
from GCC_API import *
import argparse

# ROS specific imports
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# fun mode
import cv2
import time

# xml parsing stuff
import glob
import xml.etree.ElementTree as ET

python_script_directory = os.path.dirname(os.path.abspath(__file__)) + '/'

## ROS Things
def ros_preloop():
    rospy.init_node('publisher', anonymous=True, disable_signals=True)

def ros_postloop():
    rospy.signal_shutdown('CLI is done')

def crab():
    crab_location = python_script_directory + 'crabs.mp4'
    crabs = cv2.VideoCapture(crab_location);
    crab_publisher = rospy.Publisher('crab', Image, queue_size=10)
    camera_info_publisher = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    camera_info = get_camera_info()
    crab_bridge = CvBridge()
    print("CRAB CRAB CRAB!")
    try:
        while True:
            crab_ret, crab_frame = crabs.read()

            if not crab_ret:
                crabs.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            crab_msg = crab_bridge.cv2_to_imgmsg(crab_frame)
            crab_publisher.publish(crab_msg)
            camera_info_publisher.publish(camera_info)
            time.sleep(1/60)

    except KeyboardInterrupt:
        crabs.release()
        print("")
        print("Crab stream done :(")
        print("")

def xml_findall_to_item_list(element_list):
    item_list = []
    for element in element_list:
        item_list.append(float(element.text))
    return item_list

def get_camera_info():
    # grab an xml file
    camera_info_files = glob.glob(python_script_directory + 'CameraInfo/*.xml')
    if len(camera_info_files) == 0:
        print("Please put your calibration data in the CameraInfo directory in the scripts folder")
        return None
    xml_file_path = camera_info_files[0]
    tree = ET.parse(xml_file_path)
    camera_info = CameraInfo()
    camera_info.height = int(tree.find('Height').text)
    camera_info.width = int(tree.find('Width').text)
    camera_info.distortion_model = tree.find('DistortionModel').text
    camera_info.D = xml_findall_to_item_list(tree.findall('D'))
    camera_info.K = xml_findall_to_item_list(tree.findall('K'))
    camera_info.R = xml_findall_to_item_list(tree.findall('R'))
    camera_info.P = xml_findall_to_item_list(tree.findall('P'))
    return camera_info

def stream_amp_and_dist_over_ROS():
    "Streams amplify image to 'video_frames'"
    from GCC_API import get_image # this is needed cause python is stupid for some reason
    amp_publisher = rospy.Publisher('amp_frames', Image, queue_size=10)
    dist_publisher = rospy.Publisher('dist_frames', Image, queue_size=10)
    camera_info_publisher = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    camera_info = get_camera_info()
    bridge = CvBridge()
    try:
        while not rospy.is_shutdown():
            img_amp_data, img_dist_data = get_image('nothing_really_matters', False)

            normalized_amp = np.array(255 * img_amp_data / np.max(img_amp_data), dtype = np.uint8)
            normalized_dist = np.array(255 * img_dist_data / np.max(img_dist_data), dtype = np.uint8)

            # normalized = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
            # ^ uncomment for more fun mode

            amp_img = cv2.cvtColor(normalized_amp, cv2.COLOR_GRAY2BGR)
            dist_img = cv2.cvtColor(normalized_dist, cv2.COLOR_GRAY2BGR)
            inverted_dist_img = np.invert(dist_img)

            amp_msg = bridge.cv2_to_imgmsg(normalized_amp)
            dist_msg = bridge.cv2_to_imgmsg(inverted_dist_img)

            amp_publisher.publish(amp_msg)
            dist_publisher.publish(dist_msg)
            camera_info_publisher.publish(camera_info)

    except KeyboardInterrupt:
        print("ROS stream finished")


if __name__ == "__main__":
    GCC_Commands().cmdloop()
