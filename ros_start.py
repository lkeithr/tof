#!/usr/bin/env python3

# this file will serve as an entryway for ROS
# so that you can run the CLI without ROS
# for development

# required so ROS will know where the python modules are
import sys
sys.path.append("/home/ubuntu/ToF_ws/src/ToF_ROS_CLI/ToF_ROS_CLI")

# maybe should think about removing some of these imports
import os
from GCC_API import *
import argparse

# ROS specific imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.signals import SignalHandlerOptions

class VideoStreamer(Node):
    def __init__(self):
        super().__init__('video_streamer')
        self.amp_publisher = self.create_publisher(Image, 'amp_frames', 10)
        self.dist_publisher = self.create_publisher(Image, 'dist_frames', 10)
        self.bridge = CvBridge()

    def publish_frames(self):
        from GCC_API import get_image # this is needed cause python is stupid for some reason
        while rclpy.ok():
            img_amp_data, img_dist_data = get_image('nothing_really_matters', False)
            normalized_amp = np.array(255 * img_amp_data / np.max(img_amp_data), dtype = np.uint8)
            normalized_dist = np.array(255 * img_dist_data / np.max(img_dist_data), dtype = np.uint8)

            # normalized = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
            # ^ uncomment for fun mode

            amp_img = cv2.cvtColor(normalized_amp, cv2.COLOR_GRAY2BGR)
            dist_img = cv2.cvtColor(normalized_dist, cv2.COLOR_GRAY2BGR)
            amp_msg = self.bridge.cv2_to_imgmsg(amp_img)
            dist_msg = self.bridge.cv2_to_imgmsg(dist_img)
            self.amp_publisher.publish(amp_msg)
            self.dist_publisher.publish(dist_msg)

    def destroy(self):
        super().destroy_node()

# these functions should be injected into the GCC_Commands class
def ROS_preloop(instance):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    instance.video_streamer = VideoStreamer()

def ROS_postloop(instance):
    instance.video_streamer.destroy()
    rclpy.shutdown()

def stream_amp_and_dist_over_ROS(instance):
    "Streams amplify image to 'video_frames'"
    try:
        instance.video_streamer.publish_frames()
    except KeyboardInterrupt:
        print("ROS stream finished")


def main(): # this needs to be main because otherwise no entry point (I think)
    GCC_Commands().cmdloop()
