# required so ROS will know where the python modules are
import sys
sys.path.append("/home/ubuntu/ToF_ws/src/ToF_ROS_CLI/ToF_ROS_CLI")

# maybe should think about removing some of these imports
import os
from GCC_API import *
import argparse

# this is very simple now since we are only entering into GCC_Commands

GCC_Commands().cmdloop()
