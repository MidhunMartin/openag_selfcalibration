#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int16

import random, numpy
import sys, getopt

def callback(data):
    
    ValueWithNoise = random.gauss(data.data,float(sys.argv[2]))

    # Publish info into ROS
    rospy.loginfo(ValueWithNoise)
    pub.publish(ValueWithNoise)

def Subscriber(argv):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('NoiseGenerator', anonymous=True)

    # Topic you want to subscribe
    rospy.Subscriber(argv[0], Float32, callback)
    
def Publisher(argv):

    global pub
    pub = rospy.Publisher("%s_WithNoise" % (argv[0]), Float32, queue_size=10)
   
def Run(argv):

    # Information is obtained and processed
    Subscriber(argv)

    # Information is published
    Publisher(argv)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':    
    Run(sys.argv[1:])
