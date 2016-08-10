#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import random, numpy
import sys, getopt

def IncrementNoise(Noise):
    Noise += Step
    return Noise
   
def ResetNoise(Noise):
    if(numpy.isclose(Noise, NoiseMaxValue, rtol=1e-05, atol=1e-08, equal_nan=False)):
        Noise = 0
    return Noise

def ProcessNoise():
    global Noise
    Noise = IncrementNoise(Noise)
    Noise = ResetNoise(Noise)
    return Noise

def NoiseCalculator():            
    return ProcessNoise()

def callback(data):
    
    ValueWithNoise = random.gauss(data.data,NoiseCalculator())
    
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
    pub = rospy.Publisher("%s_WithIncrementalNoise" % (argv[0]), Float32, queue_size=10)
   
def Run(argv):

    # Information is obtained and processed
    Subscriber(argv)

    # Information is published
    Publisher(argv)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':

    Noise = 0
    Step = float(sys.argv[2])
    NoiseMaxValue = float(sys.argv[3])
    
    Run(sys.argv[1:])
