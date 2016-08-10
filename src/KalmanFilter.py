#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float32

import random, numpy
import sys, getopt

from collections import deque

# Implements a linear Kalman filter.
class KalmanFilterLinear:

    def __init__(self,_A, _B, _H, _x, _P, _Q, _R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.current_state_estimate = _x # Initial state estimate.
        self.current_prob_estimate = _P  # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements.
    def GetCurrentState(self):
        return self.current_state_estimate
    def Step(self,control_vector,measurement_vector):
        #---------------------------Prediction step-----------------------------
        predicted_state_estimate = self.A * self.current_state_estimate + self.B * control_vector
        predicted_prob_estimate = (self.A * self.current_prob_estimate) * numpy.transpose(self.A) + self.Q
        #--------------------------Observation step-----------------------------
        innovation = measurement_vector - self.H*predicted_state_estimate
        innovation_covariance = self.H*predicted_prob_estimate*numpy.transpose(self.H) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = predicted_prob_estimate * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
        self.current_state_estimate = predicted_state_estimate + kalman_gain * innovation
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = (numpy.eye(size)-kalman_gain*self.H)*predicted_prob_estimate


KalmanFilterIsinControl = False    
CircularBuffer = deque(maxlen=25)
    
def callback(data):

    CircularBuffer.append(data.data)

    # Input new data into the filter
    filter.Step(numpy.matrix([0]),numpy.matrix([data.data]))

    # We calculate the stdev of the buffer
    CircularBufferStandardDeviation = numpy.var(CircularBuffer)

    rospy.loginfo(CircularBufferStandardDeviation)

    global KalmanFilterIsinControl
    
    if (CircularBufferStandardDeviation > 10 | KalmanFilterIsinControl):
        
        KalmanFilterIsinControl = True

        # Publish info into ROS
        rospy.loginfo(filter.GetCurrentState()[0,0])
        pub.publish(filter.GetCurrentState()[0,0])

    else:
        # Publish info into ROS
        rospy.loginfo(data.data)
        pub.publish(data.data)

def KalmanFilterDefinition():
    
    A = numpy.matrix([1])
    H = numpy.matrix([1])
    B = numpy.matrix([0])
    Q = numpy.matrix([0.00001])
    R = numpy.matrix([0.1])
    xhat = numpy.matrix([350])
    P    = numpy.matrix([1])

    global filter
    filter = KalmanFilterLinear(A,B,H,xhat,P,Q,R)

def Subscriber(argv):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('KalmanFilter', anonymous=True)

    # KLF Instance 
    KalmanFilterDefinition()

    # Topic you want to subscribe
    rospy.Subscriber(argv[0], Float32, callback)
    
def Publisher(argv):

    global pub
    pub = rospy.Publisher("%s_kf" % (argv[0]), Float32, queue_size=10)

    
def Run(argv):

    # Information is obtained and processed
    Subscriber(argv)

    # Information is published
    Publisher(argv)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    
    if (len(sys.argv)<2):
        print ('Not enough arguments')
    else:
        Run(sys.argv[1:])
