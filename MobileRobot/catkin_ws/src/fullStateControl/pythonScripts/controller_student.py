#!/usr/bin/env python
import rospy#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

def main():
    # first define system parameters
    global J1, J2, Ks
    J1 = 0.0628 # kg m^2
    J2 = 0.832987 # kg m^2
    Ks = 106.5474 # Nm/rad
    
    global dt
    dt = .008 # unit is seconds, also needs to be set in the arduino code?
    
    # define the state space matrices
    global A, B, C, D, K, x, y
    A = np.matrix([[0, 0, 1, 0], [0, 0, 0, 1], [-Ks/J1, (Ks)/(J1), 0, 0], [Ks/J2, -(Ks)/(J2), 0, 0]])
    B = np.matrix([[0],[0],[1/J1],[0]])
    C = np.matrix([[0, 1, 0, 0]])
    D = np.matrix([0])
    K = np.matrix([[2.3291,29.2937, 2.1583, 27.9045]])

    x = np.matrix([[0],[0],[0],[0]]) # theta_1, theta_2, theta_1_dot, theta_2_dot
    y = np.matrix([0])

    # add state for integrator
    #global z
    #z = np.matrix([0])
    
    # define r, reference point (in degrees)
    global r, rIsZero
    r = 0
    rIsZero = 0

    # find nBar, the precompensator on the trajectory
    global nBar
    nBar = calcnBar()
    
    #### define some ros stuff
    # initialize node
    rospy.init_node('controller', anonymous = True)

    # initialize controlEffort publisher
    global uPub
    uPub = rospy.Publisher('uPub', Float32, queue_size = 10)

    # initalize encoder subscriber
    global encSub
    encSub = rospy.Subscriber('encPub', Float32MultiArray, callback)
    
    # spin ros
    rospy.spin()
  
def callback(encData):
    #### now lets do that algorithm
    # 1) any changes to the reference position
    global r
    
    # 2) read in from encoders and assign to x
    # because we are sensing every state, our y vector is identical to our state vector x. thus we   can read these values into an x vector.
    global x

    # motor position
    x[0,0] = encData.data[0]
    
    # arm position
    x[1,0] = encData.data[1]
    
    # motor velocity
    x[2,0] = encData.data[2]
    
    # arm velocity
    x[3,0] = encData.data[3]

    # error for integrator
    #global z
    #z = z + C*x - r # this should produce arm position minus reference
    # windup control
    #if z > 100000:
	#z = 100000
    #if z < -100000:
        #z = -100000
    
    # 3) determine if hard stop is required....digital stops
    # this is done in the arduino code. 
            
    # 4) calculate the controller output u (scalar) u = nBar*r - K*x #- ki*(z)
    #ki = -.1
    u = nBar*r - K*x #- ki*(z)
    
    # 5) send u to the motor
    uPub.publish(float(u))     
    
    # 6) update the state estimate x_t+1 = x_t + ((A - BK)x + BNr)*dt
    # this is not being used because we are directly sensing all of our states. 
    # x = x + ((A-B*K)*x + B*nBar*r)*dt

def calcnBar():
    # calculate nBar, precompensator on the trajectory
    global A, B, C, D, K
    nBar = -1/(C*np.linalg.inv(A-B*K)*B) # finally have precompensator!    
    return nBar

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

