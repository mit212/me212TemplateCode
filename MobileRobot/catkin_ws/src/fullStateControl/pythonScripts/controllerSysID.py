#!/usr/bin/env python
import rospy#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import time

def main():
    # first define system parameters
    global J1, J2, Ks
    J1 = 0.00040984488 # kg m^2
    J2 = 0.832987 # kg m^2
    Ks = 350 # N/m
    
    global dt
    dt = .001 # unit is seconds, also needs to be set in the arduino code?
    
    # define the state space matrices
    global A, B, C, D, K, x, y
    A = np.matrix([[0, 0, 1, 0], [0, 0, 0, 1], [(Ks)/J1, (-Ks)/(4*J1), 0, 0], [Ks/J2, (-Ks)/(4*J2), 0, 0]])
    B = np.matrix([[0],[0],[1/J1],[0]])
    C = np.matrix([[0, 1, 0, 0]])
    D = np.matrix([0])
    K = np.matrix([[0, 0, 0, 0]]) #0.0484, -31.8166, 0.0033, -2.1533
    
    x = np.matrix([[0],[0],[0],[0]])
    y = np.matrix([0])

    # add state for integrator
    #global z
    #z = np.matrix([0])
    
    # define r, reference trajectory (in degrees)
    global r, inAmp, inFreq
    inAmp = 10; 
    inFreq = 0.0/1000; # in msec
    r = inAmp*np.sin(2*np.pi*inFreq*(time.time()*1000.0));
    
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

    global joySubscriber
    joySubscriber = rospy.Subscriber("joy", Joy, joyCallback)
	
    # spin ros
    rospy.spin()

def callback(encData):
    #### now lets do that algorithm
    # 1) any changes to the reference position
    global r, inAmp, inFreq
    r = inAmp*np.sin(2*np.pi*inFreq*(time.time()*1000.0));

    #if false: # going to have to figure out scope stuff here
    nBar = calcnBar()
    nBar= 1;
    
    # 2) read in from encoders and assign to x
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
    blah = time.time()*1000.0

    # 6) update the state estimate x_t+1 = x_t + ((A - BK)x + BNr)*dt
    x = x + ((A-B*K)*x + B*nBar*r)*dt

def joyCallback(data):
	# set inFreq with y value of left joystick
	global inFreq

	# look at zero with A, B (zero freq), and X
	if data.buttons[0] == 1: # A
		inFreq = .8/1000
	if data.buttons[1] == 1: # B
		inFreq = 1.8/1000
	if data.buttons[2] == 1: # X
		inFreq = 2.8/1000

	# look at pole with X, Y (pole freq), and RB
	if data.buttons[3] == 1: # Y
		inFreq = 6.8/1000
	if data.buttons[5] == 1: # RB
		inFreq = 11.0/1000

	# set to zero freq with LB
	if data.buttons[4] == 1: # Y
		inFreq = 0.0/1000
	
	# global inFreq
	# inFreq = ((data.axes[1]+1)*0.5 + 6.3)/1000#.8)/1000
    
def calcnBar():
    # calculate nBar, precompensator on the trajectory
    global A, B, C, D, K
    Z = np.matrix([[0], [0], [0], [0], [1]])
    temp1 = np.concatenate((A,B), axis = 1)
    temp2 = np.concatenate((C,D), axis = 1)
    temp3 = np.concatenate((temp1,temp2), axis = 0)
    temp4 = np.linalg.inv(temp3)
    temp5 = temp4*Z
    nX = temp5[[0,1,2,3]]
    nU = temp5[4]
    nBar = K*nX + nU # finally have precompensator!    
    return nBar

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
