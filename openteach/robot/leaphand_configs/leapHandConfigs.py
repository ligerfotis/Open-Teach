import numpy as np

# when the scalingfactors are coosen with 1, the additionial added offsets are 0 (f.e. np.pi and not np.pi-0.8)

# joints 0 to 3 are the joints from the cnuckle to the fingertip - joint 4 is the rotational joint
finger_offsets = np.array([[np.pi/2, np.pi, np.pi, np.pi/2-0.1],        # indexFinger
                          [np.pi/2, np.pi, np.pi, np.pi*3/2],         # middleFinger
                          [np.pi/2,np.pi,np.pi, np.pi*3/2+0.1],            # ringFinger
                          [np.pi+1.0, np.pi*3/4-0.3, np.pi-1.9, np.pi*2]])         # thumb  


# the boundaries are not set correct jet
upperBounds = np.array([[np.inf, np.inf, np.inf, np.inf],               # indexFinger
                        [np.inf, np.inf, np.inf, np.inf],               # middleFinger
                        [np.inf, np.inf, np.inf, np.inf],               # ringFinger
                        [np.pi*(3/2)+0.2, np.inf, np.inf, np.pi]])              # thumb

lowerBounds = np.array([[-np.inf, -np.inf, -np.inf, -np.inf],           # indexFinger -0.95
                        [-np.inf, -np.inf, -np.inf, -np.inf],           # middleFinger
                        [-np.inf, -np.inf, -np.inf, -np.inf],           # ringFinger    
                        [np.pi*(3/2)+0.2, -np.inf, -np.inf, np.pi]])          # thumb
# the boundaries are not set correct jet - ENDE

thumbJoint3ScalingFactor = [1, 1.5, 2, 1]


'''
# for moving all fingers
# joints 0 to 3 are the joints from the cnuckle to the fingertip - joint 4 is the rotational joint
finger_offsets = np.array([[np.pi/2, np.pi, np.pi, np.pi/2],        # indexFinger
                          [np.pi/2, np.pi, np.pi, np.pi*3/2],         # middleFinger
                          [np.pi/2,np.pi,np.pi, np.pi*3/2],            # ringFinger
                          [np.pi+1.0, np.pi*3/4-0.5, np.pi-1.9, np.pi*2-0.3]])         # thumb  


# the boundaries are not set correct jet
upperBounds = np.array([[-1.57, -0.5, -0.9, -2.25],               # indexFinger
                        [-1.57, -0.7, -0.9, 0.835],               # middleFinger
                        [-1.57,-0.7,-0.9, 0.9],              # ringFinger
                        [-0.6, -0.5, -0.2, 2.8]])              # thumb

lowerBounds = np.array([[-1.57, -0.5, -0.9, -2.25],           # indexFinger -0.95
                        [-1.57, -0.7, -0.9, 0.835],           # middleFinger
                        [-1.57,-0.7,-0.9, 0.9],           # ringFinger    
                        [-0.6, -0.5, -0.2, 2.8]])          # thumb
# the boundaries are not set correct jet - ENDE

thumbJoint3ScalingFactor = [1, 1.5, 2, 1]
'''