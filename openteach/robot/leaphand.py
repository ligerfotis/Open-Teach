import numpy as np
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation as R



#from openteach.ros_links.leaphand_control import DexArmControl
from openteach.ros_links.leaphand_control import DexArmControl 
from openteach.franka_link.otFrankaLink import OTFrankaLink
from openteach.constants import *
from openteach.utils.files import get_yaml_data, get_path_in_package
from openteach.robot.robot import RobotWrapper



#import sys
#sys.path.append('/home/clemens/Schreibtisch/masterThesis/Open-Teach/openteach/robot')
#print(sys.path)
#from leaphand_configs import *
#from leaphand_configs import finger_offsets, thumbJoint3ScalingFactor
from openteach.robot.leaphand_configs.leapHandConfigs import finger_offsets, thumbJoint3ScalingFactor, upperBounds, lowerBounds
#from openteach.robot.leaphand_configs.leapHandConfigs import *
#from openteach.robot.leaphand_configs import finger_offsets



class LeapHand(RobotWrapper):
    def __init__(self, **kwargs):
        self._controller = DexArmControl(robot_type='leaphand')

        # die Callback-Struktur gehört vermutlich noch rausgelöscht
        #self._controller.currentJointStateSubscriber_callback = None

        # For robot configurations
        #self._joint_limit_config = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))['links_info']

        # gibt an, in welcher Frequenz die Daten aufgenommen werden sollen.
        self._data_frequency = 60

        print("LeapHand-RobotWrapper-Test Output")
        #LeapHandOperator

    @property
    def name(self):
        return 'leaphand'

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_measuredLeapHandJointState, 
            'commanded_joint_states': self.get_commandedLeapHandJointState,
            'frankaArm_measuredStates': self.get_measured_frankaArm_state,
            'frankaArm_commandedStates': self.get_commanded_frankaArm_state
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions

    #def initGetCurrentJointStateCallback(self, callbackFunc):
    #    self._controller.currentJointStateSubscriber_callback = callbackFunc


    def get_measuredLeapHandJointState(self):
        return self._controller.get_measuredLeapHandState()

    def get_commandedLeapHandJointState(self):
        return self._controller.get_commandedLeapHandState()

    def get_measured_frankaArm_state(self):
        return self._controller.get_measuredFrankaArmState()
    
    def get_commanded_frankaArm_state(self):
        return self._controller.get_commandedFrankaArmState()

    def get_joint_state(self):
        pass

    # die Funktion wird nicht benötigt, muss aber drinnen sein, da sie (vermutlich) wegen der Vererbung benötigt wird
    def get_joint_position(self):
        return self._controller.get_hand_position()
    '''
    def get_joint_velocity(self):
        return self._controller.get_hand_velocity()

    def get_joint_torque(self):
        return self._controller.get_hand_torque()

    def get_commanded_joint_position(self):
        return self._controller.get_commanded_hand_joint_position()
    '''

    # Getting random position initializations for the fingers
    '''
    def _get_finger_limits(self, finger_type):
        finger_min = np.array(self._joint_limit_config[finger_type]['joint_min'])
        finger_max = np.array(self._joint_limit_config[finger_type]['joint_max'])
        return finger_min, finger_max
    
        
    def _get_thumb_random_angles(self):
        thumb_low_limit, thumb_high_limit = self._get_finger_limits('thumb')

        random_angles = np.zeros((ALLEGRO_JOINTS_PER_FINGER))
        for idx in range(ALLEGRO_JOINTS_PER_FINGER - 1): # ignoring the base
            random_angles[idx + 1] = 0.5 * (thumb_low_limit[idx + 1] + (np.random.rand() * (thumb_high_limit[idx + 1] - thumb_low_limit[idx + 1])))

        return random_angles
    

    def _get_finger_random_angles(self, finger_type):
        if finger_type == 'thumb':
            return self._get_thumb_random_angles()

        finger_low_limit, finger_high_limit = self._get_finger_limits(finger_type)

        random_angles = np.zeros((ALLEGRO_JOINTS_PER_FINGER))
        for idx in range(ALLEGRO_JOINTS_PER_FINGER - 1): # ignoring the base
            random_angles[idx + 1] = 0.8 * (finger_low_limit[idx + 1] + (np.random.rand() * (finger_high_limit[idx + 1] - finger_low_limit[idx + 1])))

        random_angles[0] = -0.1 + (np.random.rand() * 0.2) # Base angle
        return random_angles
    
        
    def set_random_position(self):
        random_angles = []
        for finger_type in ['index', 'middle', 'ring', 'thumb']:
            random_angles.append(self._get_finger_random_angles(finger_type))

        target_angles = np.hstack(random_angles)
        self.move(target_angles)
    '''
        
    # Kinematics functions
    def get_fingertip_coords(self, joint_positions):
        return self._kdl_solver.get_fingertip_coords(joint_positions)

    def _get_joint_state_from_coord(self, index_tip_coord, middle_tip_coord, ring_tip_coord, thumb_tip_coord):
        return self._kdl_solver.get_joint_state_from_coord(
            index_tip_coord, 
            middle_tip_coord, 
            ring_tip_coord, 
            thumb_tip_coord,
            seed = self.get_joint_position()
        )


    # incorporating the offset of the leapHandJoints
    def addOffsets(self, angles):

        # index finger
        angles[0] = angles[0]+finger_offsets[0][0]
        angles[1] = angles[1]+finger_offsets[0][1]
        angles[2] = angles[2]+finger_offsets[0][2]
        angles[3] = angles[3]+finger_offsets[0][3]  

        # middle finger
        angles[4] = angles[4]+finger_offsets[1][0]
        angles[5] = angles[5]+finger_offsets[1][1]
        angles[6] = angles[6]+finger_offsets[1][2]
        angles[7] = angles[7]+finger_offsets[1][3]

        # ring finger
        angles[8] = angles[8]+finger_offsets[2][0]
        angles[9] = angles[9]+finger_offsets[2][1]
        angles[10] = angles[10]+finger_offsets[2][2]
        angles[11] = angles[11]+finger_offsets[2][3]

        # thumb finger
        angles[12] = angles[12]+finger_offsets[3][0]
        angles[13] = angles[13]+finger_offsets[3][1]
        angles[14] = angles[14]+finger_offsets[3][2]
        angles[15] = angles[15]+finger_offsets[3][3]

        return angles

    # incorporating the scaling of the leapHandJoints
    def scaleJoints(self, angles):

        angles[12] = angles[12] * thumbJoint3ScalingFactor[0]
        angles[13] = angles[13] * thumbJoint3ScalingFactor[1]
        angles[14] = angles[14] * thumbJoint3ScalingFactor[2]
        angles[15] = angles[15] * thumbJoint3ScalingFactor[3]

        return angles

    # incorporating the angleBoundaries of the leapHandJoints
    def checkBoundaries(self, angles):
        
        indexId = 0
        middleId = 1
        ringId = 2
        thumbId = 3

        # indexfinger
        if angles[0] > upperBounds[indexId][0]:
            angles[0] = upperBounds[indexId][0]
            print("indexfinger - OT joint 0 - upper bound reached ###################") 
        if angles[0] < lowerBounds[indexId][0]:
            angles[0] = lowerBounds[indexId][0]
            print("indexfinger - OT joint 0 - lower bound reached ###################") 

        if angles[1] > upperBounds[indexId][1]:
            angles[1] = upperBounds[indexId][1]
            print("indexfinger - OT joint 1 - upper bound reached ###################") 
        if angles[1] < lowerBounds[indexId][1]:
            angles[1] = upperBounds[indexId][1]
            print("indexfinger - OT joint 1 - lower bound reached ###################") 

        if angles[2] > upperBounds[indexId][2]:
            angles[2] = upperBounds[indexId][2]
            print("indexfinger - OT joint 2 - upper bound reached ###################") 
        if angles[2] < lowerBounds[indexId][2]:
            angles[2] = upperBounds[indexId][2]
            print("indexfinger - OT joint 2 - lower bound reached ###################") 

        if angles[3] > upperBounds[indexId][3]:
            angles[3] = upperBounds[indexId][3]
            print("indexfinger - OT joint 3 - upper bound reached ###################") 
        if angles[3] < lowerBounds[indexId][3]:
            angles[3] = upperBounds[indexId][3]
            print("indexfinger - OT joint 3 - lower bound reached ###################") 

        # middlefinger
        if angles[5] > upperBounds[middleId][0]:
            angles[5] = upperBounds[middleId][0]
            print("middlefinger - OT joint 0 - upper bound reached ###################") 
        if angles[5] < lowerBounds[middleId][0]:
            angles[5] = lowerBounds[middleId][0]
            print("middlefinger - OT joint 0 - lower bound reached ###################") 

        if angles[6] > upperBounds[middleId][1]:
            angles[6] = upperBounds[middleId][1]
            print("middlefinger - OT joint 1 - upper bound reached ###################") 
        if angles[6] < lowerBounds[middleId][1]:
            angles[6] = upperBounds[middleId][1]
            print("middlefinger - OT joint 1 - lower bound reached ###################") 

        if angles[7] > upperBounds[middleId][2]:
            angles[7] = upperBounds[middleId][2]
            print("middlefinger - OT joint 2 - upper bound reached ###################") 
        if angles[7] < lowerBounds[middleId][2]:
            angles[7] = upperBounds[middleId][2]
            print("middlefinger - OT joint 2 - lower bound reached ###################") 

        if angles[8] > upperBounds[middleId][3]:
            angles[8] = upperBounds[middleId][3]
            print("middlefinger - OT joint 3 - upper bound reached ###################") 
        if angles[8] < lowerBounds[middleId][3]:
            angles[8] = upperBounds[middleId][3]
            print("middlefinger - OT joint 3 - lower bound reached ###################") 

        # ringfinger
        if angles[9] > upperBounds[ringId][0]:
            angles[9] = upperBounds[ringId][0]
            print("ringfinger - OT joint 0 - upper bound reached ###################") 
        if angles[9] < lowerBounds[ringId][0]:
            angles[9] = lowerBounds[ringId][0]
            print("ringfinger - OT joint 0 - lower bound reached ###################") 

        if angles[9] > upperBounds[ringId][1]:
            angles[9] = upperBounds[ringId][1]
            print("ringfinger - OT joint 1 - upper bound reached ###################") 
        if angles[9] < lowerBounds[ringId][1]:
            angles[9] = upperBounds[ringId][1]
            print("ringfinger - OT joint 1 - lower bound reached ###################") 

        if angles[10] > upperBounds[ringId][2]:
            angles[10] = upperBounds[ringId][2]
            print("ringfinger - OT joint 2 - upper bound reached ###################") 
        if angles[10] < lowerBounds[ringId][2]:
            angles[10] = upperBounds[ringId][2]
            print("ringfinger - OT joint 2 - lower bound reached ###################") 

        if angles[11] > upperBounds[ringId][3]:
            angles[11] = upperBounds[ringId][3]
            print("ringfinger - OT joint 3 - upper bound reached ###################") 
        if angles[11] < lowerBounds[ringId][3]:
            angles[11] = upperBounds[ringId][3]
            print("ringfinger - OT joint 3 - lower bound reached ###################") 

        # thumb
        if angles[12] > upperBounds[thumbId][0]:
            angles[12] = upperBounds[thumbId][0]
            print("thumb - OT joint 0 - upper bound reached ###################") 
        if angles[12] < lowerBounds[thumbId][0]:
            angles[12] = lowerBounds[thumbId][0]
            print("thumb - OT joint 0 - lower bound reached ###################") 

        if angles[13] > upperBounds[thumbId][1]:
            angles[13] = upperBounds[thumbId][1]
            print("thumb - OT joint 1 - upper bound reached ###################") 
        if angles[13] < lowerBounds[thumbId][1]:
            angles[13] = upperBounds[thumbId][1]
            print("thumb - OT joint 1 - lower bound reached ###################") 

        if angles[14] > upperBounds[thumbId][2]:
            angles[14] = upperBounds[thumbId][2]
            print("thumb - OT joint 2 - upper bound reached ###################") 
        if angles[14] < lowerBounds[thumbId][2]:
            angles[14] = upperBounds[thumbId][2]
            print("thumb - OT joint 2 - lower bound reached ###################") 

        if angles[15] > upperBounds[thumbId][3]:
            angles[15] = upperBounds[thumbId][3]
            print("thumb - OT joint 3 - upper bound reached ###################") 
        if angles[15] < lowerBounds[thumbId][3]:
            angles[15] = upperBounds[thumbId][3]
            print("thumb - OT joint 3 - lower bound reached ###################") 

        return angles

    def flatGrasp(self, angles):
        #print("function called")

        # Daumenwinkel 1 - erster Index - andere Finger: 3
        # indexfinger startet bei 0
        # Mittelfinger startet bei 4
        # Ringfinger startet bei 8
        # Daumen startet bei 12


        # wenn der Daumenwinkel und jener vom Zeigefinger, oder Mittelfinger kleiner sind als xxx, dann soll die Funktionalität aktiviert werden.
        if angles[13] > (np.pi*2) and angles[1] > 3.4:
            pass
        # Funktionalität:
        # wenn Winkel 0 (Daumen 3) kleiner ist, als xxx, dann 

        # ich brauch eine Funktion, welcher ich die Fingerwinkel übergeben kann, diese überprüft, ob die jeweiligen Winkel für einen Flat-Grasp außerhalb des Bereichs sind und ändert die Winkel dann


        # Summe des gestreckten Zeigefingers: 
        if (angles[0]+angles[1]+angles[2]) > (3*np.pi+np.pi/8):
            print("Der Winkel ist darüber ####################################")

            if angles[1] > np.pi and (3*np.pi - (angles[0]+angles[2])+np.pi/8) > np.pi:   # this is the case of the staight finger joint
                angles[1] = 3*np.pi - (angles[0]+angles[2])
            elif angles[1] > np.pi and (3*np.pi - (angles[0]+angles[2])+np.pi/8) < np.pi:
                angles[1] = np.pi
                angles[2] = 3*np.pi - (angles[0]+angles[1])+np.pi/8

        # Summe des gestreckten Mittelfinger: 
        if (angles[4]+angles[5]+angles[6]) > 3*np.pi:
            print("Der Winkel ist darüber ####################################")

            if angles[5] > np.pi and (3*np.pi - (angles[4]+angles[6])+np.pi/8) > np.pi:   # this is the case of the staight finger joint
                angles[5] = 3*np.pi - (angles[4]+angles[6])
            elif angles[5] > np.pi and (3*np.pi - (angles[4]+angles[6])+np.pi/8) < np.pi:
                angles[5] = np.pi
                angles[6] = 3*np.pi - (angles[4]+angles[5])+np.pi/8

        # Summe des gestreckten Ringfinger: 
        if (angles[8]+angles[9]+angles[10]) > 3*np.pi:
            print("Der Winkel ist darüber ####################################")

            if angles[9] > np.pi and (3*np.pi - (angles[8]+angles[10])+np.pi/8) > np.pi:   # this is the case of the staight finger joint
                angles[9] = 3*np.pi - (angles[8]+angles[10])
            elif angles[9] > np.pi and (3*np.pi - (angles[8]+angles[10])+np.pi/8) < np.pi:
                angles[9] = np.pi
                angles[10] = 3*np.pi - (angles[8]+angles[9])+np.pi/8

        # Thumb
        if (angles[13]+angles[14]) > (2*np.pi+np.pi/8):
            print("Der Winkel ist darüber ####################################")

            if angles[13] > np.pi and (angles[13]+angles[14]):   # this is the case of the staight finger joint
                angles[14] = (2*np.pi+np.pi/8) - angles[13]


        return angles


    # Movement functions
    def home(self):
        self._controller.home_hand()

    # Bewegung der Hand
    def move_coords(self, fingertip_coords):
        desired_angles = self._get_joint_state_from_coord(
            index_tip_coord = fingertip_coords[0:3],
            middle_tip_coord = fingertip_coords[3:6],
            ring_tip_coord = fingertip_coords[6:9],
            thumb_tip_coord = fingertip_coords[9:]
        )
        self._controller.move_hand(desired_angles)

    def move(self, angles):
        """
        This function sets the correct offsets and scaling factors related to the leapHand Joints and forewards the data to the rosLink to be sent to the leapHand
        """

        # adding the offsets to the angles
        angles = self.addOffsets(angles)

        # scaling the thumbRotation
        angles = self.scaleJoints(angles)

        # flatGrasp functionality
        angles = self.flatGrasp(angles)

        # checking if the jointBoundaries are
        angles = self.checkBoundaries(angles)
        
        #Winkel für den Batterietask
        #if angles[1] > 3.5:
        #    angles = [2, 3.78, 3.23, 3.0, 2.4, 3.7, 3.4, 3.0, 2.1, 3.8, 3.46, 3.15, 4.9, 4.1, 2.2, 3.14]

        #Winkel für den Buch-Öffnungs-Task
        '''
        angles[0] = 1.5
        angles[1] = 3.15
        angles[2] = 3.15
        angles[3] = 3.14
        angles[8] = 1.5
        angles[9] = 3.15
        angles[10] = 3.15
        angles[11] = 3.14
        angles[12] = 3.15
        angles[13] = 3.15
        angles[14] = 3.15
        angles[15] = 3.14
        '''

        # sending the commanded jointAngles via the RosLink/Controller to the leapHand
        self._controller.move_hand(angles)


    def getCoordsInFrankaCoordSystem(self, robotHandFramePosition):
        """
        This function transforms the KS of the OT-Framework into the KS of the FrankaArm
        """

        # für das globale KS
        # auf x kommt -z
        # auf y kommt -x
        # auf z kommt y


        handPosInfrankaCoordSystem = np.empty((4,3))

        # global frame pos in new coord System
        handPosInfrankaCoordSystem[0][0] = -robotHandFramePosition[0][2]    # auf x kommt -z
        handPosInfrankaCoordSystem[0][1] = -robotHandFramePosition[0][0]    # auf y kommt -x
        handPosInfrankaCoordSystem[0][2] = robotHandFramePosition[0][1]     # auf z kommt y


        # x axis
        handPosInfrankaCoordSystem[1][0] = -robotHandFramePosition[1][2]     # auf x kommt -z 
        handPosInfrankaCoordSystem[1][1] = -robotHandFramePosition[1][0]     # auf y kommt -x
        handPosInfrankaCoordSystem[1][2] = robotHandFramePosition[1][1]      # auf z kommt y

        # y axis
        handPosInfrankaCoordSystem[2][0] = -robotHandFramePosition[2][2]     # auf x kommt -z 
        handPosInfrankaCoordSystem[2][1] = -robotHandFramePosition[2][0]     # auf y kommt -x
        handPosInfrankaCoordSystem[2][2] = robotHandFramePosition[2][1]      # auf z kommt y

        # z axis
        handPosInfrankaCoordSystem[3][0] = -robotHandFramePosition[3][2]     # auf x kommt -z 
        handPosInfrankaCoordSystem[3][1] = -robotHandFramePosition[3][0]     # auf y kommt -x
        handPosInfrankaCoordSystem[3][2] = robotHandFramePosition[3][1]      # auf z kommt y
        

        return handPosInfrankaCoordSystem

    def changeHandKSToFrankaHandKS(self, robotHandFramePosition):

        newHandKs = np.empty((4,3))

        # global position does not change
        newHandKs[0][0] = robotHandFramePosition[0][0]
        newHandKs[0][1] = robotHandFramePosition[0][1]
        newHandKs[0][2] = robotHandFramePosition[0][2]

        # new x axis gets to be the old z axis 
        newHandKs[1][0] = robotHandFramePosition[3][0]     
        newHandKs[1][1] = robotHandFramePosition[3][1]    
        newHandKs[1][2] = robotHandFramePosition[3][2]

        # new y axis gets to be the old -x axis 
        newHandKs[2][0] = robotHandFramePosition[1][0]      
        newHandKs[2][1] = robotHandFramePosition[1][1]     
        newHandKs[2][2] = robotHandFramePosition[1][2]     

        # new x axis gets to be the old -y axis
        newHandKs[3][0] = robotHandFramePosition[2][0]  
        newHandKs[3][1] = robotHandFramePosition[2][1] 
        newHandKs[3][2] = robotHandFramePosition[2][2] 

        return newHandKs

    # die Funktion wurde vollständig von ChatGPT geschrieben
    def axes_to_quaternion(self, x_axis, y_axis, z_axis):
        """
        Convert the orientation defined by the x, y, and z axes into a quaternion.
        
        Args:
            x_axis (list or np.ndarray): The x-axis of the frame.
            y_axis (list or np.ndarray): The y-axis of the frame.
            z_axis (list or np.ndarray): The z-axis of the frame.
            
        Returns:
            np.ndarray: Quaternion [x, y, z, w].
        """
        # Ensure the axes are normalized
        x_axis = np.array(x_axis) / np.linalg.norm(x_axis)
        y_axis = np.array(y_axis) / np.linalg.norm(y_axis)
        z_axis = np.array(z_axis) / np.linalg.norm(z_axis)

        # Create a 3x3 rotation matrix using the axes
        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
        
        # Convert the rotation matrix to a quaternion using scipy
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # Returns [x, y, z, w]
        
        return quaternion

    def moveArm(self, handFramePosition):

        # adapting the coordinateSystem to the coordinateSystem of the FrankaArm
        #print("handFramePosition: ", handFramePosition)
        robotHandFramePosition = self.getCoordsInFrankaCoordSystem(handFramePosition)
        #print("robotHandFramePosition: ", robotHandFramePosition)
        newHandKsPosition = self.changeHandKSToFrankaHandKS(robotHandFramePosition)
        #print("newHandKsPosition: ", newHandKsPosition)

        # die y und z-Achse der Hand so legen, dass, wenn die Handfläche nach unten schaut, die HandKS-y Achse nach rechts schaut - also in globale negative y-Richtung
        # und die y-Achse nach unten schaut - also in globale negative z-Richtung
        #y-Achse
        newHandKsPosition[2][0] = -newHandKsPosition[2][0]
        newHandKsPosition[2][1] = -newHandKsPosition[2][1]
        newHandKsPosition[2][2] = -newHandKsPosition[2][2]
        # z-Achse
        newHandKsPosition[3][0] = -newHandKsPosition[3][0]
        newHandKsPosition[3][1] = -newHandKsPosition[3][1]
        newHandKsPosition[3][2] = -newHandKsPosition[3][2]
        #print("finalHandKsPosition: ", newHandKsPosition)

        # transforming the rotation display of the hand into a quaternion 
        newHandKsPositionQuaternion = np.zeros(7)
        newHandKsPositionQuaternion[0] = newHandKsPosition[0][0]
        newHandKsPositionQuaternion[1] = newHandKsPosition[0][1]
        newHandKsPositionQuaternion[2] = newHandKsPosition[0][2]        
        newHandKsPositionQuaternion[3:] = self.axes_to_quaternion(newHandKsPosition[1], newHandKsPosition[2], newHandKsPosition[3])
        print("newHandKsPositionQuaternion: ", newHandKsPositionQuaternion)
        # sending the handFramePosition via ROS
        self._controller.move_arm(newHandKsPositionQuaternion)
        #self.frankaController.sendHandPosToFrankaInterface(newHandKsPosition)

        