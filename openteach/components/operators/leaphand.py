import numpy as np
from copy import deepcopy as copy
import zmq
from openteach.utils.vectorops import *

#Holo-bot Components
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from .operator import Operator
from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points
#from .calibrators.allegro import OculusThumbBoundCalibrator

from openteach.robot.leaphand import LeapHand # importieren des Files, welches ... [tbc]

# from openteach.robot.allegro.allegro import AllegroHand
#from openteach.robot.allegro.allegro_retargeters import AllegroKDLControl, AllegroJointControl
from openteach.utils.files import *
from openteach.utils.vectorops import coord_in_bound
from openteach.utils.timer import FrequencyTimer
from openteach.constants import *
from openteach.components.recorders import *
from openteach.components.sensors import *
from openteach.utils.images import rotate_image, rescale_image
from collections import deque

#Isaac Gym components
'''
from isaacgym import gymapi, gymutil
import gym
from isaacgym import gymtorch
from isaacgym.torch_utils import *
'''


class LeapHandOperator(Operator):
    def __init__(
        self,
        host, 
        transformed_keypoints_port,
        jointanglepublishport):


        #self.hand_configs = get_yaml_data(get_path_in_package("robot/leaphand_configs/leaphand_info.yaml"))
        #self.finger_configs = get_yaml_data(get_path_in_package("robot/leaphand_configs/leaphand_link_info.yaml"))
        #self.bound_info = get_yaml_data(get_path_in_package("robot/leaphand_configs/leaphand_bounds.yaml"))

        #self.linear_scaling_factors = self.bound_info['linear_scaling_factors']
        #self.rotatory_scaling_factors = self.bound_info['rotatory_scaling_factors']

        self.notify_component_start('LeapHand operator')
        self._host, self._port = host, transformed_keypoints_port

        ''' 
        # Transformed Arm Keypoint Subscriber
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )
        '''
        
        # Transformed Hand Keypoint Subscriber
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_coords'
        )

        '''
        # Gripper Publisher
        self.gripper_publisher = ZMQKeypointPublisher(
            host=host,
            port=gripper_port
        )
        
        
        # Cartesian Publisher
        self.cartesian_publisher = ZMQKeypointPublisher(
            host=host,
            port=cartesian_publisher_port
        )
        
        '''
        # Joint Publisher
        self.currentJointAngle_publisher = ZMQKeypointPublisher(
            host=host,
            port=jointanglepublishport
        )
        '''

        # Cartesian Command Publisher
        self.cartesian_command_publisher = ZMQKeypointPublisher(
            host=host,
            port=cartesian_command_publisher_port
        )
        
        # Arm Resolution Subscriber
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host= host,
            port= arm_resolution_port,
            topic = 'button'
        )
        '''

        
        # Define Robot object
        #self._robot = RobotWrapper()
        self._robot = LeapHand()

        # setzen des Callbacks im Roslink, um die currentJointStates, welche von der LeapHand via Ros gepublished werden, zu erhalten
        #self._robot.initGetCurrentJointStateCallback(self.getCurrentJointStates)        
        



        #self.robot.reset()
        print("LeaphandOperator - Start")
        # Get the initial pose of the robot
        #home_pose=np.array(self.robot.get_cartesian_position())
        #self.robot_init_H = self.robot_pose_aa_to_affine(home_pose)
        self._timer = FrequencyTimer(BIMANUAL_VR_FREQ) 

        # Use the filter
        #self.use_filter = use_filter
        #if use_filter:
        #    robot_init_cart = self._homo2cart(self.robot_init_H)
        #    self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)
            
        # Class variables
        self.gripper_flag=1
        self.pause_flag=1
        self.prev_pause_flag=0
        self.is_first_frame= True
        self.gripper_cnt=0
        self.prev_gripper_flag=0
        self.pause_cnt=0
        self.gripper_correct_state=1
        self.resolution_scale =1
        self.arm_teleop_state = ARM_TELEOP_STOP


        
    
        # Adding the thumb debugging components
        self._arm_transformed_keypoint_subscriber = ZMQKeypointSubscriber(
            host = self._host,
            port = self._port,
            topic = 'transformed_hand_frame'
        )
        '''
        # Adding the Joint Angle Publisher and Subscriber
        self.joint_angle_publisher = ZMQKeypointPublisher(
            host = host,
            port = jointanglepublishport
        )
        
        self.joint_angle_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = jointanglesubscribeport,
            topic= 'current_angles'
        )
      
        # Initializing the solvers
        self.finger_configs = finger_configs
        self.fingertip_solver = AllegroKDLControl()
        self.finger_joint_solver = AllegroJointControl()


       
        # Initializing the queues
        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
            'ring': []
        }

        # Calibrating to get the thumb bounds
        self._calibrate_bounds()
        self._stream_oculus=stream_oculus
        self.stream_configs=stream_configs 

        # Getting the bounds for the allegro hand
        allegro_bounds_path = get_path_in_package('components/operators/configs/allegro.yaml')
        self.allegro_bounds = get_yaml_data(allegro_bounds_path)

        self._timer = FrequencyTimer(VR_FREQ)

        # Using 3 dimensional thumb motion or two dimensional thumb motion
        if self.finger_configs['three_dim']:
            self.thumb_angle_calculator = self._get_3d_thumb_angles
        else:
            self.thumb_angle_calculator = self._get_2d_thumb_angles
        '''
        
        '''
       

    '''    
    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot
    
    def return_real(self):
        return False

    
    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    
    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._arm_transformed_keypoint_subscriber


    # Calibrate the bounds for the thumb
    def _calibrate_bounds(self):
        self.notify_component_start('calibration')
        #calibrator = OculusThumbBoundCalibrator(self._host, self._port)
        #self.hand_thumb_bounds = calibrator.get_bounds() # Provides [thumb-index bounds, index-middle bounds, middle-ring-bounds]
        #print(f'THUMB BOUNDS IN THE OPERATOR: {self.hand_thumb_bounds}')  

    # Get Finger Coordinates
    def _get_finger_coords(self):

        raw_keypoints = self.transformed_hand_keypoint_subscriber.recv_keypoints()
        return dict(
            index = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['index']]]),
            middle = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['middle']]]),
            ring = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['ring']]]),
            thumb =  np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['thumb']]])
        )


    # Get Thumb 2D Angles
    def _get_2d_thumb_angles(self, thumb_keypoints, curr_angles):

        '''
        for idx, thumb_bounds in enumerate(self.hand_thumb_bounds):
            if coord_in_bound(thumb_bounds[:4], thumb_keypoints[:2]) > -1:
                return self.fingertip_solver.thumb_motion_2D(
                    hand_coordinates = thumb_keypoints, 
                    xy_hand_bounds = thumb_bounds[:4],
                    yz_robot_bounds = self.allegro_bounds['thumb_bounds'][idx]['projective_bounds'],
                    robot_x_val = self.allegro_bounds['x_coord'],
                    moving_avg_arr = self.moving_average_queues['thumb'], 
                    curr_angles = curr_angles
                )
        return curr_angles
        '''

    # Get Thumb 3D Angles
    def _get_3d_thumb_angles(self, thumb_keypoints, curr_angles):
        
        '''
        # We will be using polygon implementations of shapely library to test this
        planar_point = Point(thumb_keypoints)
        planar_thumb_bounds = Polygon(self.hand_thumb_bounds[:4])

        # Get the closest point from the thumb to the point
        # this will return the point if it's inside the bounds
        closest_point = nearest_points(planar_thumb_bounds, planar_point)[0]
        closest_point_coords = [closest_point.x, closest_point.y, thumb_keypoints[2]]
        return self.fingertip_solver.thumb_motion_3D(
            hand_coordinates = closest_point_coords,
            xy_hand_bounds = self.hand_thumb_bounds[:4],
            yz_robot_bounds = self.allegro_bounds['thumb_bounds'][0]['projective_bounds'], # NOTE: We assume there is only one bound now
            z_hand_bound = self.hand_thumb_bounds[4],
            x_robot_bound = self.allegro_bounds['thumb_bounds'][0]['x_bounds'],
            moving_avg_arr = self.moving_average_queues['thumb'], 
            curr_angles = curr_angles
        )
        '''
    
    # Generate Frozen Angles
    def _generate_frozen_angles(self, joint_angles, finger_type):
        '''
        for idx in range(ALLEGRO_JOINTS_PER_FINGER):
            if idx > 0:
                joint_angles[idx + ALLEGRO_JOINT_OFFSETS[finger_type]] = 0
            else:
                joint_angles[idx + ALLEGRO_JOINT_OFFSETS[finger_type]] = 0

        return joint_angles
        '''

    '''
    def getCurrentJointStates(self, msg):
        print("im Operator ###########")
        #print(msg.data[0])
        self.currentJointAngle_publisher.pub_keypoints(msg.data, 'currentJointAngles')

        #pass
    '''
        
    ## die Funktion wurde von dem AllegroHandOperator genommen und angepasst
    # die calculate_angle - Funktion steht im File openteach/utils/vectorops.py
    # die calculate_finger_angles - Funktion kommt aus dem File openteach/robot//allegro/allegro_retargeters.py
    # in dem Code ist "finger_joint_coords[idx]" ein dreidimensionaler Vekter, welcher einen Punkt im Raum beschreibt
    def calculate_finger_angles(self, finger_type, finger_joint_coords, hand_keypoints):
        """
        This function calculates the angles of a specific finger. The first three angles are translatory angles and the last angle is a rotatory angle, which is added via the function calculate_finger_rotation.
        """


        translatory_angles = []

        if finger_type != "thumb":
            for idx in range(0,3): # inklusive linker Ziffer, ohne rechter Ziffer
                angle = calculate_angle(
                    finger_joint_coords[idx],
                    finger_joint_coords[idx + 1],
                    finger_joint_coords[idx + 2]
                )
                translatory_angles.append(angle)

        # beim Daumen ist das Ganze um einen Knoten nach hinten verschoben, da bei diesem ein zusätzlicher "starrer" Knoten im Handgelenk vorhanden ist (und somit insgesamt auch ein Knoten mehr
        # vorhanden ist)
        if finger_type == "thumb":
            for idx in range(1,4): # Ignoring the rotatory joint
                angle = calculate_angle(
                    finger_joint_coords[idx],
                    finger_joint_coords[idx + 1],
                    finger_joint_coords[idx + 2]
                )
                #print("angle: ", angle)
                translatory_angles.append(angle)
            
        rotatory_angle = [self.calculate_finger_rotation(hand_keypoints, finger_type)] 
        translatory_angles.append(rotatory_angle[0])

        return translatory_angles
    
    
    # erste Dimension ist vermutlich entlang der Breite der Handfläche
    # die dritte Dimension geht vermutlich aus der Handfläche hinaus - es wirkt so, als ob die Skalierungen der Dimensionen unterschiedlich wäre
    # die calculate_finger_rotation - Funktion kommt aus dem File openteach/robot/allegro/allegro_retargeters.py
    def calculate_finger_rotation(self, hand_keypoints, finger_type):
    
        angle = -1
        if finger_type == "index":
            angle = calculate_angle(hand_keypoints['middle'][1][:], hand_keypoints['index'][1][:], hand_keypoints['index'][2][:])
            #print("angle-index: ", angle)
            

        if finger_type == "middle":
            angle = calculate_angle(hand_keypoints['index'][1][:], hand_keypoints['middle'][1][:], hand_keypoints['middle'][2][:])
            angle = angle * (-1)    # for the correct rotationdirection
            #print("angle-middle: ", angle)

        if finger_type == "ring":
            angle = calculate_angle(hand_keypoints['middle'][1][:], hand_keypoints['ring'][1][:], hand_keypoints['ring'][2][:])
            angle = angle * (-1)    # for the correct rotationdirection
            #print("angle-ring: ", angle)

        if finger_type == "thumb":

            # die Vorgehensweise der Berechnung basiert auf ChatGPT
            # die Knöchelknoten des Zeigefingers und des Mittelfinger spannen mit dem Knotenvektor [0-1] des Zeigefingers eine Ebene auf
            # zu dieser Ebene wird der Normalvektor berechnet, sowie anschließend die Projektion des Daumens zu diesem Normalvektor
            
            thumbVect = hand_keypoints['thumb'][3][:]
            
            # Berechnen des Normalvektors
            # Kreuzprodukt der Vektoren berechnen
            vectKnuckleInd_orig = np.array(hand_keypoints['index'][1][:])
            vectKnuckleInd_Mid = np.array(hand_keypoints['index'][1][:]) - np.array(hand_keypoints['middle'][1][:])
            normalVect = np.cross(vectKnuckleInd_orig,vectKnuckleInd_Mid) 

            # die Projektion des Daumens auf den Normalvektor der Ebene berechnen
            projectionThumbNorm = (np.dot(thumbVect, normalVect) / np.dot(normalVect, normalVect)) * normalVect

            # die Projektion des Daumens in die Ebene berechnen
            projectionThumbPlane = thumbVect - projectionThumbNorm

            # Winkel zwischen dem Daumen und der Ebene berechnen
            angle = calculate_angle(projectionThumbPlane, [0,0,0], thumbVect)
            angle = angle * (-1)    # for the correct rotationdirection
            #print("angle-thumb: ", angle)

        return angle


    # Apply Retargeted Angles
    def _apply_retargeted_angles(self):
        armKeyPoints = self.transformed_arm_keypoint_subscriber.recv_keypoints(flags = 1)
        if armKeyPoints is not None:
            print("armKeyPoints: ", armKeyPoints)
            # switching the z-axis, in order to get a right handed coordinate system
            armKeyPoints[0][2] = armKeyPoints[0][2] * -1
            armKeyPoints[1][2] = armKeyPoints[1][2] * -1
            armKeyPoints[2][2] = armKeyPoints[2][2] * -1
            armKeyPoints[3][2] = armKeyPoints[3][2] * -1
            # when the hand is in front of you with the thumb looking to the right side, the palm to the top and the fingers to the front, the axis are now layed in the hand as:
            # x pointing to the right, y pointing to the top and z pointing towards you

            self._robot.moveArm(armKeyPoints)

        # die Funktion wird in einer Schleife aufgerufen

        hand_keypoints = self._get_finger_coords()
        #print("hand_keypoints: ", hand_keypoints)
        
        # Movement for the index finger 
        desired_joint_angles = self.calculate_finger_angles(
            finger_type = 'index',
            finger_joint_coords = hand_keypoints['index'],
            hand_keypoints = hand_keypoints     # für die Berechnung der Rotation werden zusätzlich alle Keypoints weitergegeben
            #curr_angles = desired_joint_angles,
            #moving_avg_arr = self.moving_average_queues['index']
        )
        #print("desired_joint_angls: ", desired_joint_angles)
        
        # Movement for the middle finger
        desired_joint_angles_appendVar = self.calculate_finger_angles(
            finger_type = 'middle',
            finger_joint_coords = hand_keypoints['middle'],
            hand_keypoints = hand_keypoints     # für die Berechnung der Rotation werden zusätzlich alle Keypoints weitergegeben
            #
            #curr_angles = desired_joint_angles,
            #moving_avg_arr = self.moving_average_queues['index']
        )
        
        desired_joint_angles.extend(desired_joint_angles_appendVar)
        
        # Movement for the ring finger
        desired_joint_angles_appendVar = self.calculate_finger_angles(
            finger_type = 'ring',
            finger_joint_coords = hand_keypoints['ring'],
            hand_keypoints = hand_keypoints     # für die Berechnung der Rotation werden zusätzlich alle Keypoints weitergegeben
            #
            #curr_angles = desired_joint_angles,
            #moving_avg_arr = self.moving_average_queues['index']
        )
        
        desired_joint_angles.extend(desired_joint_angles_appendVar)
        
        # Movement for the thumb finger
        desired_joint_angles_appendVar = self.calculate_finger_angles(
            finger_type = 'thumb',
            finger_joint_coords = hand_keypoints['thumb'],
            hand_keypoints = hand_keypoints     # für die Berechnung der Rotation werden zusätzlich alle Keypoints weitergegeben
            #
            #curr_angles = desired_joint_angles,
            #moving_avg_arr = self.moving_average_queues['index']
        )
        
        desired_joint_angles.extend(desired_joint_angles_appendVar)
        #print("desired_joint_angles: ", len(desired_joint_angles))
        
        # schicken der Daten an den Roboter, welcher selber den RosLink/Controler verwendet um die Daten via Ros weiter zu schicken
        self._robot.move(desired_joint_angles)
        
       
    
        
       
    
