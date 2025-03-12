import numpy as np
import time
import rospy
from copy import deepcopy as copy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState 

import uuid


class DexArmControl():
    def __init__(self, record_type=None, robot_type='both'):
        self.instance_id = uuid.uuid4()  

    # Initialize Controller Specific Information
        try:
                rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
                pass

        currentJointStateSubscriber_callback = None

        # initialisierung der Variablen
        self.robotHand_joint_state = None
        self.commanded_robotHand_joint_state = None
        self.frankaArm_measuredState = None
        self.frankaArm_commandedState = None
        #self.myRobotHand_joint_state = None

    # Publisher initialisieren, der auf ein ROS-Topic veröffentlicht
        self.hand_publisher = rospy.Publisher('/leaphand/joint_command', Float64MultiArray, queue_size=10)
        self.hand_subscriber = rospy.Subscriber('/leaphand/joint_currentState', Float64MultiArray, self._callback_robotHand_joint_state)                   # subscribed zum OT-Framework und erhält die States
        # in dem File wird zu den Commanded Joint States Subscribed (obwohl sie auch gepublished werden), damit die commanded Joint States in allen Instanzen des Objektes verfügbar sind
        self.handCommands_subscriber = rospy.Subscriber('/leaphand/joint_command', Float64MultiArray, self._callback_commandedRobotHand_joint_state)       # wird benötigt für den Recorder der commandedJointStates
        # -> wenn etwas gepublished wird, dann soll die Callbackfunktion "_callback_commandedRobotHand_joint_state" aufgerufen werden, welche die Variable commanded_robotHand_joint_state "aktualisiert"
        # vermutlich kann man das Ganz auch ohne Subscriber machen, sondern direkt beim Publishen der Nachricht die Funktion auch aufrufen
        
        # Publisher der Nachrichten für den FrankaArm
        self.frankaHandFramePos_publisher = rospy.Publisher('/frankaArm/commandedHandFramePosition', Float64MultiArray, queue_size=10)
        self.frankaRecordArmState_subscriber = rospy.Subscriber('/frankaArm/measuredRecordArmState', Float64MultiArray, self._callback_frankaRecordArm_state)
        self.frankaCommandedArmState_subscriber = rospy.Subscriber('/frankaArm/commandedRecordArmState', Float64MultiArray, self._callback_frankaCommandedArm_state)

        
        
        #self.myHand_subscriber = rospy.Subscriber('/leaphand/myJoint_currentState', JointState, self._myCallback_robotHand_joint_state)       # subscribed zum OT-Framework und erhält die States
        #self.hand_subscriber = rospy.Subscriber('/leaphand/joint_currentState', Float64MultiArray, self.get_robot_position)       # subscribed zum OT-Framework und erhält die States
        #self.hand_publisher = rospy.Subscriber('/leaphand/joint_currentState', Float64MultiArray)

        
    # Rostopic callback functions
    def _callback_robotHand_joint_state(self, joint_state):
        self.robotHand_joint_state = joint_state
        #print("self.robotHand_joint_state: ", self.robotHand_joint_state)
        #print(f"Von der Instanz {self.instance_id}")


    #Commanded joint state is basically the joint state being sent as an input to the controller
    def _callback_commandedRobotHand_joint_state(self, joint_state):
        self.commanded_robotHand_joint_state = joint_state

    def _callback_frankaRecordArm_state(self, receivedState):
        self.frankaArm_measuredState = receivedState

    def _callback_frankaCommandedArm_state(self, receivedState):
        self.frankaArm_commandedState = receivedState

    # State information function
    def get_measuredLeapHandState(self):
        # wenn der Code ausgeführt wird, während self.robotHand_joint_state noch Null ist - also, wenn die Callbackfunktion noch nicht ausgeführt wurde
        # dann wirft der Code Errors
        if self.robotHand_joint_state is None:
            return None

        # Get the robot joint state 
        raw_joint_state =self.robotHand_joint_state
        joint_state = dict(
            position = np.array(raw_joint_state.data, dtype = np.float32),
            timestamp = rospy.get_time()
        )

        return joint_state
    

    # Commanded joint state is the joint state being sent as an input to the controller
    def get_commandedLeapHandState(self):
        

        if self.commanded_robotHand_joint_state is None:
            #print("Warte auf die ROS-Nachricht...")
            return None


        raw_joint_state = copy(self.commanded_robotHand_joint_state)
        #print("commanded RobotHand JointStates: ", raw_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.data, dtype = np.float32),
            timestamp = rospy.get_time()
         )
        return joint_state
        
    def get_measuredFrankaArmState(self):

        if self.frankaArm_measuredState is None:
            return None

        raw_state = copy(self.frankaArm_measuredState)
        #print("commanded RobotHand JointStates: ", raw_joint_state)

        measuredArmState = dict(
            position = np.array(raw_state.data, dtype = np.float32),
            timestamp = rospy.get_time()
         )
        return measuredArmState

    def get_commandedFrankaArmState(self):

        if self.frankaArm_commandedState is None:
            return None

        raw_state = copy(self.frankaArm_commandedState)
        #print("commanded RobotHand JointStates: ", raw_joint_state)

        measuredArmState = dict(
            position = np.array(raw_state.data, dtype = np.float32),
            timestamp = rospy.get_time()
         )
        return measuredArmState




    # Get the robot joint/cartesian position
    def get_robot_position(self, msg):
        
        #if self.currentJointStateSubscriber_callback != None:
        #    self.currentJointStateSubscriber_callback(msg)

        #robotJointStates = msg.data
        #print("##############################################")
        #print(robotJointStates[0])
        #print(robotJointStates[1])

        pass
        

    # Get the robot joint velocity
    def get_robot_velocity(self):
        #Get Robot Velocity
        pass

    # Get the robot joint torque
    def get_robot_torque(self):
        # Get torque applied by the robot.
        pass

    # Get the commanded robot joint position
    def get_commanded_robot_joint_position(self):
        pass

    # Movement functions
    def move_robot(self, joint_angles):
        pass

    # Home Robot
    def home_robot(self):
        pass

    # Reset the Robot
    def reset_robot(self):
        pass

    # Full robot commands
    def move_robot(self, joint_angles, arm_angles):
        pass

    def arm_control(self, arm_pose):
        pass

    #Home the Robot
    def home_robot(self):
        pass
        # For now we're using cartesian values
     


    def move_hand(self, teleop_handAngles):
        #print("moveHand Called (RobotConroler)")
        

        # Nachricht erstellen
        msg = Float64MultiArray()
        msg.data = teleop_handAngles  # Hier eine Liste der Winkelwerte setzen

        #print("teleop_handAngles: ", teleop_handAngles)
        #self.commanded_robotHand_joint_state = teleop_handAngles    # Daten in der Variable dieser Klasse für weitere Nutzung speichern

        # Nachricht über den Publisher senden
        self.hand_publisher.publish(msg)
        print("leapHand commands sent to robot.")
        #rospy.loginfo(f"Published joint angles: {teleop_handAngles} to /leaphand/joint_command")

        #pass

    def move_arm(self, handFramePosition):
        
        # Nachricht erstellen
        msg = Float64MultiArray()
        msg.data = handFramePosition  # Hier eine Liste der Winkelwerte setzen

        # Nachricht über den Publisher senden
        self.frankaHandFramePos_publisher.publish(msg)
        print("handframe commands sent to robot.")
        #rospy.loginfo(f"Published joint angles: {teleop_handAngles} to /leaphand/joint_command")



