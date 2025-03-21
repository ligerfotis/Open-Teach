import os
import hydra
from abc import ABC
from .recorders.image import RGBImageRecorder, DepthImageRecorder, FishEyeImageRecorder
from .recorders.robot_state import RobotInformationRecord
from .recorders.sim_state import SimInformationRecord
from .recorders.sensors import XelaSensorRecorder
from .recorders.digitSensorRecorder import DigitSensRecorder
from .recorders.gelsightSensorRecorder import GelSightSensRecorder
from .sensors import *
from .sensors import DigitSensor
from .sensors import GelSightSensor
from multiprocessing import Process
from openteach.constants import *
import re



class ProcessInstantiator(ABC):
    def __init__(self, configs):
        self.configs = configs
        self.processes = []

    def _start_component(self,configs):
        raise NotImplementedError('Function not implemented!')

    def get_processes(self):
        return self.processes


class RealsenseCameras(ProcessInstantiator):
    """
    Returns all the camera processes. Start the list of processes to start
    the camera stream."
    """
    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_camera_processes()

    def _start_component(self, cam_idx):
        component = RealsenseCamera(
            stream_configs = dict(
                host = self.configs.host_address,
                port = self.configs.cam_port_offset + cam_idx
            ),
            cam_serial_num = self.configs.robot_cam_serial_numbers[cam_idx],
            cam_id = cam_idx + 1,
            cam_configs = self.configs.cam_configs,
            stream_oculus = True if self.configs.oculus_cam == cam_idx else False
        )
        component.stream()

    def _init_camera_processes(self):
        for cam_idx in range(len(self.configs.robot_cam_serial_numbers)):
            self.processes.append(Process(
                target = self._start_component,
                args = (cam_idx, )
            ))

class FishEyeCameras(ProcessInstantiator):
    """
    Returns all the fish eye camera processes. Start the list of processes to start
    the camera stream.
    """
    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_camera_processes()

    def _start_component(self, cam_idx):
        #print('cam_idx: {}, stream_oculus: {}'.format(cam_idx, True if self.configs.oculus_cam == cam_idx else False)) # Original
        print('cam_idx: {}, stream_oculus: {}'.format(cam_idx, True if self.configs.stream_oculus and self.configs.oculus_cam == cam_idx else False))  # ausgebesserte Version
        component = FishEyeCamera(
            cam_index=self.configs.fisheye_cam_numbers[cam_idx],
            stream_configs = dict(
                host = self.configs.host_address,
                port = self.configs.fish_eye_cam_port_offset+ cam_idx,
                set_port_offset = self.configs.fish_eye_cam_port_offset,
                width = self.configs.cam_configs.width,
                height = self.configs.cam_configs.height,
            ),
            
            stream_oculus = True if self.configs.stream_oculus and self.configs.oculus_cam == cam_idx else False,
            
        )
        print("self.configs.host_address: ", self.configs.host_address)
        print("self.configs.fish_eye_cam_port_offset: ", self.configs.fish_eye_cam_port_offset)
        component.stream()

    def _init_camera_processes(self):
        for cam_idx in range(len(self.configs.fisheye_cam_numbers)):
            self.processes.append(Process(
                target = self._start_component,
                args = (cam_idx, )
            ))

class DigitSensors(ProcessInstantiator):

    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_digitSensor_processes()

    def _start_component(self, digitSens_Nr):
        print('digitSens_Nr: {}'.format(digitSens_Nr))  # ausgebesserte Version
        
        # die Klasse befindet sich im Ordner openteach/components/sensors/digitSensor.py
        component = DigitSensor(
            digitSens_Nr=self.configs.digitSensor_numbers[digitSens_Nr],
            stream_configs = dict(
                host = self.configs.host_address,
                port = self.configs.digitSens_port_offset+ digitSens_Nr,
                set_port_offset = self.configs.digitSens_port_offset,
            )
        )
        
        #print("self.configs.host_address: ", self.configs.host_address)
        #print("self.configs.fish_eye_cam_port_offset: ", self.configs.fish_eye_cam_port_offset)
        component.stream()

    def _init_digitSensor_processes(self):
        for digitSens_Nr in range(len(self.configs.digitSensor_numbers)):
            self.processes.append(Process(
                target = self._start_component,
                args = (digitSens_Nr, )
            ))


class GelSightSensors(ProcessInstantiator):

    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_gelsightSensor_processes()

    def _start_component(self, gelsightSensor_Nr):
        print('gelsightSens_Nr: {}'.format(gelsightSensor_Nr))  # ausgebesserte Version
        # die Klasse befindet sich im Ordner openteach/components/sensors/digitSensor.py
        component = GelSightSensor(
            gelsightSensor_Nr=self.configs.gelsightSensor_numbers[gelsightSensor_Nr],
            gelsightSensor_width = self.configs.gelsight_configs.width,
            gelsightSensor_hight = self.configs.gelsight_configs.height,
            stream_configs = dict(
                host = self.configs.host_address,
                port = self.configs.gelsightSens_port_offset+ gelsightSensor_Nr,
                set_port_offset = self.configs.gelsightSens_port_offset,
            )
        )
        
        #print("self.configs.host_address: ", self.configs.host_address)
        #print("self.configs.fish_eye_cam_port_offset: ", self.configs.fish_eye_cam_port_offset)
        component.stream()

    def _init_gelsightSensor_processes(self):
        for gelsightSensor_Nr in range(len(self.configs.gelsightSensor_numbers)):
            self.processes.append(Process(
                target = self._start_component,
                args = (gelsightSensor_Nr, )
            ))

class TeleOperator(ProcessInstantiator):
    """
    Returns all the teleoperation processes. Start the list of processes 
    to run the teleop.
    """
    def __init__(self, configs):
        super().__init__(configs)
      
        # For Simulation environment start the environment as well
        if configs.sim_env:
            self._init_sim_environment()
        # Start the Hand Detector
        self._init_detector()
        # Start the keypoint transform
        self._init_keypoint_transform()
        self._init_visualizers()


        if configs.operate: 
            self._init_operator()
        
    #Function to start the components
    def _start_component(self, configs):    
        component = hydra.utils.instantiate(configs)
        component.stream()

    #Function to start the detector component
    def _init_detector(self):
        self.processes.append(Process(
            target = self._start_component,
            args = (self.configs.robot.detector, )
        ))

    #Function to start the sim environment
    def _init_sim_environment(self):
         for env_config in self.configs.robot.environment:
            self.processes.append(Process(
                target = self._start_component,
                args = (env_config, )
            ))

    #Function to start the keypoint transform
    def _init_keypoint_transform(self):
        for transform_config in self.configs.robot.transforms:
            self.processes.append(Process(
                target = self._start_component,
                args = (transform_config, )
            ))

    #Function to start the visualizers
    def _init_visualizers(self):
       
        for visualizer_config in self.configs.robot.visualizers:
            self.processes.append(Process(
                target = self._start_component,
                args = (visualizer_config, )
            ))
        # XELA visualizer
        if self.configs.run_xela:
            for visualizer_config in self.configs.xela_visualizers:
                self.processes.append(Process(
                    target = self._start_component,
                    args = (visualizer_config, )
                ))

    #Function to start the operator
    def _init_operator(self):
        for operator_config in self.configs.robot.operators:
            
            self.processes.append(Process(
                target = self._start_component,
                args = (operator_config, )

            ))

    

# Data Collector Class
class Collector(ProcessInstantiator):
    """
    Returns all the recorder processes. Start the list of processes 
    to run the record data.
    """
    def __init__(self, configs):
        super().__init__(configs)

        self.demonstrationSetPath = self.configs.storage_path + "/" + self.configs.demonstrationset_name
        self._create_storage_dir(self.demonstrationSetPath)
        highestStorageNr = self.findHighestStoragePathNumber(self.demonstrationSetPath)
        self._storage_path = self.configs.storage_path + "/" + self.configs.demonstrationset_name + "/demoNr_" + str(highestStorageNr+1)

        self._create_storage_dir(self._storage_path)
        self._init_camera_recorders()
        # Initializing the recorders
        if self.configs.sim_env is True:
            self._init_sim_recorders()
        else:
            print("Initialising robot recorders")
            self._init_robot_recorders()
        
        
        if self.configs.is_xela is True:
            self._init_sensor_recorders()

        if self.configs.is_digit is True:
            self._init_digitSensor_recorder()

        if self.configs.is_gelSight is True:
            self._init_gelsightSensor_recorder()

    # die Funktion wurde von ChatGPT geschrieben
    def findHighestStoragePathNumber(self, folder_path):
        highest_number = 0
        pattern = r"^demoNr_(\d+)$"  # Muster für Verzeichnisnamen, die "demoNr_[Zahl]" entsprechen

        for dir_name in os.listdir(folder_path):
            # Prüfen, ob der Name dem Muster entspricht
            match = re.match(pattern, dir_name)
            if match:
                # Extrahiere die Zahl und konvertiere sie in einen Integer
                number = int(match.group(1))
                # Aktualisiere das höchste gefundene Nummer
                if number > highest_number:
                    highest_number = number

        return highest_number

    def _create_storage_dir(self, dirPath):
        if os.path.exists(dirPath):
            return 
        else:
            os.makedirs(dirPath)

    #Function to start the components
    def _start_component(self, component):
        component.stream()

    # Record the rgb components
    def _start_rgb_component(self, cam_idx=0):
        # This part has been isolated and made different for the sim and real robot
        # If using simulation and real robot on the same network, only one of them will stream into the VR. Close the real robot realsense camera stream before launching simulation.
        if self.configs.sim_env is False:
            print("RGB function")
            component = RGBImageRecorder(
                host = self.configs.host_address,
                image_stream_port = self.configs.cam_port_offset + cam_idx,
                storage_path = self._storage_path,
                filename = 'cam_{}_rgb_video'.format(cam_idx)
            )
        else:
            print("Reaching correct function")
            component = RGBImageRecorder(
            host = self.configs.host_address,
            image_stream_port = self.configs.sim_image_port+ cam_idx,
            storage_path = self._storage_path,
            filename = 'cam_{}_rgb_video'.format(cam_idx),
            sim = True
        )
        component.stream()

    # Record the depth components
    def _start_depth_component(self, cam_idx):
        if self.configs.sim_env is not True:
            component = DepthImageRecorder(
                host = self.configs.host_address,
                image_stream_port = self.configs.cam_port_offset + cam_idx + DEPTH_PORT_OFFSET,
                storage_path = self._storage_path,
                filename = 'cam_{}_depth'.format(cam_idx)
            )
        else:
            component = DepthImageRecorder(
                host = self.configs.host_address,
                image_stream_port = self.configs.sim_image_port + cam_idx + DEPTH_PORT_OFFSET,
                storage_path = self._storage_path,
                filename = 'cam_{}_depth'.format(cam_idx)
            )
        component.stream()

    #Function to start the camera recorders
    def _init_camera_recorders(self):
        if self.configs.sim_env is not True:
            print("Camera recorder starting")

            # das if mit dem else-Bereich, wurde von mir selber hinzugefügt
            if hasattr(self.configs, 'robot_cam_serial_numbers'):
                for cam_idx in range(len(self.configs.robot_cam_serial_numbers)):
                    #print(cam_idx)
                    self.processes.append(Process(
                        target = self._start_rgb_component,
                        args = (cam_idx, )
                    ))

                    self.processes.append(Process(
                        target = self._start_depth_component,
                        args = (cam_idx, )
                    ))
            else:   # selbst hinzugefügter Code
                # wenn keine Seriennummer in der Config vorhanden ist, dann wird keine realSense Kamera verwendet, sonder eine "normale"
                # -> in diesem Fall soll das if ausgeführt werden
                cam_idx = self.configs.oculus_cam
                self.processes.append(Process(
                    target = self._start_fish_eye_component,
                    args = (cam_idx, )
                ))


        else:
          
            for cam_idx in range(self.configs.num_cams):
                self.processes.append(Process(
                    target = self._start_rgb_component,
                    args = (cam_idx, )
                ))

                self.processes.append(Process(
                    target = self._start_depth_component,
                    args = (cam_idx, )
                ))


    #Function to start the sim recorders
    def _init_sim_recorders(self):
        port_configs = self.configs.robot.port_configs
        for key in self.configs.robot.recorded_data[0]:
            self.processes.append(Process(
                        target = self._start_sim_component,
                        args = (port_configs[0],key)))

    #Function to start the xela sensor recorders
    def _start_xela_component(self,
        controller_config
    ):
        component = XelaSensorRecorder(
            controller_configs=controller_config,
            storage_path=self._storage_path
        )
        component.stream()

    #Function to start the sensor recorders
    def _init_sensor_recorders(self):
        """
        For the XELA sensors 
        """
        for controller_config in self.configs.robot.xela_controllers:
            self.processes.append(Process(
                target = self._start_xela_component,
                args = (controller_config, )
            ))


    #Function to start the digit sensor recorders
    def _start_digitSens_component(self,
        digitSens_Nr
    ):
        resolution = (self.configs.digitCam_configs.width, self.configs.digitCam_configs.height)
        component = DigitSensRecorder(
            host = self.configs.host_address,
            image_stream_port = self.configs.digitSens_port_offset + digitSens_Nr,
            storage_path=self._storage_path,
            filename = 'digitSens_{}_video'.format(digitSens_Nr),
            resolution = resolution
        )
        component.stream()


    def _init_digitSensor_recorder(self):
        '''
        # for the DigitSensor 
        '''
        print("self.configs.digitSensor_numbers:", self.configs.digitCam_configs.width)
        for digitSens_Nr in range(len(self.configs.digitSensor_numbers)):
            self.processes.append(Process(
                target = self._start_digitSens_component,
                args = (digitSens_Nr, )
            ))

    #Function to start the digit sensor recorders
    def _start_gelsightSens_component(self,
        gelsightSens_Nr
    ):
        resolution = (self.configs.gelsight_configs.width, self.configs.gelsight_configs.height)
        component = GelSightSensRecorder(
            host = self.configs.host_address,
            image_stream_port = self.configs.gelsightSens_port_offset + gelsightSens_Nr,
            storage_path=self._storage_path,
            filename = 'gelsightSens_{}_video'.format(gelsightSens_Nr),
            resolution = resolution
        )
        component.stream()

    def _init_gelsightSensor_recorder(self):
        '''
        # for the GelSight
        '''
        for gelsightSens_Nr in range(len(self.configs.gelsightSensor_numbers)):
            self.processes.append(Process(
                target = self._start_gelsightSens_component,
                args = (gelsightSens_Nr, )
            ))


    #Function to start the fish eye recorders
    def _start_fish_eye_component(self, cam_idx):
        resolution = (self.configs.cam_configs.width, self.configs.cam_configs.height)  # selbst hinzugefügte Zeile
        component = FishEyeImageRecorder(
            host = self.configs.host_address,
            image_stream_port = self.configs.fish_eye_cam_port_offset + cam_idx,
            storage_path = self._storage_path,
            filename = 'cam_{}_fish_eye_video'.format(cam_idx),
            resolution = resolution
        )
        component.stream()

    #Function to start the robot recorders
    def _start_robot_component(
        self, 
        robot_configs, 
        recorder_function_key):
        component = RobotInformationRecord(
            robot_configs = robot_configs,
            recorder_function_key = recorder_function_key,
            storage_path = self._storage_path
        )

        component.stream()

    #Function to start the sim recorders
    def _start_sim_component(self,port_configs, recorder_function_key):
        component = SimInformationRecord(
                   port_configs = port_configs,
                   recorder_function_key= recorder_function_key,
                   storage_path=self._storage_path
        )
        component.stream()

    #Function to start the robot recorders - der "robot_controller_configs" ist eigentlich ein Objekt der Klasse LeapHand
    # des Files openteach/robot/leaphand welches hier unter den Namen robot_controller_configs weiter gegeben wird.
    def _init_robot_recorders(self):
        # Instantiating the robot classes
        for idx, robot_controller_configs in enumerate(self.configs.robot.controllers):
            for key in self.configs.robot.recorded_data[idx]:
                self.processes.append(Process(
                    target = self._start_robot_component,
                    args = (robot_controller_configs, key, )
                ))


    

   