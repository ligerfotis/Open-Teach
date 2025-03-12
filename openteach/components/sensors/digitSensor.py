import numpy as np
import cv2
from digit_interface import Digit
from openteach.utils.network import ZMQCameraPublisher, ZMQCompressedImageTransmitter
from openteach.components import Component
from openteach.constants import VR_FREQ,CAM_FPS,DEPTH_RECORD_FPS,IMAGE_RECORD_RESOLUTION,CAM_FPS_SIM,IMAGE_RECORD_RESOLUTION_SIM
from openteach.utils.timer import FrequencyTimer
import time

class DigitSensor(Component):
    def __init__(self,digitSens_Nr,stream_configs):
        # Disabling scientific notations
        np.set_printoptions(suppress=True)
        self.sensor_id = digitSens_Nr
        #self.output_file = output_file
        self._stream_configs = stream_configs
       
        # Different publishers to avoid overload
        self.rgb_publisher = ZMQCameraPublisher(
            host = stream_configs['host'],
            port = stream_configs['port']#(0 if self.sensor_id == 24 else self.sensor_id)
        )
        
        #print("stream_configs['host']: ", stream_configs['port'])
        
        
        self.timer = FrequencyTimer(CAM_FPS) # 30 fps

        # Starting the Fisheye pipeline
        self._start_digitSensor()
        

    def _start_digitSensor(self):

        # IDs von den Digit Sensoren: 
        # D20843
        # D20846
        # D20850

        '''
        d = Digit("D20843") # Unique serial number
        d.connect()
        d.set_fps(30)
        print(d.fps)
        print(d.resolution)
        d.show_view()
        d.disconnect()
        '''
        print("DigitSens Id is ", self.sensor_id)
        self.digitSens = Digit(self.sensor_id)
        self.digitSens.connect()
        #self.digitSens.set_fps(1) # it can only be set to specific values - setting it to the value 30 - unklar, ob die Zeile benötigt wird/ ob sie eine Auswirkung hat
        
        self.digitSens.set_fps(Digit.STREAMS["QVGA"]["fps"]["30fps"])
        #self.digitSens.set_resolution(Digit.STREAMS["VGA"])     # 640x480
        #self.digitSens.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])
        #self.digitSens.set_resolution(Digit.STREAMS["QVGA"])   # 320x240 - die kleinere Auflösung kann nicht gespeichert werden
        print("Digitsensor-streamer-fps: ", self.digitSens.fps)
        print("Digitsensor-streamer-resolution: ", self.digitSens.resolution)
        


    def get_rgb_images(self):
        frame = None
        while frame is None:
            frame = self.digitSens.get_frame()
        timestamp = time.time()
        return frame, timestamp
    def stream(self):
        # Starting the fisheye stream
        self.notify_component_start('DigitSensor')
        print(f"Started the pipeline for DigitSensor: {self.sensor_id}!")
        print("Starting stream on {}:{}...\n".format(self._stream_configs['host'], self._stream_configs['port']))
        
        while True:
            try:
                self.timer.start_loop()
                color_image,timestamp = self.get_rgb_images()
                #print("color_image: ", color_image)
                #print(color_image.dtype)
                #print(color_image.shape)


                # Publishing the rgb images
                self.rgb_publisher.pub_rgb_image(color_image, timestamp)
                
                self.timer.end_loop()
                if cv2.waitKey(1) == ord('q'):
                    break
            except KeyboardInterrupt:
                break
        self.digitSens.disconnect()
        print('Shutting down pipeline for the digitSensor {}.'.format(self.sensor_id))
        self.rgb_publisher.stop()
