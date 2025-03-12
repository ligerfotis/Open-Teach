import numpy as np
import cv2
from digit_interface import Digit
from openteach.utils.network import ZMQCameraPublisher, ZMQCompressedImageTransmitter
from openteach.components import Component
from openteach.constants import VR_FREQ,CAM_FPS,DEPTH_RECORD_FPS,IMAGE_RECORD_RESOLUTION,CAM_FPS_SIM,IMAGE_RECORD_RESOLUTION_SIM
from openteach.utils.timer import FrequencyTimer
import time

class GelSightSensor(Component):
    def __init__(self,gelsightSensor_Nr,gelsightSensor_width,gelsightSensor_hight, stream_configs):
        # Disabling scientific notations
        np.set_printoptions(suppress=True)
        self.sensor_id = gelsightSensor_Nr
        self.sensor_width = gelsightSensor_width
        self.sensor_hight = gelsightSensor_hight
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
        self._start_gelsightSensor()
        

    def _start_gelsightSensor(self):


        self.cap = cv2.VideoCapture(self.sensor_id)
       
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.sensor_width)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.sensor_hight)

        # Überprüfen, ob die Webcam erfolgreich geöffnet wurde
        if not self.cap.isOpened():
            print("Fehler: Webcam konnte nicht geöffnet werden")
            exit()
        print("Cap is ", self.cap.isOpened())
        # Check if the camera is opened successfully, wait until it is
        while not self.cap.isOpened():
            cap=self.cap.isOpened()



    def get_rgb_images(self):
        frame = None
        while frame is None:
            ret, frame = self.cap.read()
        timestamp = time.time()
        return frame, timestamp
    def stream(self):
        # Starting the fisheye stream
        self.notify_component_start('GelSightSensor')
        print(f"Started the pipeline for GelSightSensor: {self.sensor_id}!")
        print("Starting stream on {}:{}...\n".format(self._stream_configs['host'], self._stream_configs['port']))
        
        while True:
            try:
                self.timer.start_loop()
                color_image,timestamp = self.get_rgb_images()
                #print("color_image: ", color_image)
                        # Frame auf gewünschte Größe skalieren
                color_image = cv2.resize(color_image, (self.sensor_width, self.sensor_hight))
                # Publishing the rgb images
                #print(color_image.shape)
                #print("color_image: ", color_image)
                self.rgb_publisher.pub_rgb_image(color_image, timestamp)
                
                self.timer.end_loop()
                if cv2.waitKey(1) == ord('q'):
                    break
            except KeyboardInterrupt:
                break
        self.digitSens.disconnect()
        print('Shutting down pipeline for the gelsightSensor {}.'.format(self.sensor_id))
        self.rgb_publisher.stop()