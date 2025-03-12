import os
from digit_interface import Digit
from .recorder import Recorder
import cv2
import time
import numpy as np
from openteach.utils.network import ZMQCameraSubscriber
from openteach.constants import VR_FREQ,CAM_FPS,DEPTH_RECORD_FPS,IMAGE_RECORD_RESOLUTION,CAM_FPS_SIM,IMAGE_RECORD_RESOLUTION_SIM
from openteach.utils.timer import FrequencyTimer
from openteach.utils.files import store_pickle_data
 
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

# die Devices können mit v4l2-ctl --list-devices ausgegeben werden
# mit dem Terminalbefehl: v4l2-ctl --list-formats-ext -d /dev/video0
# kann das unterstützte Videoformat des jeweiligen Gerätes ausgegeben werden
# --> der Ditisensor verwendet das Format YUYV

class DigitSensRecorder(Recorder):
    def __init__(
        self,
        host,
        image_stream_port,
        storage_path,
        filename,
        resolution
        ):

        # Subscribing to the image stream port
        print("DigitImage Stream Port", image_stream_port)
        self._host, self._image_stream_port = host, image_stream_port
        self.image_subscriber = ZMQCameraSubscriber(
            host = host,
            port = image_stream_port,
            topic_type = 'RGB'
        )

        self.timer = FrequencyTimer(CAM_FPS)

        # Storage path for file
        self._filename = filename
        self._recorder_file_name = os.path.join(storage_path, filename + '.avi')
        self._metadata_filename = os.path.join(storage_path, filename + '.metadata')
        self._pickle_filename = os.path.join(storage_path, filename + '.pkl')

        # the resolution of the digit Sensor
        #resolution = (640,480)
        #resolution = (320,240)
        print("resolution digitSens-Recorder: ", resolution)

        self.recorder = cv2.VideoWriter(
            self._recorder_file_name, 
            cv2.VideoWriter_fourcc(*'XVID'), 
            #cv2.VideoWriter_fourcc(*'MJPG'), 
            CAM_FPS, 
            resolution
        )
        self.timestamps = []
        self.frames = []

        #d.set_fps(0.1)
        #print(d.get_frame())



    def stream(self):
        print('Starting to record RGB frames from port: {}'.format(self._image_stream_port))

        self.num_image_frames = 0
        self.record_start_time = time.time()

        while True:
            try:
                self.timer.start_loop()
                image, timestamp = self.image_subscriber.recv_rgb_image()
                #print("timestamp: ",timestamp)
                # um das Bild speicher zu können, müssen die Achsen getauscht werden
                image = image.transpose(1, 0, 2)  # Tauscht die Achsen
                #print("shape: ", np.shape(image))
                self.recorder.write(image)
                self.timestamps.append(timestamp)
                
                self.frames.append(np.array(image))
    
                self.num_image_frames += 1
                self.timer.end_loop()
            except KeyboardInterrupt:
                self.record_end_time = time.time()
                break
        self.image_subscriber.stop()

        # Displaying statistics
        self._display_statistics(self.num_image_frames)
        
        # Saving the metadata
        self._add_metadata(self.num_image_frames)
        self.metadata['timestamps'] = self.timestamps
        self.metadata['recorder_ip_address'] = self._host
        self.metadata['recorder_image_stream_port'] = self._image_stream_port

        # Storing the data
        print('Storing the final version of the video...')
        self.recorder.release()
        store_pickle_data(self._metadata_filename, self.metadata)
        print('Stored the video in {}.'.format(self._recorder_file_name))
        print('Stored the metadata in {}.'.format(self._metadata_filename))


