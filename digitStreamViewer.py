import cv2
import numpy as np
from time import sleep
from openteach.utils.network import ZMQCameraSubscriber
import argparse

# das File ist zum Testen der DigitStreams gedacht
# -> um im laufenden Betrieb sich die Steams ansehen zu können

class VideoStreamViewer:
    def __init__(self, host='0.0.0.0', port=10015):
        # Initialisiere den ZMQCameraSubscriber
        self.image_subscriber = ZMQCameraSubscriber(
            host=host,
            port=port,
            topic_type='RGB'
        )

    def start_stream(self):
        while True:
            
            try:
                
                # Lese das Bild und den Zeitstempel
                image, timestamp = self.image_subscriber.recv_rgb_image()

                # Falls nötig, die Achsen transponieren
                image = image.transpose(1, 0, 2)

                # Zeige das Bild im Fenster an
                cv2.imshow("Video Stream", image)
                
                # Beenden, wenn 'q' gedrückt wird
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as e:
                print("Fehler beim Empfang des Bildes:", e)
                sleep(0.1)  # Warte kurz, um eine hohe Abfragefrequenz zu vermeiden

        # Schließe das Fenster nach Beendigung
        cv2.destroyAllWindows()

# Hauptprogramm
if __name__ == "__main__":
    # Argumente einrichten
    parser = argparse.ArgumentParser(description="Digit Stream Viewer")
    parser.add_argument("--port", type=int, default=10015, help="Port number for the ZMQ stream")
    args = parser.parse_args()

    # Viewer starten
    viewer = VideoStreamViewer(host='0.0.0.0', port=args.port)
    viewer.start_stream()