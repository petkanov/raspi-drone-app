import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time, base64, socket, argparse
from utils import Utils

parser = argparse.ArgumentParser()
parser.add_argument('--name')
parser.add_argument('--port')
args = parser.parse_args()

NAME = args.name
VIDEO_SERVER_PORT = int(args.port)
FRAMES_PER_SECOND = 15

JPEG_QUALITY = 80
CONTROL_HOST = '91.230.195.104'
 
WIDTH = 512 #368
HEIGHT = 400 #304

try:
        camera = PiCamera()
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = FRAMES_PER_SECOND
        rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))
        time.sleep(0.1)
        
        videoServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        videoServerSocket.connect((CONTROL_HOST, VIDEO_SERVER_PORT))

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            
            Eret_code, jpg_buffer = cv2.imencode(".jpg", frame.array, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])

            networkMsg = Utils.createNetworkMessage(base64.b64encode(jpg_buffer))
            
            videoServerSocket.sendall(networkMsg)
            
            rawCapture.truncate(0)
            
except Exception as e:
        print(str(e))
        
finally:
        print("Done")
    