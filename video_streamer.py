import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time, base64, socket, argparse
from utils import Utils

parser = argparse.ArgumentParser()
parser.add_argument('--drone_id')
parser.add_argument('--port')
args = parser.parse_args()

DRONE_ID = args.drone_id
VIDEO_SERVER_PORT = int(args.port)
FRAMES_PER_SECOND = 15

JPEG_QUALITY = 80
CONTROL_HOST = '87.121.112.111'
 
WIDTH = 512 #368
HEIGHT = 400 #304

camera = None
videoServerSocket = None

while(True):
    try:
        camera = PiCamera()
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = FRAMES_PER_SECOND
        rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))
        time.sleep(0.1)
        
        videoServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        videoServerSocket.connect((CONTROL_HOST, VIDEO_SERVER_PORT))
        
        droneIdBytes = Utils.createNetworkMessage(str.encode(DRONE_ID))
        videoServerSocket.send(droneIdBytes)

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            #img_rot = cv2.rotate(frame.array, cv2.ROTATE_180)
            
            Eret_code, jpg_buffer = cv2.imencode(".jpg", frame.array, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])

            networkMsg = Utils.createNetworkMessage(base64.b64encode(jpg_buffer))
            
            videoServerSocket.sendall(networkMsg)
            
            rawCapture.truncate(0)
            
    except Exception as e:
        print(str(e))
        
        if camera != None:
            camera.close()
        if videoServerSocket != None:
            videoServerSocket.close()
        
        time.sleep(2)
