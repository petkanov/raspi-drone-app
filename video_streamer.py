import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time, base64, socket, argparse
from utils import Utils

parser = argparse.ArgumentParser()
parser.add_argument('--drone_id')
parser.add_argument('--ip')
parser.add_argument('--port')
args = parser.parse_args()

CONTROL_HOST = args.ip
DRONE_ID = args.drone_id
VIDEO_SERVER_PORT = int(args.port)
FRAMES_PER_SECOND = 11

JPEG_QUALITY = 78
 
WIDTH = 512 #368
HEIGHT = 352 #304

camera = None
videoServerSocket = None



JAVA_SERVER_PORT = 1313

while(True):
    try:
        camera = PiCamera()
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = FRAMES_PER_SECOND
        rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))
        time.sleep(0.1)
        
        #videoServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #videoServerSocket.connect((CONTROL_HOST, VIDEO_SERVER_PORT))
        
        #droneIdBytes = Utils.createNetworkMessage(str.encode(DRONE_ID))
        #videoServerSocket.send(droneIdBytes)
        
        
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((CONTROL_HOST, JAVA_SERVER_PORT))
        
        

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image_data = frame.array
            
            image_data = cv2.rotate(image_data, cv2.ROTATE_180)
            
            #image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
            
            Eret_code, jpg_buffer = cv2.imencode(".jpg", image_data, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])

            networkMsg = Utils.createNetworkMessage(base64.b64encode(jpg_buffer))
            
            
            
            datagramMsgBytes = Utils.createUDP_DatagramMessage(DRONE_ID, base64.b64encode(jpg_buffer))
            sock.sendall(datagramMsgBytes)
            
            
            
            
            # videoServerSocket.sendall(networkMsg)
            
            rawCapture.truncate(0)
            
    except Exception as e:
        print(str(e))
        
        if camera != None:
            camera.close()
        if videoServerSocket != None:
            videoServerSocket.close()
        
        time.sleep(2)
