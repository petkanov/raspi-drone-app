import numpy as np
import zmq
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


class SerializingSocket(zmq.Socket):
    
    def send_array(self, A, msg='NoName', flags=0, copy=True, track=False):
        md = dict(
            msg=msg,
            dtype=str(A.dtype),
            shape=A.shape,
        )
        self.send_json(md, flags | zmq.SNDMORE)
        return self.send(A, flags, copy=copy, track=track)

    def send_jpg(self,
                 msg='NoName',
                 jpg_buffer=b'00',
                 flags=0,
                 copy=True,
                 track=False):
        
        md = dict(msg=msg, )
        self.send_json(md, flags | zmq.SNDMORE)
        return self.send(jpg_buffer, flags, copy=copy, track=track)

    def recv_array(self, flags=0, copy=True, track=False):
        md = self.recv_json(flags=flags)
        msg = self.recv(flags=flags, copy=copy, track=track)
        A = np.frombuffer(msg, dtype=md['dtype'])
        return (md['msg'], A.reshape(md['shape']))

    def recv_jpg(self, flags=0, copy=True, track=False):
        md = self.recv_json(flags=flags)  # metadata text
        jpg_buffer = self.recv(flags=flags, copy=copy, track=track)
        return (md['msg'], jpg_buffer)


class SerializingContext(zmq.Context):
    _socket_class = SerializingSocket
    

class ImageSender():
    
    def __init__(self, connect_to='tcp://127.0.0.1:5555'):
        self.zmq_context = SerializingContext()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.connect(connect_to)

    def send_image(self, msg, image):
        if image.flags['C_CONTIGUOUS']:
            self.zmq_socket.send_array(image, msg, copy=False)
        else:
            image = np.ascontiguousarray(image)
            self.zmq_socket.send_array(image, msg, copy=False)
        hub_reply = self.zmq_socket.recv()  # receive the reply message
        return hub_reply

    def send_jpg(self, msg, jpg_buffer):
        self.zmq_socket.send_jpg(msg, jpg_buffer, copy=False)
        hub_reply = self.zmq_socket.recv()  # receive the reply message
        return hub_reply

jpeg_quality = 95

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--videokey',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

video_key = args.videokey
print(video_key+' -------------------------------------------------------')

try:
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 25
        rawCapture = PiRGBArray(camera, size=(640, 480))
        time.sleep(0.1)
        
        sender = ImageSender(connect_to="tcp://localhost:5555")
        #sender = ImageSender(connect_to="tcp://192.168.170.57:5555")
        myName = "droneCam"

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            Eret_code, jpg_buffer = cv2.imencode(".jpg", frame.array, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            sender.send_jpg(myName, jpg_buffer)
            rawCapture.truncate(0)
            
except Exception as e:
        print(str(e))
finally:
        print("Done")
    