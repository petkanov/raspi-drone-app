import numpy as np
import cv2
from flask import Flask, render_template, Response
import zmq

 

class ImageHub():

    def __init__(self, open_port='tcp://*:5555'):
        self.zmq_context = SerializingContext()
        self.zmq_socket = self.zmq_context.socket(zmq.REP)
        self.zmq_socket.bind(open_port)

    def recv_image(self, copy=False):
        msg, image = self.zmq_socket.recv_array(copy=False)
        return msg, image

    def recv_jpg(self, copy=False):
        msg, jpg_buffer = self.zmq_socket.recv_jpg(copy=False)
        return msg, jpg_buffer

    def send_reply(self, reply_message=b'OK'):
        self.zmq_socket.send(reply_message)


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


def gen():
    imageHub = ImageHub() 
        
    while True:
        rpiName, frame = imageHub.recv_jpg()
        frame = cv2.imdecode(np.frombuffer(frame, dtype='uint8'), -1)
        imageHub.send_reply(b'OK')
    
        ret, jpeg = cv2.imencode('.jpg', frame)
        jpeg.tobytes()
        frame = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    #app.run(host='localhost', port=5002, debug=True)
    app.run(host='192.168.170.57', port=5002, debug=True)
