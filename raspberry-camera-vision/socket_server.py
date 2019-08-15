import logging
import io
import socket
import struct
import threading
import socketserver
from http import server
import queue

from PIL import Image

from object_detection_yolo import ObjectDetectorYOLO


logging.basicConfig(level=logging.INFO)

PAGE = """\
<html>
<head>
<title>picamera MJPEG streaming demo</title>
</head>
<body>
<h1>PiCamera MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    logging.info('I want to get image!!')
                    image = image_queue.get()
                    logging.info('I got image!!')
                    with io.BytesIO() as output:
                        image.save(output, format="JPEG")
                        frame = output.getvalue()

                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def run_server():
    address = ('', 8000)
    streaming_server = StreamingServer(address, StreamingHandler)
    streaming_server.serve_forever()


class MyThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        logging.info("Starting " + self.name)
        if self.threadID == 1:
            run_server()
        else:
            start_socket_server()
        logging.info("Exiting " + self.name)


def start_socket_server():
    detector = ObjectDetectorYOLO()
    logging.info("TensorFlow is ready")

    HOST = '192.168.0.20'  # Standard loopback interface address (localhost)
    PORT = 8001  # Port to listen on (non-privileged ports are > 1023)
    # Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
    # all interfaces)
    server_socket = socket.socket()
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    # Accept a single connection and make a file-like object out of it
    connection = server_socket.accept()[0].makefile('rb')
    logging.info('Connection established')

    try:
        while True:
            # Read the length of the image as a 32-bit unsigned int. If the
            # length is zero, quit the loop
            image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
            if not image_len:
                break
            # Construct a stream to hold the image data and read the image
            # data from the connection
            image_stream = io.BytesIO()
            image_stream.write(connection.read(image_len))
            # Rewind the stream, open it as an image with PIL and do some
            # processing on it
            image_stream.seek(0)
            image = Image.open(image_stream)

            image = detector.detect(image)
            image_queue.put(image)
            logging.info('Image is %dx%d' % image.size)
            # image.verify()
            # image.show()
    finally:
        connection.close()
        server_socket.close()


def run():
    global image_queue
    image_queue = queue.Queue(0)

    thread1 = MyThread(1, "Thread-1")
    thread2 = MyThread(2, "Thread-2")
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
