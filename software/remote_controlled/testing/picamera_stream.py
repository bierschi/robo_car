#!/usr/bin/env python3
import socket
import time
import picamera


def main():
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 24
    camera.rotation = 180

    server_socket = socket.socket()
    server_socket.bind(('0.0.0.0', 8001))
    server_socket.listen(0)

    # Accept a single connection and make a file-like object out of it
    connection = server_socket.accept()[0].makefile('wb')
    try:
        camera.start_recording(connection, format='h264')
        camera.wait_recording(60)
        camera.stop_recording()
    finally:
        connection.close()
        server_socket.close()


if __name__ == '__main__':
    main()

