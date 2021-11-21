import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray


camera = PiCamera()
camera.resolution(640,480)
camera.framerate = 30
camera.brightness = 65

raw_footage = PiRGBArray(camera, size(640,480))

for frame in camera.capture_continuous(raw_footage, format="rgb", use_video_port=True):
    img = frame.array
    cv2.imshow("Original Image", img)
    raw_capture.truncate(0)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break