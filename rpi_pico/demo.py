import ov2640
import gc
import time
import sys

FILE_NAME = 'image2.jpg'

def main():
    print("initializing camera")
    cam = ov2640.ov2640(resolution=ov2640.OV2640_320x240_JPEG)
    # cam = ov2640.ov2640(resolution=ov2640.OV2640_1024x768_JPEG)
    print(gc.mem_free())

    image_length = cam.capture_to_file(FILE_NAME, True)
    print("captured image is %d bytes" % image_length)
    print("image is saved to %s" % FILE_NAME)

if __name__ == '__main__':
    main()
