from . import ov2640_constants
from . import ov2640_lores_constants
from . import ov2640_hires_constants
import machine
import time
import ubinascii
import uos
import gc
from . import contextlib


def setup():
    i2c = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=1000000)
    br = 8 * 1000*1000
    spi = machine.SPI(0, baudrate=2 * 1000 * 1000, polarity=0, phase=0,
                      mosi=machine.Pin(3), miso=machine.Pin(4), sck=machine.Pin(2))
    return i2c, spi

def i2c_dump(i2c):
    addrs = i2c.scan()
    print('Devices detected on on i2c:')
    for a in addrs:
        print('0x%x' % a)

class OvBus:
    def __init__(self, i2c, spi, cs_pin=5):
        self.cs_pin = cs_pin
        self.hspi = spi
        # init the i2c interface
        self.i2c = i2c
        # first init spi assuming the hardware spi is connected
        self.hspi.init()
        self.cs_pin = machine.Pin(5, machine.Pin.OUT)
        self.cs_pin.value(1)

    def write_register_set(self, registers):
        """Write a set of registers. Style is a list of [address, item] values."""
        for reg_addr, val in registers:
            #print("writing byte %s to addr %x register addr %x" % \
            #   (ubinascii.hexlify(val), addr, raddr))
            self.i2c.writeto_mem(ov2640_constants.SENSORADDR, reg_addr, bytes([val]))

    def cam_spi_write(self, address, value):
        self.cs_pin.value(0)
        modebit = b'\x80'
        d = bytes([address[0] | modebit[0], value[0]])
        #print("bytes %s" % ubinascii.hexlify(d))
        #print (ubd.hex())
        self.hspi.write(d)
        self.cs_pin.value(1)

    def cam_spi_read(self, address):
        self.cs_pin.value(0)
        maskbits = b'\x7f'
        wbuf = bytes([address[0] & maskbits[0]])
        self.hspi.write(wbuf)
        buf = self.hspi.read(1)
        self.cs_pin.value(1)
        return buf

    def cam_spi_read_length(self, nbytes, address):
        self.cs_pin.value(0)
        maskbits = 0x7f
        write_data = address & maskbits
        self.hspi.write(bytes([write_data]))
        result = self.hspi.read(nbytes, write_data)
        self.cs_pin.value(1)
        return result


class ov2640:
    def __init__(self, ov_bus, resolution=ov2640_lores_constants.OV2640_320x240_JPEG):
        self.ov_bus = ov_bus
        self.standby = False

        # select register set
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0xff, b'\x01')
        # initiate system reset
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0x12, b'\x80')

        # let it come up
        time.sleep_ms(200)

        # jpg init registers
        self.ov_bus.write_register_set(ov2640_constants.OV2640_JPEG_INIT)
        self.ov_bus.write_register_set(ov2640_constants.OV2640_YUV422)
        self.ov_bus.write_register_set(ov2640_constants.OV2640_JPEG)

        # select register set
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0xff, b'\x01')
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0x15, b'\x00')

        # select jpg resolution
        self.ov_bus.write_register_set(resolution)

        self.test_spi_bus()

        # register set select
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0xff, b'\x01')
        # check the camera type
        self.check_camera_type()

    def check_camera_type(self):
        parta = self.ov_bus.i2c.readfrom_mem(ov2640_constants.SENSORADDR, 0x0a, 1)
        partb = self.ov_bus.i2c.readfrom_mem(ov2640_constants.SENSORADDR, 0x0b, 1)
        if (parta != b'\x26') or (partb != b'\x42'):
            print("ov2640_init: device type does not appear to be ov2640, bytes: %s/%s" % \
                  (ubinascii.hexlify(parta), ubinascii.hexlify(partb)))
        else:
            print("ov2640_init: device type looks correct, bytes: %s/%s" % \
                  (ubinascii.hexlify(parta), ubinascii.hexlify(partb)))

    def test_spi_bus(self):
        # test the SPI bus
        self.ov_bus.cam_spi_write(b'\x00', b'\x55')
        res = self.ov_bus.cam_spi_read(b'\x00')
        print("ov2640 init:  register test return bytes %s" % ubinascii.hexlify(res))
        if res == b'\x55':
            print("ov2640_init: register test successful")
        else:
            print("ov2640_init: register test failed!")

    def capture_to_file(self, file_name, overwrite):
        if overwrite:
            print("deleting old file %s" % file_name)
            try:
                uos.remove(file_name)
            except OSError:
                pass
        total = 0
        with open(file_name, 'ab') as fd:
            for buffer, length in self.capture():
                fd.write(buffer)
                total += length
        return total

    def capture(self):
        """Will yield picture buffer chunks"""
        # bit 0 - clear FIFO write done flag
        self.ov_bus.cam_spi_write(b'\x04', b'\x01')

        # bit 1 - start capture then read status
        self.ov_bus.cam_spi_write(b'\x04', b'\x02')
        time.sleep_ms(10)

        # read status
        res = self.ov_bus.cam_spi_read(b'\x41')
        #if (res == b'\x00'):
        #    print("initiate capture may have failed, return byte: %s" % ubinascii.hexlify(res))

        # read the image from the camera fifo
        ## Wait for camera to be ready
        self.wait_for_camera_ready()

        # read the fifo size
        bytes_available = self.read_fifo_size()
        gc.collect()

        read_length = 0
        while read_length < bytes_available:
            to_read_length = min(ov2640_constants.PICBUFSIZE, bytes_available - read_length)
            print("Reading %s bytes, read %s bytes" % (to_read_length, read_length))
            data = self.ov_bus.cam_spi_read_length(to_read_length, 0x3d)
            if b'JFIF' in data:
                print(ubinascii.hexlify(data[:32], ' '))
            yield data, to_read_length
            read_length += to_read_length

        print("read %d bytes from fifo, camera said %d were available" % (read_length, bytes_available))

    def wait_for_camera_ready(self):
        cnt = 0
        mask = b'\x08'
        while True:
            res = self.ov_bus.cam_spi_read(b'\x41')
            if res[0] & mask[0]:
                break
            # print("continuing, res register %s" % ubinascii.hexlify(res))
            time.sleep_ms(10)
            cnt += 1
        # print("slept in loop %d times" % cnt)

    def read_fifo_size(self):
        b1 = self.ov_bus.cam_spi_read(b'\x44')
        b2 = self.ov_bus.cam_spi_read(b'\x43')
        b3 = self.ov_bus.cam_spi_read(b'\x42')
        val = b1[0] << 16 | b2[0] << 8 | b3[0]
        print("ov2640_capture: %d bytes in fifo" % val)
        return val

    # XXX these need some work
    def standby(self):
        # register set select
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0xff, b'\x01')
        # standby mode
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0x09, b'\x10')
        self.standby = True

    def wake(self):
        # register set select
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0xff, b'\x01')
        # standby mode
        self.ov_bus.i2c.writeto_mem(ov2640_constants.SENSORADDR, 0x09, b'\x00')
        self.standby = False


# cam driver code
# https://github.com/kanflo/esparducam/blob/master/arducam/arducam.c
# register info
# https://github.com/ArduCAM/Sensor-Regsiter-Decoder/blob/master/OV2640_JPEG_INIT.csv
