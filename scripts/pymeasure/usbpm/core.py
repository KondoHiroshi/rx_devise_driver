import serial

class usbpm_driver(object):
    rtimeout = 0.1
    wtimeout = 0.1

    def __init__(self, port):
        self._pm = self.serial_open(port)
        return

    def serial_open(self, port):
        _pm = serial.Serial(port, timeout = self.rtimeout, write_timeout = self.wtimeout)
        return _pm

    def send(self, cmd):
        self._pm.write(cmd)
        return

    def read(self, byte=100):
        res = self._pm.read(byte)
        return res
