import serial


class Motor(object):
    # Serial communication stuff
    BAUDRATE = 921600
    TIMEOUT = 1

    def __init__(self, serial_port_num):
        self.serial_device = "/dev/ttyACM" + str(serial_port_num)
        # Ver si ocupa un try
        self.serial_comm = serial.Serial()
        self.serial_comm.port = self.serial_device
        self.serial_comm.baudrate = self.BAUDRATE
        self.serial_comm.timeout = self.TIMEOUT
        self.serial_comm.open()

    def serial_read(self):
        # LOGICA DE PARSEO #
        msg = None
        return msg

    def serial_write(self):
        # REVISAR QUE EL MENSAJE ES VALIDO #
        return None
