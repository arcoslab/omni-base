import serial


class Motor(object):
    # Serial communication stuff
    BAUDRATE = 921600
    TIMEOUT = 1

    def __init__(self, serial_port_num):
        self.serial_device = "/dev/ttyACM" + str(serial_port_num)
        # Ver si ocupa un try
        self.serial_comm = serial.Serial(
            self.serial_device, baudrate=self.BAUDRATE, timeout=self.TIMEOUT)
        self.serial_write()
        msg = self.serial_read()
        self.motor_pos = msg['motor_pos']

    def serial_read(self):
        # LOGICA DE PARSEO #
        msg = None
        return msg

    def serial_write(self):
        # REVISAR QUE EL MENSAJE ES VALIDO #
        return None
