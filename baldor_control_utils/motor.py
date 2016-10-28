import serial


class Motor(object):
    # Serial communication stuff
    BAUDRATE = 921600
    TIMEOUT = 1
    RETRY = 3
    MSG_LENGTH = 15

    def __init__(self, serial_port_num):
        self.serial_device = "/dev/ttyACM" + str(serial_port_num)
        # Ver si ocupa un try
        self.serial_comm = serial.Serial()
        self.serial_comm.port = self.serial_device
        self.serial_comm.baudrate = self.BAUDRATE
        self.serial_comm.timeout = self.TIMEOUT
        self.serial_comm.open()

    def serial_read(self):
        msg = None
        for i in range(self.RETRY):
            try:
                if not self.serial_comm.is_open:
                    self.serial_comm.open()
                msg = self.serial_comm.read(self.MSG_LENGTH)  # FALTA NUMERO DE BYTES
            except serial.SerialException:
                continue
            break
        return msg

    def serial_write(self, msg):
        # REVISAR QUE EL MENSAJE ES VALIDO #
        for i in range(self.RETRY):
            try:                
                if not self.serial_comm.is_open:
                    self.serial_comm.open()
                msg = self.serial_comm.write(msg+"\n\r")  # FALTA NUMERO DE BYTES
            except serial.SerialException:
                continue
            break
        return None
