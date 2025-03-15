import traceback
import bluepy
from bluepy.btle import DefaultDelegate

from internal_utils import BAUDRATE_SETUP, BLUNO_GATT_PASSWORD, GATT_SERIAL_CHARACTERISTIC_HANDLE, LOWER_4BITS_MASK, PACKET_SIZE, BlePacketType, bcolors, is_metadata_byte

# Delegate
class BlePacketDelegate(DefaultDelegate):
    def __init__(self, dataBuffer):
        super().__init__()
        self.dataBuffer = dataBuffer
        self.fragmented_packet_count = 0

    # Bluno Beetle uses cHandle 37
    def handleNotification(self, cHandle, data):
        try:
            if cHandle == GATT_SERIAL_CHARACTERISTIC_HANDLE:
                    # Add incoming bytes to receive buffer
                if len(data) < PACKET_SIZE:
                    print("{}Fragmented packet received{}".format(bcolors.BRIGHT_YELLOW, bcolors.ENDC))
                    self.fragmented_packet_count += 1
                for dataByte in data:
                    if is_metadata_byte(dataByte) or len(self.dataBuffer) > 0:
                        self.dataBuffer.append(dataByte)
                    else:
                        print("Dropping byte {}".format(dataByte))
            else:
                print(f"""{bcolors.BRIGHT_YELLOW}Bytes received from unknown handle {cHandle}, dropping{bcolors.ENDC}""")
        except Exception as exc:
            traceback.print_exception(exc)

    def get_fragmented_packet_count(self):
        return self.fragmented_packet_count
    
    def isHeaderByte(self, dataByte):
        packet_id = dataByte & LOWER_4BITS_MASK
        return packet_id <= BlePacketType.GAME_STAT.value and packet_id >= BlePacketType.HELLO.value
    
class NewBlePacketDelegate(BlePacketDelegate):
    def __init__(self, dataBuffer, command_char, model_num_char, serial_char):
        super().__init__(dataBuffer)
        self.command_char: bluepy.btle.Characteristic = command_char
        self.model_num_char: bluepy.btle.Characteristic = model_num_char
        self.serial_char: bluepy.btle.Characteristic = serial_char
        self.has_bluno_auth = False

    # Bluno Beetle uses cHandle 37
    def handleNotification(self, cHandle, data):
        if cHandle == self.serial_char.getHandle():
            try:
                # Add incoming bytes to receive buffer
                if len(data) < PACKET_SIZE:
                    print("{}Fragmented packet received{}".format(bcolors.BRIGHT_YELLOW, bcolors.ENDC))
                    self.fragmented_packet_count += 1
                for dataByte in data:
                    if is_metadata_byte(dataByte) or len(self.dataBuffer) > 0:
                        self.dataBuffer.append(dataByte)
                    else:
                        print("Dropping byte {}".format(dataByte))
            except Exception as exc:
                traceback.print_exception(exc)
        elif cHandle == self.model_num_char.getHandle():
            self.do_bluno_auth()
        else:
            print(f"""Data received for unknown GATT handle {cHandle}""")

    def get_bluno_auth(self):
        return self.has_bluno_auth

    def do_bluno_auth(self):
        if self.has_bluno_auth:
            print(f"""WARNING: {self.serial_char.peripheral.addr} requested for authentication again""")
        self.command_char.write(bytes(BLUNO_GATT_PASSWORD, "ascii"))
        self.command_char.write(bytes(BAUDRATE_SETUP, "ascii"))
        self.has_bluno_auth = True
        self.dataBuffer.clear()
        
