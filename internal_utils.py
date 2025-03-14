from typing import MutableSequence
from enum import Enum
from typing import NamedTuple

# Constants
## Stop-and-wait protocol config
BAUDRATE = 115200
BLE_TIMEOUT = 0.165
BLE_WAIT_TIMEOUT = 0.01
INITIAL_SEQ_NUM = 0
MAX_SEQ_NUM = 65535
MAX_RETRANSMITS = 10
TRANSMIT_DELAY = 0.015
BEETLE_KEEP_ALIVE_TIMEOUT = (MAX_RETRANSMITS * 2 + 1) * BLE_TIMEOUT
## Packet config
BITS_PER_BYTE = 8
ERROR_VALUE = -1
LOWER_4BITS_MASK = 0x0f
### Timeout value below should match the delay() value for sending packets on the Beetle
PACKET_SIZE = 20
PACKET_DATA_SIZE = 16
PACKET_TYPE_ID_LENGTH = 4
PACKET_FORMAT = "=BH16sB"
## IMU constants
ACC_LSB_SCALE = 16384.0
GYRO_LSB_SCALE = 131.0
IMU_DIMENSION = 3
## Game state constants
GAME_STATE_QUEUE_TIMEOUT = 1
INVALID_HP = 255

BLUNO_MAC_ADDR_LIST = [
    "f4:b8:5e:42:67:2b",
    "F4:B8:5E:42:6D:75",
    "F4:B8:5E:42:67:6E",
    # Glove
    "F4:B8:5E:42:61:62",
    # Gun
    "D0:39:72:DF:CA:F2",
    # Vest
    "F4:B8:5E:42:6D:0E",
    # Extra 1
    "B4:99:4C:89:1B:FD",
    # Extra 2
    "B4:99:4C:89:18:1D"
]

class BEETLE_MAC_ADDR(Enum):
    # Below must be player 1 IMU(glove) Beetle
    P1_GLOVE = "50:65:83:77:5D:6B"
    # Below must be player 1 IMU(ankle) Beetle
    P1_ANKLE = "34:08:E1:2A:29:19"
    # Below must be player 1 gun Beetle
    P1_GUN = "34:08:E1:2A:24:B7"
    # Below must be player 1 vest Beetle
    P1_VEST = ""
    # Below must be player 2 IMU(glove) Beetle
    P2_GLOVE = ""
    # Below must be player 2 IMU(ankle) Beetle
    P2_ANKLE = ""
    # Below must be player 2 gun Beetle
    P2_GUN = ""
    # Below must be player 2 vest Beetle
    P2_VEST = "34:08:E1:2A:1C:00"
    # Extra Beetle
    EXTRA = ""

BEETLE_MAC_ADDR_MAP = {
    # Below must be player 1 IMU(glove) Beetle
    "50:65:83:77:5D:6B": 1,
    # Below must be player 1 IMU(ankle) Beetle
    "34:08:E1:2A:29:19": 1,
    # Below must be player 1 gun Beetle
    "34:08:E1:2A:24:B7": 1,
    # Below must be player 1 vest Beetle
    "": 1,

    # Below must be player 2 IMU(glove) Beetle
    "": 2,
    # Below must be player 2 IMU(ankle) Beetle
    "": 2,
    # Below must be player 2 gun Beetle
    "": 2,
    # Below must be player 2 vest Beetle
    "34:08:E1:2A:1C:00": 2,

    # Extra 1
    "": 1,
    # Extra 2
#    "": 2,
}

## Bluno Beetle BLE GATT UUIDs
GATT_SERIAL_SERVICE_UUID = "0000dfb0-0000-1000-8000-00805f9b34fb"
GATT_SERIAL_CHARACTERISTIC_UUID = "0000dfb1-0000-1000-8000-00805f9b34fb"
GATT_MODEL_NUMBER_CHARACTERISTIC_UUID = "00002a24-0000-1000-8000-00805f9b34fb"
GATT_COMMAND_CHARACTERISTIC_UUID = "0000dfb2-0000-1000-8000-00805f9b34fb"

## Bluno Beetle BLE GATT Handles
GATT_COMMAND_CHARACTERISTIC_HANDLE = 40
GATT_MODEL_NUMBER_CHARACTERISTIC_HANDLE = 20
GATT_SERIAL_CHARACTERISTIC_HANDLE = 37

## Bluno Beetle GATT setup
BLUNO_GATT_PASSWORD = "AT+PASSWOR=DFRobot\r\n"
BAUDRATE_SETUP = "AT+CURRUART={}\r\n".format(BAUDRATE)

class bcolors:
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class BlePacket(NamedTuple):
    metadata: int
    seq_num: int
    data: bytearray
    crc: int

# Packet Type ID
class BlePacketType(Enum):
    HELLO = 0
    ACK = 1
    NACK = 2
    IMU = 3
    IR_RECV = 4
    IR_TRANS = 5
    GAME_STAT = 6
    GAME_ACTION = 7
    KEEP_ALIVE = 8
    INFO = 9

class HandshakeStatus(Enum):
    # Laptop has not sent HELLO packet, will send HELLO
    HELLO = 0
    # Laptop has sent HELLO packet, waiting for ACK from Beetle
    ACK = 1 
    # Laptop received ACK from Beetle, will send SYN+ACK packet to Beetle
    SYN = 2
    # Handshake complete
    COMPLETE = 3

class GunPacket(NamedTuple):
    beetle_mac: str
    gunBoolean: bool

class ImuPacket(NamedTuple):
    beetle_mac: str
    accel: MutableSequence[float]
    gyro: MutableSequence[float]

class VestPacket(NamedTuple):
    beetle_mac: str
    vestBoolean: bool

class GunUpdatePacket(NamedTuple):
    player_id: int
    bullets: int

class VestUpdatePacket(NamedTuple):
    player_id: int
    is_hit: bool
    player_hp: int

# Public functions
def get_player_id_for(beetle_mac_addr):
    return BEETLE_MAC_ADDR_MAP[beetle_mac_addr]

def is_metadata_byte(given_byte):
    packet_type = metadata_to_packet_type(given_byte)
    return packet_type <= BlePacketType.INFO.value and packet_type >= BlePacketType.HELLO.value

def metadata_to_packet_type(metadata):
    return metadata & LOWER_4BITS_MASK
