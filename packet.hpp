#include <CRC8.h>

#define BAUDRATE 115200
#define BITS_PER_BYTE 8
// Given average RTT 100ms and 15ms receive/transmit delay
#define BLE_TIMEOUT 165
#define INITIAL_SEQ_NUM 0
#define INVALID_PACKET_ID -1
// Duration after last sensor packet transmission when keep alive packet is transmitted
//   Equal to MAX_RETRANSMITS * BLE_TIMEOUT
#define KEEP_ALIVE_INTERVAL 1650
#define LOWER_4BIT_MASK 0x0F
#define MAX_BUFFER_SIZE 20 // _HTCHANGED
#define MAX_INVALID_PACKETS_RECEIVED 5
#define MAX_RETRANSMITS 10
#define PACKET_SIZE 20
#define PACKET_DATA_SIZE 16
#define PLACEHOLDER_METADATA 0x0F
#define READ_PACKET_DELAY 15
#define RETRANSMIT_DELAY 25
#define TRANSMIT_DELAY 15

struct BlePacket {
	/* Start packet header */
	/* Lowest 4 bits: packet type ID, 
   * highest 4 bits: number of padding bytes */
	byte metadata;
	uint16_t seqNum;
	/* End packet header */
	/* Start packet body */
	// 16-bytes of data, e.g. accelerometer data
	byte data[PACKET_DATA_SIZE];
	/* End packet body */
	/* Start footer */
	byte crc;
	/* End footer */
};

enum HandshakeStatus {
  STAT_NONE = 0,
  STAT_HELLO = 1,
  STAT_ACK = 2,
  STAT_SYN = 3
};

enum PacketType {
  HELLO = 0,
  ACK = 1,
  NACK = 2,
  IMU = 3,
  IR_RECV = 4,
  IR_TRANS = 5,
  GAME_STAT = 6,
  GAME_ACTION = 7,
  KEEP_ALIVE = 8,
  INFO = 9
};

template <typename T> class MyQueue {
private:
  T elements[MAX_BUFFER_SIZE];
  int head;
  int tail;
  int length;

  int getLength() {
    return this->length;
  }

  void setLength(int length) {
    this->length = length;
  }

public:
  MyQueue() {
    this->length = 0;
    this->head = 0;
    this->tail = 0;
  }

  void clear() {
    this->head = 0;
    this->tail = 0;
    this->setLength(0);
  }

  T get(int index) {
    if (index < this->size()) {
      return this->elements[this->head + index];
    }
    return T();
  }

  bool push_back(T element) {
    if (isFull()) {
      return false;
    }
    this->elements[this->tail] = element;
    this->tail = (this->tail + 1) % MAX_BUFFER_SIZE;
    this->setLength(this->getLength() + 1);
    return true;
  }

  T pop_front() {
    if (isEmpty()) {
      return T();
    }
    T current = this->elements[this->head];
    this->head = (this->head + 1) % MAX_BUFFER_SIZE;
    //this->elements[this->head] = T();
    this->setLength(this->getLength() - 1);
    return current;
  }

  bool isEmpty() {
    return this->getLength() == 0;
  }

  bool isFull() {
    return this->getLength() == MAX_BUFFER_SIZE;
  }

  int size() {
    return this->getLength();
  }
};

/* Method declarations */
void createDataFrom(String givenStr, byte packetData[PACKET_DATA_SIZE]);
void createHandshakeAckPacket(BlePacket &ackPacket, uint16_t givenSeqNum);
BlePacket createRawDataPacket();
HandshakeStatus doHandshake();
uint8_t getCrcOf(const BlePacket &packet);
bool hasHandshake();
bool isHeadByte(byte currByte);
byte parsePacketTypeFrom(byte metadata);
int readIntoRecvBuffer(MyQueue<byte> &mRecvBuffer);
BlePacket sendDummyPacket();
void sendPacket(BlePacket &packetToSend);

uint8_t clearSerialInputBuffer() {
  // Count the number of bytes removed to return to the caller(if useful)
  uint8_t numBytesRemoved = 0;
  // Only clear the input buffer up to the number of bytes available at the moment clearSerialInputBuffer() is called
  int numBytesAvailable = Serial.available();
  // Keep removing data from the serial input up to (numBytesAvailable) bytes
  while (numBytesAvailable > 0) {
    // Remove the next byte from serial input and drop it
    byte nextByte = (byte) Serial.read();
    numBytesRemoved += 1;
    numBytesAvailable -= 1;
  }
  // Return the number of bytes removed from serial input buffer
  return numBytesRemoved;
}

void createPacket(BlePacket &packet, byte packetType, uint16_t givenSeqNum, byte data[PACKET_DATA_SIZE]) {
  packet.metadata = packetType;
  packet.seqNum = givenSeqNum;
  for (byte i = 0; i < PACKET_DATA_SIZE; i += 1) {
    packet.data[i] = data[i];
  }
  packet.crc = getCrcOf(packet);
}

void createAckPacket(BlePacket &ackPacket, uint16_t givenSeqNum) {
  byte packetData[PACKET_DATA_SIZE] = {};
  createDataFrom("ACK", packetData);
  createPacket(ackPacket, PacketType::ACK, givenSeqNum, packetData);
}

BlePacket createKeepAlivePacket(uint16_t givenSeqNum) {
  BlePacket keepAlivePacket = {};
  byte packetData[PACKET_DATA_SIZE] = {};
  createPacket(keepAlivePacket, PacketType::KEEP_ALIVE, givenSeqNum, packetData);
  return keepAlivePacket;
}

void createNackPacket(BlePacket &nackPacket, uint16_t givenSeqNum, String nackReason) {
  byte packetData[PACKET_DATA_SIZE] = {};
  createDataFrom(nackReason, packetData);
  createPacket(nackPacket, PacketType::NACK, givenSeqNum, packetData);
}

void createNackPacket(BlePacket &nackPacket, uint16_t givenSeqNum) {
  createNackPacket(nackPacket, givenSeqNum, "NACK");
}

void createDataFrom(String givenStr, byte packetData[PACKET_DATA_SIZE]) {
  const size_t MAX_SIZE = givenStr.length() > PACKET_DATA_SIZE ? PACKET_DATA_SIZE : givenStr.length();
  const char *stringChar = givenStr.c_str();
  for (size_t i = 0; i < MAX_SIZE; ++i) {
    packetData[i] = (byte) *(stringChar + i);
  }
  for (size_t i = MAX_SIZE; i < PACKET_DATA_SIZE; ++i) {
    packetData[i] = 0;
  }
}

bool fixPacketCrc(BlePacket &givenPacket) {
  uint8_t computedCrc = getCrcOf(givenPacket);
  if (computedCrc != givenPacket.crc) {
    givenPacket.crc = computedCrc;
    // Return true to indicate that crc value was corrupted and is now fixed
    return true;
  }
  return false;
}

void floatToData(byte packetData[PACKET_DATA_SIZE], float x1, float y1, float z1, float x2, float y2, float z2) {
  uint16_t x1s = (uint16_t) (x1 * 100);
  packetData[0] = (byte) x1s;
  packetData[1] = (byte) (x1s >> BITS_PER_BYTE);
  uint16_t y1s = (uint16_t) (y1 * 100);
  packetData[2] = (byte) y1s;
  packetData[3] = (byte) (y1s >> BITS_PER_BYTE);
  uint16_t z1s = (uint16_t) (z1 * 100);
  packetData[4] = (byte) z1s;
  packetData[5] = (byte) (z1s >> BITS_PER_BYTE);
  uint16_t x2s = (uint16_t) (x2 * 100);
  packetData[6] = (byte) x2s;
  packetData[7] = (byte) (x2s >> BITS_PER_BYTE);
  uint16_t y2s = (uint16_t) (y2 * 100);
  packetData[8] = (byte) y2s;
  packetData[9] = (byte) (y2s >> BITS_PER_BYTE);
  uint16_t z2s = (uint16_t) (z2 * 100);
  packetData[10] = (byte) z2s;
  packetData[11] = (byte) (z2s >> BITS_PER_BYTE);
  // Padding bytes
  for (size_t i = 12; i < PACKET_DATA_SIZE; i += 1) {
    packetData[i] = 0;
  }
}

/* Method definitions */
uint8_t getCrcOf(const BlePacket &packet) {
  CRC8 crcGen;
  crcGen.add((uint8_t) packet.metadata);
  crcGen.add((uint8_t) packet.seqNum);
  crcGen.add((uint8_t) (packet.seqNum >> BITS_PER_BYTE));
  for (size_t i = 0; i < PACKET_DATA_SIZE; ++i) {
    uint8_t dataByte = (uint8_t) packet.data[i];
    crcGen.add(dataByte);
  }
  uint8_t crcValue = crcGen.calc();
  return crcValue;
}

void getPacketData(byte packetData[PACKET_DATA_SIZE]) {
  for (size_t i = 0; i < PACKET_DATA_SIZE; ++i) {
    packetData[i] = (byte) Serial.read();
  }
}

void getPacketData(MyQueue<byte> &recvBuffer, byte packetData[PACKET_DATA_SIZE]) {
  for (size_t i = 0; i < PACKET_DATA_SIZE; ++i) {
    packetData[i] = recvBuffer.pop_front();
  }
}

char getPacketTypeOf(const BlePacket &packet) {
  if (!isHeadByte(packet.metadata)) {
    return INVALID_PACKET_ID;
  }
  char packetTypeId = (char) parsePacketTypeFrom(packet.metadata);
  return packetTypeId;
}

bool isHeadByte(byte currByte) {
  byte packetId = parsePacketTypeFrom(currByte);
  return packetId >= PacketType::HELLO && packetId <= PacketType::INFO;
}

// TODO: Do we want to take seqNum as parameter and check whether current seqNum makes sense vs packet's seqNum?
bool isPacketValid(BlePacket &packet) {
  if (!isHeadByte(packet.metadata)) {
    return false;
  }
  uint8_t computedCrc = getCrcOf(packet);
  if (computedCrc != packet.crc) {
    return false;
  }
  return true;
}

byte parsePacketTypeFrom(byte metadata) {
  return metadata & LOWER_4BIT_MASK;
}

BlePacket readPacket() {
  BlePacket newPacket = {};
  if (Serial.available() < PACKET_SIZE) {
    return newPacket;
  }
  newPacket.metadata = (byte) Serial.read();
  uint16_t seqNumLowByte = (uint16_t) Serial.read();
  uint16_t seqNumHighByte = (uint16_t) Serial.read();
  newPacket.seqNum = seqNumLowByte + (seqNumHighByte << BITS_PER_BYTE);
  getPacketData(newPacket.data);
  newPacket.crc = (byte) Serial.read();
  return newPacket;
}

BlePacket readPacketFrom(MyQueue<byte> &recvBuffer) {
  BlePacket newPacket;
  newPacket.metadata = (byte) recvBuffer.pop_front();
  uint16_t seqNumLowByte = (uint16_t) recvBuffer.pop_front();
  uint16_t seqNumHighByte = (uint16_t) recvBuffer.pop_front();
  newPacket.seqNum = seqNumLowByte + (seqNumHighByte << BITS_PER_BYTE);
  getPacketData(recvBuffer, newPacket.data);
  newPacket.crc = (byte) recvBuffer.pop_front();
  return newPacket;
}

void sendPacket(BlePacket &packetToSend) {
  Serial.write((byte *) &packetToSend, sizeof(packetToSend));
}
