#ifndef GUN_HPP
#define GUN_HPP


#include <IRremote.hpp>
#include <Wire.h>
#include "packet.hpp"

// BLE Constants
#define BAUDRATE 115200

// Function prototypes
BlePacket createGunPacket(bool mIsFired);
void gunSetup();
void fireGun();
byte getButtonState();
bool getIsFired();
void reload();
void updateDisplay();
void send38kHzBurst(int durationMs);
void sendShot();

// // Function to populate packet data
// void getPacketDataFor(bool mIsFired, byte packetData[PACKET_DATA_SIZE]) {
//   packetData[0] = mIsFired ? 1 : 0;
// }

#endif // GUN_HPP
