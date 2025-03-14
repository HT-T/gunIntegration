#ifndef GUN_HPP
#define GUN_HPP

#include <TimerFreeTone.h>
#include <IRremote.hpp>
#include <Wire.h>
#include "packet.hpp"

// BLE Constants
#define BAUDRATE 115200
#define ACTION_INTERVAL 600

// IR and Button Constants
#define BUTTON_PIN 4
#define BUTTON_DEBOUNCE_DELAY 5
#define LED_PIN LED_BUILTIN
#define IR_COMMAND 0x34
#define IR_TRN_PIN 5
#define BUZZER_PIN 2

// Gun Constants
#define GUN_MAGAZINE_SIZE 6
#define GUN_MAGAZINE_EMPTY_BUZZER_FREQ 350
#define GUN_MAGAZINE_EMPTY_BUZZER_DURATION 200
#define GUNFIRE_BUZZER_FREQ 1300
#define GUNFIRE_BUZZER_DURATION 100
#define RELOAD_BUZZER_FREQ 600
#define RELOAD_BUZZER_DURATION 150
#define RELOAD_TIME 500  // Reload time in ms

// Player-specific macros
#define PLAYER_ID 1
#define IR_ADDRESS_PLAYER_1 0xABCD
#define IR_ADDRESS_PLAYER_2 0x1234
#define GET_OUR_IR_ADDRESS() ((PLAYER_ID == 1) ? IR_ADDRESS_PLAYER_1 : IR_ADDRESS_PLAYER_2)

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

// Function to populate packet data
void getPacketDataFor(bool mIsFired, byte packetData[PACKET_DATA_SIZE]) {
  packetData[0] = mIsFired ? 1 : 0;
}

#endif // GUN_HPP
