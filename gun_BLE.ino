#include "gun.hpp"
#include <Wire.h>

/* Internal comms */
void processAllIncomingPackets();
void processIncomingPacket();
void retransmitLastPacket();

#define TRIGGER_PIN 2
#define IR_LED_PIN 3

// Game constants
#define RELOAD_TIME 500
#define MAX_BULLETS 6
#define GUN_MAGAZINE_SIZE 6
#define BUTTON_DEBOUNCE_DELAY 10

// Global variables - minimized types and count
uint8_t bulletCount = GUN_MAGAZINE_SIZE;
uint8_t gunFlags; // Bit flags: bit 0 = isReloading, bit 1 = lastTrigger, bit 2 = firstRun
uint16_t lastReloadTime;

// Inline bit flag handlers to replace multiple boolean variables
inline bool isReloading() { return gunFlags & 0x01; }
inline void setReloading(bool state) {
  if (state) gunFlags |= 0x01;
  else gunFlags &= ~0x01;
}
inline bool getLastTrigger() { return gunFlags & 0x02; }
inline void setLastTrigger(bool state) {
  if (state) gunFlags |= 0x02;
  else gunFlags &= ~0x02;
}
inline bool isFirstRun() { return gunFlags & 0x04; }
inline void setFirstRun(bool state) {
  if (state) gunFlags |= 0x04;
  else gunFlags &= ~0x04;
}

HandshakeStatus handshakeStatus = STAT_NONE;
// Zero-initialise lastSentPacket
BlePacket lastSentPacket = {};
unsigned long lastSentPacketTime = 0;
uint16_t receiverSeqNum = INITIAL_SEQ_NUM;
uint16_t senderSeqNum = INITIAL_SEQ_NUM;
bool isWaitingForAck = false;
uint8_t numRetries = 0;
uint8_t numInvalidPacketsReceived = 0;
// Used to maintain (RETRANSMIT_DELAY) ms period of retransmissions
unsigned long lastRetransmitTime = 0;
unsigned long lastReadPacketTime = 0;
// Used to maintain keep alive interval and transmit keep alive packets periodically
unsigned long lastKeepAliveTime = 0;
// Used to maintain retransmit interval and retransmit upon timeout
unsigned long lastRawDataSentTime = 0;

/* IR Transmitter */
/* Gun state */
bool isFiring = false;
bool isFired = false;

void setup() {
  Serial.begin(BAUDRATE);

  // Setup gun-specific logic
  gunSetup();

  // Set up internal comms 
  setupBle();
}

void loop() {
  if (!hasHandshake()) {
    handshakeStatus = doHandshake();
  }
  unsigned long currentTime = millis();
  // Retransmit last sent packet on timeout
  if (isWaitingForAck && (currentTime - lastRetransmitTime) >= RETRANSMIT_DELAY
      && (currentTime - lastRawDataSentTime) >= BLE_TIMEOUT) { 
    // Maintain at least RETRANSMIT_DELAY millisecs in between consecutive retransmits
    if (numRetries < MAX_RETRANSMITS) {
      // Retransmit only if numRetries less than max limit
      retransmitLastPacket();
      numRetries += 1;
    } else {
      // Clear serial input/output buffers to restart transmission from clean state
      clearSerialInputBuffer();
      Serial.flush();
      // Laptop might have disconnected, re-enter handshake
      handshakeStatus = STAT_NONE;
      numRetries = 0;
    }
  } else if (!isWaitingForAck && (currentTime - lastSentPacketTime) >= TRANSMIT_DELAY
      && hasRawData()) { // Send raw data packets(if any)
    // Only send new packet if previous packet has been ACK-ed and there's new sensor data to send
    //   but maintain TRANSMIT_DELAY millisecs in between sensor data packet transmissions
    // Read sensor data and generate a BlePacket encapsulating that data
    BlePacket mRawDataPacket = createRawDataPacket();
    // Send updated sensor data to laptop
    sendPacket(mRawDataPacket);
    // Update last sent packet to latest sensor data packet
    lastSentPacket = mRawDataPacket;
    // Update last raw data packet sent time to maintain transmit delay
    lastSentPacketTime = millis();
    // Update last raw data packet sent time to track timeout
    lastRawDataSentTime = lastSentPacketTime;
    isWaitingForAck = true;
  } else if (!isWaitingForAck && 
      (currentTime - lastRawDataSentTime) >= KEEP_ALIVE_INTERVAL &&
      (currentTime - lastKeepAliveTime) >= KEEP_ALIVE_INTERVAL &&
      (currentTime - lastSentPacketTime) >= TRANSMIT_DELAY) {
    // Keep alive interval has passed since the last sensor/keep alive packet transmission but no sensor data is available to transmit
    // -> Send keep alive packet periodically when no sensor packet is transmitted so laptop knows Beetle is responding
    BlePacket keepAlivePacket = createKeepAlivePacket(senderSeqNum);
    sendPacket(keepAlivePacket);
    // Update last raw data packet sent time to maintain transmit delay
    lastSentPacketTime = millis();
    // Update lastSentPacketTime to support periodic keep alive packet transmission
    lastKeepAliveTime = lastSentPacketTime;
    // Don't require ACK for keep alive packets
  }
  // Always process incoming packets regardless of what sender logic does
  if ((millis() - lastReadPacketTime) >= READ_PACKET_DELAY) { // Handle incoming packets
    // Received some bytes from laptop, process them wwhile maintaining at least READ_PACKET_DELAY
    //   in between reading of 2 consecutive packets 
    processAllIncomingPackets();
  }
  
  // Handle reloading
  // if (isReloading()) {
  //   if ((millis() & 0xFFFF) - lastReloadTime >= RELOAD_TIME) {
  //     bulletCount = MAX_BULLETS;
  //     setReloading(false);
  //   }
  // }
}

HandshakeStatus doHandshake() {
  unsigned long mLastPacketSentTime = millis();
  BlePacket mLastSentPacket;
  byte mSeqNum = INITIAL_SEQ_NUM;
  bool mIsWaitingForAck = false;
  uint8_t mNumInvalidPacketsReceived = 0;
  while (handshakeStatus != STAT_SYN) {
    // No packet sent yet or not yet timed out or last packet sent is invalid
    switch (handshakeStatus) {
      case HandshakeStatus::STAT_NONE:
        {
          // Whether at least 1 HELLO packet has been received
          bool mHasHello = false;
          int mNumBytesAvailable = Serial.available();
          // Process all HELLO packets received so far at once
          // -> This should minimise cycling between NONE and HELLO handshake states
          //      when laptop transmits multiple HELLO packets
          while (mNumBytesAvailable >= PACKET_SIZE) {
            // TODO: Add read packet delay like in main loop()
            // At least 1 complete packet in serial input buffer, read it
            BlePacket mReceivedPacket = readPacket();
            // Decrease the number of bytes left to read from serial input
            mNumBytesAvailable -= PACKET_SIZE;
            if (!isPacketValid(mReceivedPacket)) {
              // TODO: Add retransmit delay like in main loop()
              mNumInvalidPacketsReceived += 1;
              if (mNumInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
                // Clear serial input buffer as an attempt to recover from unusual amount of packet corruption
                clearSerialInputBuffer();
                mNumInvalidPacketsReceived = 0;
                mSeqNum = INITIAL_SEQ_NUM;
              }
              BlePacket mNackPacket;
              // Create NACK packet indicating that packet received is invalid or has wrong seq num
              createNackPacket(mNackPacket, mSeqNum, "Invalid/seqNum");
              // Notify the laptop by transmitting the NACK packet
              sendPacket(mNackPacket);
            } else if (getPacketTypeOf(mReceivedPacket) == PacketType::HELLO) {
              // Reset invalid packet count when a valid packet is received
              mNumInvalidPacketsReceived = 0;
              if (mReceivedPacket.seqNum != mSeqNum) {
                // Drop packet if seq num does not match
                // TODO: Consider if there's a proper way to handle this
                continue;
              }
              // Indicate that at least 1 valid HELLO packet has been received
              mHasHello = true;
            }
            // Drop all non-HELLO packets received
            // Continue processing the other remaining HELLO packets in serial input
          }
          if (mHasHello) {
            // Progress to the next handshake stage as long as at least 1 HELLO packet is received
            handshakeStatus = STAT_HELLO;
          }
          // Continue waiting for HELLO packet from laptop in the next iteration of while() if none is received so far
          break;
        }
      case HandshakeStatus::STAT_HELLO:
        {
          unsigned long mTransmitPeriod = millis() - mLastPacketSentTime;
          if (mTransmitPeriod < TRANSMIT_DELAY) {
            // Maintain at least (TRANSMIT_DELAY) ms delay between transmissions to avoid overwhelming the Beetle
            delay(TRANSMIT_DELAY - mTransmitPeriod);
          }
          BlePacket ackPacket;
          createHandshakeAckPacket(ackPacket, mSeqNum);  
          sendPacket(ackPacket);
          mLastSentPacket = ackPacket;
          mLastPacketSentTime = millis();
          mSeqNum += 1;
          handshakeStatus = HandshakeStatus::STAT_ACK;
          mIsWaitingForAck = true;
          break;
        }
      case HandshakeStatus::STAT_ACK:
        {
          unsigned long mCurrentTime = millis();
          if (mIsWaitingForAck && (mCurrentTime - mLastPacketSentTime) >= BLE_TIMEOUT) {
            handshakeStatus = STAT_HELLO;
            mSeqNum = INITIAL_SEQ_NUM;
            // Invalidate mLastSentPacket to prevent retransmission in any edge case
            mLastSentPacket.metadata = PLACEHOLDER_METADATA;
            // Reset mIsWaitingForAck as well to fully reset transmission state
            mIsWaitingForAck = false;
            // TODO: Consider if there's a need to clear serial input buffer here(after retransmitting)
            continue;
          }
          if (Serial.available() < PACKET_SIZE) {
            // Skip this iteration since we haven't received a full 20-byte packet
            continue;
          }
          // TODO: Add read packet delay like in main loop()
          BlePacket receivedPacket = readPacket();
          if (!isPacketValid(receivedPacket)) {
            // TODO: Add retransmit delay like in main loop()
            mNumInvalidPacketsReceived += 1;
            if (mNumInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
              // Clear serial input buffer as an attempt to recover from unusual amount of packet corruption
              clearSerialInputBuffer();
              mNumInvalidPacketsReceived = 0;
              mSeqNum = INITIAL_SEQ_NUM;
            }
            BlePacket nackPacket;
            createNackPacket(nackPacket, mSeqNum, "Corrupted");
            sendPacket(nackPacket);
          } else { // Packet received is valid
            // Reset invalid packet count when a valid packet is received
            mNumInvalidPacketsReceived = 0;
            if (getPacketTypeOf(receivedPacket) == PacketType::ACK) {
              if (receivedPacket.seqNum > mSeqNum) {
                // TODO: Add retransmit delay like in main loop()
                BlePacket nackPacket;
                // Use existing seqNum for NACK packet to indicate current packet is not received
                createNackPacket(nackPacket, mSeqNum, "Over seqNum");
                sendPacket(nackPacket);
                continue;
              }
              if (receivedPacket.seqNum < mSeqNum) {
                // Likely a delayed ACK packet, drop it
                continue;
              }
              // TODO: Handle seq num update if laptop seq num != beetle seq num
              handshakeStatus = HandshakeStatus::STAT_SYN;
              mSeqNum += 1;
              mIsWaitingForAck = false;
              // Drop duplicate SYN+ACK packets received from laptop so transmission logic 
              //   in loop() doesn't process leftover SYN+ACK packets from handshake
              clearSerialInputBuffer(); // TODO: Replace this with while loop that reads until non-handshake packet is received or serial input is empty
              // Break switch block since handshake process is complete
              //   This would also terminate the outer while() loop since handshake status is now STAT_SYN
              break;
            } else if (getPacketTypeOf(receivedPacket) == PacketType::HELLO &&
                (mCurrentTime - mLastPacketSentTime) >= BLE_TIMEOUT) {
              // Return to HELLO state only if we sent ACK a sufficiently long time ago(handshake has restarted or timeout occurred)
              handshakeStatus = STAT_HELLO;
              mSeqNum = INITIAL_SEQ_NUM;
              // Drop the HELLO packet if we just sent an ACK to avoid cycling between HELLO and ACK states
              //   This should clear the Serial input buffer of duplicate HELLO packets
            } else if (getPacketTypeOf(receivedPacket) == PacketType::NACK &&
                receivedPacket.seqNum == (mSeqNum - 1) && isPacketValid(mLastSentPacket)) {
              /* TODO: Consider if this block is ever entered, since we only accept NACK
                for our ACK to a HELLO, which means receivedPacket.seqNum = 0 and mSeqNum = 1 */
              // TODO: Add retransmit delay like in main loop()
              sendPacket(mLastSentPacket);
              mIsWaitingForAck = true;
            }
          }
        }
    }
  }
  return handshakeStatus;
}

/**
 * Setup for the BLE internal communications-related logic and variables
 */
void setupBle() {
  // Clear the serial output buffer
  //   WARNING: This sends out all existing data in the output buffer over BLE though
  Serial.flush();

  // Clear the serial input buffer
  clearSerialInputBuffer();

  /* Initialise lastSentPacket with invalid metadata
    to ensure it's detected as corrupted if ever
    sent without assigning actual (valid) packet */
  lastSentPacket.metadata = PLACEHOLDER_METADATA;
}

BlePacket createGunPacket(bool mIsFired) {
  BlePacket gunPacket = {};
  byte packetData[PACKET_DATA_SIZE] = {};
  getPacketDataFor(mIsFired, packetData);
  createPacket(gunPacket, PacketType::IR_TRANS, senderSeqNum, packetData);
  return gunPacket;
}

void createHandshakeAckPacket(BlePacket &ackPacket, uint16_t givenSeqNum) {
  byte packetData[PACKET_DATA_SIZE] = {};
  uint16_t seqNumToSyn = senderSeqNum;
  if (isWaitingForAck && isPacketValid(lastSentPacket)) {
    seqNumToSyn = lastSentPacket.seqNum;
  }
  packetData[0] = (byte) seqNumToSyn;
  packetData[1] = (byte) (seqNumToSyn >> BITS_PER_BYTE);
  createPacket(ackPacket, PacketType::ACK, givenSeqNum, packetData);
}

BlePacket createRawDataPacket() {
  return createGunPacket(isFired);
}

uint8_t getBulletCountFrom(const BlePacket &gamePacket) {
  return gamePacket.data[0];
}

/*
 * Update internal variables based on the new game state received
 */
// void handleGamePacket(const BlePacket &gamePacket) {
//   uint8_t newBulletCount = getBulletCountFrom(gamePacket);
//   if (bulletCount == 0 && newBulletCount > bulletCount) {
//     bulletCount = newBulletCount;
//     reload();
//   } else if (newBulletCount >= 0 && newBulletCount <= GUN_MAGAZINE_SIZE) {
//     bulletCount = newBulletCount;
//   }
// }

void handleGamePacket(const BlePacket &gamePacket) {
  uint8_t newBulletCount = getBulletCountFrom(gamePacket);
  bulletCount = newBulletCount;  // Directly update bullet count
  setReloading(false);           // Ensure not in reloading state
}



bool hasHandshake() {
  return handshakeStatus == HandshakeStatus::STAT_SYN;
}

// Optimized IR burst function
void send38kHzBurst(int durationMs) {
  uint16_t endTime = millis() + durationMs;
  while(millis() < endTime) {
      digitalWrite(IR_LED_PIN, HIGH);
      delayMicroseconds(13);
      digitalWrite(IR_LED_PIN, LOW);
      delayMicroseconds(13);
  }
}

// Optimized shot sequence - direct writes
void sendShot() {
  send38kHzBurst(50);  // Header
  delay(25);
  send38kHzBurst(10);  // Player ID
  delay(25);
  send38kHzBurst(7);   // Bullet
  delay(25);
  send38kHzBurst(20);  // End marker
}

/**
 * Checks whether the connected sensors of this Beetle has raw data to send to laptop.
 * Returns true when there's raw data to transmit (gun fire)
 */
bool hasRawData() {
  // Check if the trigger is pressed
  bool currentTrigger = digitalRead(TRIGGER_PIN);
  
  if (currentTrigger == LOW && getLastTrigger() == HIGH) {
    delay(BUTTON_DEBOUNCE_DELAY); // Debounce
    
    if (digitalRead(TRIGGER_PIN) == LOW) { // Still pressed after debounce
      if (bulletCount > 0 && !isReloading()) {
        // Send IR signal
        sendShot();
        bulletCount--;
        // Set flag that gun was fired
        isFired = true;
        
        // Save trigger state
        setLastTrigger(currentTrigger);
        
        // Tell BLE there's data to send
        return true;
      }
    }
  }
  
  // Save trigger state
  setLastTrigger(currentTrigger);
  
  // No need to send data
  isFired = false;
  return false;
}

void processGivenPacket(const BlePacket &packet) {
  char givenPacketType = getPacketTypeOf(packet);
  switch (givenPacketType) {
    case PacketType::HELLO:
      handshakeStatus = STAT_HELLO;
      break;
    case PacketType::ACK:
      if (!isWaitingForAck) {
        // Not expecting an ACK, so this ACK is likely delayed and we drop it
        return;
      }
      // Have been waiting for an ACK and we received it
      if (packet.seqNum > senderSeqNum) {
        BlePacket nackPacket;
        createNackPacket(nackPacket, senderSeqNum, "seqNum too high");
        // Inform laptop about seq num mismatch by sending a NACK with our current seq num
        sendPacket(nackPacket);
        return;
      } else if (packet.seqNum < senderSeqNum) {
        // If packet.seqNum < senderSeqNum, it's (likely) a delayed ACK packet and we ignore it
        return;
      }
      // Valid ACK received, so stop waiting for incoming ACK
      isWaitingForAck = false;
      // Increment senderSeqNum upon every ACK
      senderSeqNum += 1;
      isFired = false;
      numRetries = 0;
      break;
    case PacketType::NACK:
      if (!isWaitingForAck) {
        // Didn't send a packet, there's nothing to NACK
        // Likely a delayed packet so we just drop it
        return;
      }
      // Sent a packet but received a NACK, attempt to retransmit
      if (packet.seqNum == senderSeqNum) {
        if (isPacketValid(lastSentPacket) && getPacketTypeOf(lastSentPacket) != PacketType::NACK) {
          // Only retransmit if packet is valid
          sendPacket(lastSentPacket);
        }
        // No else{}: Don't retransmit a corrupted packet or another NACK packet
      }
      // If packet.seqNum < senderSeqNum, NACK packet is likely delayed and we drop it
      break;
    // case GAME_STAT:
    //   {
    //     uint16_t seqNumToAck = receiverSeqNum;
    //     bool shouldHandlePacket = false;
    //     if (receiverSeqNum == packet.seqNum) {
    //       shouldHandlePacket = true;
    //       receiverSeqNum += 1;
    //     } else if (receiverSeqNum > packet.seqNum) {
    //       /* If receiverSeqNum > packet.seqNum, I incremented receiverSeqNum after sending ACK 
    //           but sender did not receive ACK and thus retransmitted packet
    //         */
    //       // ACK the packet but don't decrement my sequence number
    //       seqNumToAck = packet.seqNum;
    //       // Don't process the same packet again
    //     }
    //     // TODO: Consider what to do if receiverSeqNum < packet.seqNum?
    //     BlePacket ackPacket;
    //     createAckPacket(ackPacket, seqNumToAck);
    //     sendPacket(ackPacket);
    //     if (numInvalidPacketsReceived > 0) {
    //       numInvalidPacketsReceived = 0;
    //     }
    //     if (shouldHandlePacket) {
    //       // Process the packet to handle specific game logic(e.g. updating Beetle's internal game state)
    //       handleGamePacket(packet);
    //     }
    //     break;
    //   }
    case GAME_STAT:
  {
    // Always process game packets regardless of sequence
    handleGamePacket(packet);
    
    // Still maintain sequence number protocol for ACKs
    uint16_t seqNumToAck = packet.seqNum;
    
    // Update receiver sequence if needed
    if (packet.seqNum >= receiverSeqNum) {
      receiverSeqNum = packet.seqNum + 1;
    }
    
    // Send ACK
    BlePacket ackPacket;
    createAckPacket(ackPacket, seqNumToAck);
    sendPacket(ackPacket);
    break;
    }
    case INVALID_PACKET_ID:
    default:
      // All other packet types are unsupported, inform sender that packet is rejected
      BlePacket nackPacket;
      createNackPacket(nackPacket, receiverSeqNum, "Invalid type");
      sendPacket(nackPacket);
  } // switch (receivedPacketType)
}

void processAllIncomingPackets() {
  int numBytesAvailable = Serial.available();
  // Read as many packets as are available in the serial input buffer at the moment processAllIncomingPackets() is called
  while (numBytesAvailable >= PACKET_SIZE) {
    // Complete packet received, read packet bytes from receive buffer as BlePacket
    BlePacket receivedPacket = readPacket();
    // Update lastReadPackeTime to maintain read packet delay in loop()
    lastReadPacketTime = millis();
    // Read PACKET_SIZE number of bytes, decrease number of bytes available accordingly
    numBytesAvailable -= PACKET_SIZE;
    if (!isPacketValid(receivedPacket)) {
      numInvalidPacketsReceived += 1;
      if (numInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
        // Clear serial input buffer as an attempt to recover from unusual amount of packet corruption
        clearSerialInputBuffer();
        numInvalidPacketsReceived = 0;
        // Cleared all received packets, nothing left to parse
        numBytesAvailable = 0;
      }
      BlePacket nackPacket;
      createNackPacket(nackPacket, receiverSeqNum, "Corrupted");
      // Received invalid packet, request retransmit with NACK
      sendPacket(nackPacket);
      continue;
    } else {
      // Valid packet received, reset invalid packet count
      numInvalidPacketsReceived = 0;
      // Process valid packet
      processGivenPacket(receivedPacket);
    }
  }
}

void processIncomingPacket() {
  if (Serial.available() < PACKET_DATA_SIZE) {
    // Don't read from serial input buffer unless 1 complete packet is received
    return;
  }
  // Complete 20-byte packet received, read 20 bytes from receive buffer as packet
  BlePacket receivedPacket = readPacket();
  // Update lastReadPackeTime to maintain read packet delay in loop()
  lastReadPacketTime = millis();
  if (!isPacketValid(receivedPacket)) {
    numInvalidPacketsReceived += 1;
    if (numInvalidPacketsReceived == MAX_INVALID_PACKETS_RECEIVED) {
      clearSerialInputBuffer();
      delay(BLE_TIMEOUT);
      numInvalidPacketsReceived = 0;
      return;
    }
    BlePacket nackPacket;
    createNackPacket(nackPacket, receiverSeqNum, "Corrupted");
    // Received invalid packet, request retransmit with NACK
    sendPacket(nackPacket);
  } else {
    if (numInvalidPacketsReceived > 0) {
      numInvalidPacketsReceived = 0;
    }
    processGivenPacket(receivedPacket);
  }
}

int readIntoRecvBuffer(MyQueue<byte> &mRecvBuffer) {
  int numOfBytesRead = 0;
  while (Serial.available() > 0) {
    byte nextByte = (byte) Serial.read();
    if (isHeadByte(nextByte) || !mRecvBuffer.isEmpty()) {
      mRecvBuffer.push_back(nextByte);
      numOfBytesRead += 1;
    }
  }
  return numOfBytesRead;
}

void retransmitLastPacket() {
  if (isPacketValid(lastSentPacket)) {
    sendPacket(lastSentPacket);
    // Update last retransmit time to maintain retransmit delay
    lastRetransmitTime = millis();
    // Update last sent packet time to maintain transmit delay independently from retransmit delay
    lastSentPacketTime = lastRetransmitTime;
    // Update sent time and wait for ACK again
    lastRawDataSentTime = lastRetransmitTime;
  } else {
    isWaitingForAck = false;
  }
}

/* IR Transmitter */
void gunSetup() {
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, LOW);
  
  bulletCount = MAX_BULLETS;
  gunFlags = 0x04; // Set firstRun flag only
}

void reload() {
  // Set reloading state
  setReloading(true);
  lastReloadTime = millis() & 0xFFFF; // Keep only lower 16 bits
}

void fireGun() {
  // This function has been replaced by the improved hasRawData() implementation
  // It's kept for compatibility with other parts of the code
  isFired = true;
}