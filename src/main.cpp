#include <FlexCAN_T4.h>
#include <SD.h>
#include <Arduino.h>
#include <math.h>
#include <iostream>
// #include <CANopen.h>
// #include "CANopen.h"

// --------------------- DEFINITIONS ---------------------
#define MAXON_AFT 0x04
#define MAXON_PORT 0x05
#define MAXON_STARBOARD 0x06

#define TEST_ADDRESS 0x0383         // CAN Address for testing purposes
#define CONTROL_SETPOINTS 0x0381    // CAN Address for control setpoints

#define CTX_PIN 23   // SPI Data Input (MOSI)
#define CRX_PIN 22   // SPI Data Input (MOSI)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;

// constants

File logFile; // File object for logging
int i = 1;
double  beginTime;
String fileName = "log1.csv";
uint8_t nodeID = MAXON_PORT;

// Position controller gains
uint32_t P = 1661459;
uint32_t I = 62667205;
uint32_t D = 16448;
uint32_t ff_v = 494;
uint32_t ff_a = 66;


// Parameters for the sinusoidal wave
const int amplitude = 30000;       // Amplitude in encoder ticks (1 revolution = 4096 ticks)
const float frequency = 0.5;      // Frequency in Hz (0.5 Hz = 2 seconds per cycle)
const int updateInterval = 10;    // Sampling period in ms (100 Hz update rate)
unsigned long lastUpdateTime = 0; // Time tracker for position updates
unsigned long lastRequestTime = 0;        // Time tracker for actual position requests
float timeElapsed = 0.0;          // Total elapsed time in seconds

int targetPosition = 0;   // Stores the target position for logging
int actualPosition = 0;   // Stores the actual position for logging (updated via CAN message)

// void logPosition(double timestamp, int32_t target, int32_t actual) {
//   logFile = SD.open(fileName.c_str(), FILE_WRITE);
//   if (logFile) {
//     logFile.print(timestamp, 3);
//     logFile.print(",");
//     logFile.print(target);
//     logFile.print(",");
//     logFile.println(actual);
//     logFile.close();
//   } else {
//     Serial.println("Error opening log file!");
//   }
// }

void logCANMessage(const CAN_message_t &msg) {
  
  logFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (logFile) {
      logFile.print(msg.id, HEX);
      logFile.print(",");
      logFile.print(msg.len);
      logFile.print(",");
      for (int i = 0; i < msg.len; i++) {
          logFile.print(msg.buf[i], HEX);
          if (i < msg.len - 1) logFile.print(" ");
      }
      logFile.println();
      logFile.close();
  } else {
      Serial.println("Error opening log file!");
  }
}

void printCANMessage(CAN_message_t message) {
    // Print the CAN message ID
    Serial.print("CAN message - ID: 0x");
    Serial.print(message.id, HEX);
    
    // Print the message length
    Serial.print(", Length: ");
    Serial.print(message.len);
    
    // Print the message data bytes
    Serial.print(", Data: ");
    for (int i = 0; i < message.len; i++) {
        Serial.print("0x");
        Serial.print(message.buf[i], HEX);
        if (i < message.len - 1) Serial.print(" ");  // Add space between bytes
    }
    
    Serial.println();  // End the line
}

void canMessageHandler(const CAN_message_t &msg) {

    if (msg.id != 0x182 && msg.id != 0x446 && msg.id != 0x381 && msg.id != 0x80) {
      Serial.print("Received ");
      printCANMessage(msg);
      logCANMessage(msg); // Save to SD card
    }
}

// Callback for receiving CAN messages
// Updated `canMessageHandler` to set `actualPosition`
// void canMessageHandler(const CAN_message_t &msg) {
//   if (msg.id == 0x580 + nodeID) { // Response to SDO request
//       uint16_t index = (msg.buf[2] << 8) | msg.buf[1];
//       uint8_t subIndex = msg.buf[3];

//       if (index == 0x6064 && subIndex == 0x00) { // Actual position
//           actualPosition = (int32_t)(msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24));
//           Serial.print("Actual Position: ");
//           Serial.println(actualPosition);
//       }
//   }
//   Serial.print("Received ");
//   printCANMessage(msg);

//   logPosition();
// }

void sendCAN(uint8_t nodeID, uint16_t index ,uint8_t subindex, uint32_t value){
  CAN_message_t msg;
  msg.id = 0x600 + nodeID;
  msg.flags.extended = 0;
  msg.len = 8;

  msg.buf[0] = 0x23;               // SDO Command: Write 4 bytes
  msg.buf[1] = index & 0xFF;       // Index low byte
  msg.buf[2] = (index >> 8) & 0xFF;// Index high byte
  msg.buf[3] = subindex;           // Subindex
  // Convert value to Little Endian format
  msg.buf[4] = value & 0xFF;       
  msg.buf[5] = (value >> 8) & 0xFF;  
  msg.buf[6] = (value >> 16) & 0xFF; 
  msg.buf[7] = (value >> 24) & 0xFF; 

  myCan.write(msg);

  Serial.print("Sent ");
  printCANMessage(msg);
}

void requestValue(uint16_t index, uint8_t subindex) {
    CAN_message_t msg;
    msg.id = 0x600 + nodeID;  // SDO Request ID (0x600 + NodeID)
    msg.flags.extended = 0;   // Standard CAN ID
    msg.len = 8;              // CANopen SDO messages are always 8 bytes

    msg.buf[0] = 0x40;        // SDO Read Request (4 bytes)
    msg.buf[1] = index & 0xFF;      // Index Low Byte
    msg.buf[2] = (index >> 8) & 0xFF; // Index High Byte
    msg.buf[3] = subindex;    // Subindex
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    myCan.write(msg);  // Send SDO request

    Serial.print("Requested ");
    printCANMessage(msg);
}

void setMotorMode(uint32_t mode) {
  sendCAN(nodeID, 0x6060, 0x00, 0x01); // A Operation Mode
  delay(100);
  sendCAN(nodeID, 0x6086, 0x00, 0x00); // B Set parameter
  delay(100);
  sendCAN(nodeID, 0x6040, 0x00, 0x0F); // C Enable Motor
  delay(100);
  sendCAN(nodeID, 0x607A, 0x00, 0x00); // D Target Position (absolute position, start immediately)
  delay(100);
  sendCAN(nodeID, 0x6040, 0x00, 0x3F); // E Start movement
  delay(100);
}

void sendTargetPosition(uint32_t position){
  sendCAN(nodeID, 0x607A, 0x00, position);
}

void canInit() {
  // init flexCAN object
    myCan.begin();
    myCan.setBaudRate(500000);
    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();
    myCan.enableMBInterrupts();
    myCan.onReceive(canMessageHandler); // Attach callback for receiving CAN messages
}

void SDInit() {
  // Initialise SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialisation failed!");
        while (1);
    }
    Serial.println("SD card initialised.");

    // Reset the log file (overwrite or create new)
    logFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (!logFile) {
        Serial.println("Failed to create log file!");
        while (1);
    } else {
        Serial.println("Log file opened successfully.");
        logFile.println("Time (s),Target Position,Actual Position"); // Write header
        logFile.close(); // Close the file to ensure it resets
        Serial.println("Log file reset and header written.");
    }
}


void setup() {
    beginTime = millis();

    Serial.begin(115200);

    Serial.println("Starting setup...");

    SDInit();

    Serial.println("Pins initialised");

    canInit();

    Serial.println("CAN initialised");

    // Initialize motor settings
    setMotorMode(0x01);
    Serial.println("Motor enabled");

    // Set position control values. 
    sendCAN(nodeID, 0x30A1, 0x01, P);
    sendCAN(nodeID, 0x30A1, 0x02, I);
    sendCAN(nodeID, 0x30A1, 0x03, D);
    sendCAN(nodeID, 0x30A1, 0x04, ff_v);
    sendCAN(nodeID, 0x30A1, 0x05, ff_a);
    Serial.println("Position controller gains sent");


}

// --------------------- LOOP ---------------------


void loop() {

  unsigned long currentTime = millis();

    // Calculate time since the last update for target position
    if (currentTime - lastUpdateTime >= updateInterval) {
      lastUpdateTime = currentTime;
      timeElapsed += updateInterval / 1000.0; // Convert ms to seconds

      // Calculate the sinusoidal target position
      targetPosition = (int)(amplitude * sin(2 * PI * frequency * timeElapsed));
      
      // Serial.println("Target position: " + String(targetPosition));

      // Send the target position to the EPOS4
      // sendTargetPosition(targetPosition);

      // Immediately request the actual position
      // requestPosition();

      // Log the data to SD card
      // logPosition(currentTime / 1000.0, targetPosition, actualPosition); // Convert ms to seconds

  }

  // sendCAN(nodeID, 0x30A1, 0x01, P);

  
  delay(3000);
  requestValue(0x6041, 0x00);  
}




