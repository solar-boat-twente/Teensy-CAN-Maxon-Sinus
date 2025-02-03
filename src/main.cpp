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
double  beginTime;
String fileName = "portLog.csv";
uint8_t nodeID = MAXON_PORT;

// Position controller gains
uint32_t P = 1661459;
uint32_t I = 62667205;
uint32_t D = 16448;
uint32_t ff_v = 494;
uint32_t ff_a = 66;


// Parameters for the sinusoidal wave
const int amplitude = 3000;       // Amplitude in encoder ticks
const float frequency = 0.1;      // Frequency in Hz
unsigned long duration = 3000;  // Duration in milliseconds

unsigned long lastSendTime = 0; // Track time of the last message
unsigned long startTime = 0;    // Track the start time of the loop

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
  Serial.print("Received ");
  printCANMessage(msg);
  // logCANMessage(msg); // Save to SD card
}

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

  Serial.print("Sent     ");
  printCANMessage(msg);
}

void sendCAN_2B(uint8_t nodeID, uint16_t index ,uint8_t subindex, uint32_t value){
  CAN_message_t msg;
  msg.id = 0x600 + nodeID;
  msg.flags.extended = 0;
  msg.len = 8;

  msg.buf[0] = 0x2B;               // SDO Command: Write 4 bytes
  msg.buf[1] = index & 0xFF;       // Index low byte
  msg.buf[2] = (index >> 8) & 0xFF;// Index high byte
  msg.buf[3] = subindex;           // Subindex
  // Convert value to Little Endian format
  msg.buf[4] = value & 0xFF;       
  msg.buf[5] = (value >> 8) & 0xFF;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;

  myCan.write(msg);

  Serial.print("Sent     ");
  printCANMessage(msg);
}

void sendCAN_2F(uint8_t nodeID, uint16_t index ,uint8_t subindex, uint32_t value){
  CAN_message_t msg;
  msg.id = 0x600 + nodeID;
  msg.flags.extended = 0;
  msg.len = 8;

  msg.buf[0] = 0x2F;               // SDO Command: Write 4 bytes
  msg.buf[1] = index & 0xFF;       // Index low byte
  msg.buf[2] = (index >> 8) & 0xFF;// Index high byte
  msg.buf[3] = subindex;           // Subindex
  // Convert value to Little Endian format
  msg.buf[4] = value & 0xFF;       
  msg.buf[5] = (value >> 8) & 0xFF;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;

  myCan.write(msg);

  Serial.print("Sent     ");
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

void initMotor() {
  Serial.println("Ready to switch on, Switched on, Enable operation:");
  sendCAN_2B(nodeID, 0x6040, 0x00, 0x06); // Ready to switch on
  delay(1);
  sendCAN_2B(nodeID, 0x6040, 0x00, 0x07); // Switched on
  delay(1);
  sendCAN_2B(nodeID, 0x6040, 0x00, 0x0F); // Enable operation
  delay(1);
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

// Function to convert decimal value to 2's complement hex value
unsigned long toTwosComplementHex(int value) {
  // Assuming the value fits within 32 bits, use a 32-bit mask to handle wrapping
  if (value < 0) {
    // Calculate 2's complement for negative numbers
    return (unsigned long)((1L << 32) + value); // 32-bit 2's complement
  }
  // For positive values, just return the value
  return (unsigned long)value;
}


void setup() {
  beginTime = millis();

  Serial.begin(115200);

  Serial.println("Starting setup...");

  SDInit();

  Serial.println("Pins initialised");

  canInit();

  // Set position control values. 
  // sendCAN(nodeID, 0x30A1, 0x01, P);
  // sendCAN(nodeID, 0x30A1, 0x02, I);
  // sendCAN(nodeID, 0x30A1, 0x03, D);
  // sendCAN(nodeID, 0x30A1, 0x04, ff_v);
  // sendCAN(nodeID, 0x30A1, 0x05, ff_a);
  // Serial.println("Position controller gains sent");

  delay(3000);

  requestValue(0x6041, 0x00);
  

  // initMotor(); // Motor enable

  // Serial.println("Set target position:");
  // delay(1);
  // sendCAN(nodeID, 0x607A, 0x00, 0); // Set target position

  // Serial.println("What is the target position:");
  // delay(1);
  // requestValue(0x607A, 0x00); // Kijken wat de target position is

  // Serial.println("Start movement:");
  // delay(1);
  // sendCAN_2B(nodeID, 0x6040, 0x00, 0x1F); // Set 'new setpoint' and 'start motion bits'

  // Serial.println("Movement status:");
  // delay(1);
  // requestValue(0x6041, 0x00);

  // startTime = millis();

}

// --------------------- LOOP ---------------------


void loop() {
  // if (millis() - startTime < duration) {
  //   int value = amplitude * sin(2 * PI * frequency * (millis() / 1000.0));
  //   sendCAN(nodeID, 0x607A, 0x00, toTwosComplementHex(value));
  //   delay(1);
  //   sendCAN_2B(nodeID, 0x6040, 0x00, 0x1F);
  //   delay(1);

  //   // logPosition(currentTime / 1000.0, targetPosition, actualPosition); // Convert ms to seconds
  // }
}





