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
int i = 0;
double  beginTime;
String fileName = "log1.csv";
uint8_t nodeID = MAXON_PORT;

// Position controller gains
uint32_t P = 1000000000;
uint32_t I = 1;
uint32_t D = 1;
uint32_t ff_v = 1;
uint32_t ff_a = 1;


// Parameters for the sinusoidal wave
const int amplitude = 4096;       // Amplitude in encoder ticks (1 revolution = 4096 ticks)
const float frequency = 0.5;      // Frequency in Hz (0.5 Hz = 2 seconds per cycle)
const int updateInterval = 10;    // Sampling period in ms (100 Hz update rate)
unsigned long lastUpdateTime = 0; // Time tracker for position updates
unsigned long lastRequestTime = 0;        // Time tracker for actual position requests
float timeElapsed = 0.0;          // Total elapsed time in seconds

int targetPosition = 0;   // Stores the target position for logging
int actualPosition = 0;   // Stores the actual position for logging (updated via CAN message)


// --------------------- INITIALISE ---------------------

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

// Callback for receiving CAN messages
// Updated `canMessageHandler` to set `actualPosition`
void canMessageHandler(const CAN_message_t &msg) {
    if (msg.id == 0x580 + static_cast<int>(nodeID)) { // Response to SDO request
        uint16_t index = (msg.buf[2] << 8) | msg.buf[1];
        uint8_t subIndex = msg.buf[3];

        if (index == 0x6064 && subIndex == 0x00) { // Actual position
            actualPosition = (int32_t)(msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24));
            Serial.print("Actual Position: ");
            Serial.println(actualPosition);
        }
    }

    if (msg.id == (0x580 + nodeID)) {  // Check if the response is from our node
      if (msg.buf[0] == 0x43) {  // 4-byte response
          uint32_t value = msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24);
          Serial.print("Actual Value: ");
          Serial.println((int32_t)value);  // Print as signed integer
      } else {
          Serial.println("Error in response!");
      }
    }

    printCANMessage(msg);
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
}

void requestActualValue(uint16_t index, uint8_t subindex) {
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
}

void setMotorMode(uint32_t mode) {
  sendCAN(nodeID, 0x6060, 0x00, mode);
}

void sendTargetPosition(uint32_t position){
  sendCAN(nodeID, 0x607A, 0x00, position);
}


void initPins() {
    pinMode(CTX_PIN, OUTPUT);
    pinMode(CRX_PIN, OUTPUT);
    pinMode(2, OUTPUT);
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
    myCan.mailboxStatus();
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


void logPosition(double timestamp, int32_t target, int32_t actual) {
  logFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (logFile) {
    logFile.print(timestamp, 3);
    logFile.print(",");
    logFile.print(target);
    logFile.print(",");
    logFile.println(actual);
    logFile.close();
  } else {
    Serial.println("Error opening log file!");
  }
}


void setup() {
    beginTime = millis();

    Serial.begin(115200);

    Serial.println("Starting setup...");

    SDInit();
    initPins();

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

    Serial.println("Starting sinusoidal position control...");


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
        sendTargetPosition(targetPosition);

        // Immediately request the actual position
        // requestActualPosition();

        // Log the data to SD card
        // logPosition(currentTime / 1000.0, targetPosition, actualPosition); // Convert ms to seconds

    }

    // sendCAN(nodeID, 0x30A1, 0x03, D);
    delay(100);
    
}

// --------------------- FUNCTIONS ---------------------


// // Request actual position from the EPOS4
// void requestActualPosition() {
//     CAN_message_t message;
//     message.id = 0x600 + 0x01; // Node-ID + 0x00
//     message.buf[0] = 0x40;       // Read request
//     message.buf[1] = 0x64;       // Index LSB (Actual position)
//     message.buf[2] = 0x60;       // Index MSB
//     message.buf[3] = 0x00;       // Sub-index 0
//     message.len = 8;
//     myCan.write(message);
// }

