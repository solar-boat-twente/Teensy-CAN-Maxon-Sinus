#include <FlexCAN_T4.h>
#include <SD.h>
#include <Arduino.h>
#include <math.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// CANopen object dictionary entries for EPOS4
const uint16_t CONTROLWORD_INDEX = 0x6040; // Controlword Index
const uint16_t TARGET_POSITION_INDEX = 0x607A; // Target Position Index
const uint16_t OPERATION_MODE_INDEX = 0x6060; // Operation Mode Index
const uint16_t ACTUAL_POSITION_INDEX = 0x6064; // Actual Position Index
const uint16_t STATUSWORD_INDEX = 0x6041; // Statusword Index
const uint8_t PROFILE_POSITION_MODE = 1; // Profile Position Mode

const uint8_t EPOS_NODE_ID = 1; // Node ID of EPOS4
const float sineFrequency = 0.1; // Hz, sine wave frequency
const int amplitude = 10000; // Position amplitude
const int sineUpdateInterval = 10; // Update interval in ms

unsigned long lastUpdate = 0;
float timeElapsed = 0;
File logFile; // File object for logging

void setup() {
  Serial.begin(115200);
  while (!Serial) {} // Wait for Serial Monitor to open

  // Initialize CAN
  can1.begin();
  can1.setBaudRate(500000); // EPOS4 uses 500k baud by default

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Open the log file
  logFile = SD.open("log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to create log file!");
    while (1);
  }

  // Write CSV header
  logFile.println("Time(ms),Target Position,Actual Position");
  logFile.close();

  Serial.println("CAN bus and logging initialized. Sending initialization commands.");

  // Upload DCF configuration (if needed)
  uploadDCFConfiguration();

  // Send EPOS initialization commands
  setOperationMode(PROFILE_POSITION_MODE);
  enableEPOS();
}

void loop() {
  // Update sine wave target position
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= sineUpdateInterval) {
    lastUpdate = currentTime;

    // Generate a sine wave profile
    int32_t targetPosition = amplitude * sin(2 * M_PI * sineFrequency * timeElapsed);
    timeElapsed += (float)sineUpdateInterval / 1000.0;

    // Send the target position
    writeSDO(TARGET_POSITION_INDEX, 0x00, targetPosition);

    // Read the actual position from the motor
    int32_t actualPosition = readSDO(ACTUAL_POSITION_INDEX, 0x00);

    // Log to SD card
    logPosition(currentTime, targetPosition, actualPosition);

    Serial.print("Target Position: ");
    Serial.print(targetPosition);
    Serial.print(" | Actual Position: ");
    Serial.println(actualPosition);
  }

  // Process CAN messages (if needed)
  CAN_message_t message;
  while (can1.read(message)) {
    handleCANMessage(message);
  }
}

void uploadDCFConfiguration() {
  // Example: Write DCF settings to EPOS4
  // Replace these with actual settings from the .dcf file

  Serial.println("Uploading DCF configuration...");

  // Example entries from the DCF (replace with your specific entries)
  writeSDO(0x6060, 0x00, PROFILE_POSITION_MODE); // Set operation mode
  writeSDO(0x6081, 0x00, 1000); // Profile velocity
  writeSDO(0x6083, 0x00, 2000); // Profile acceleration
  writeSDO(0x6084, 0x00, 2000); // Profile deceleration

  // Add more settings from the DCF file here...

  Serial.println("DCF configuration complete.");
}

void enableEPOS() {
  // Send controlword to enable EPOS4
  writeSDO(CONTROLWORD_INDEX, 0x00, 0x0006); // Shutdown
  delay(100);
  writeSDO(CONTROLWORD_INDEX, 0x00, 0x000F); // Switch On & Enable Operation
  delay(100);
}

void setOperationMode(uint8_t mode) {
  // Set the operation mode (e.g., Profile Position Mode)
  writeSDO(OPERATION_MODE_INDEX, 0x00, mode);
}

void writeSDO(uint16_t index, uint8_t subindex, int32_t data) {
  CAN_message_t message;

  message.id = 0x600 + EPOS_NODE_ID; // SDO Request COB-ID
  message.len = 8;
  message.buf[0] = 0x23; // SDO write command (4-byte data)
  message.buf[1] = index & 0xFF;
  message.buf[2] = (index >> 8) & 0xFF;
  message.buf[3] = subindex;
  message.buf[4] = data & 0xFF;
  message.buf[5] = (data >> 8) & 0xFF;
  message.buf[6] = (data >> 16) & 0xFF;
  message.buf[7] = (data >> 24) & 0xFF;

  can1.write(message);
}

int32_t readSDO(uint16_t index, uint8_t subindex) {
  CAN_message_t request, response;

  // Send SDO read request
  request.id = 0x600 + EPOS_NODE_ID; // SDO Request COB-ID
  request.len = 8;
  request.buf[0] = 0x40; // SDO read command
  request.buf[1] = index & 0xFF;
  request.buf[2] = (index >> 8) & 0xFF;
  request.buf[3] = subindex;
  for (int i = 4; i < 8; i++) request.buf[i] = 0;

  can1.write(request);

  // Wait for response (simplified; add timeout for robustness)
  while (!can1.read(response)) {}

  if (response.id == (0x580 + EPOS_NODE_ID)) {
    return response.buf[4] | (response.buf[5] << 8) | (response.buf[6] << 16) | (response.buf[7] << 24);
  }
  return 0;
}

void logPosition(unsigned long timestamp, int32_t target, int32_t actual) {
  logFile = SD.open("log.csv", FILE_WRITE);
  if (logFile) {
    logFile.print(timestamp);
    logFile.print(",");
    logFile.print(target);
    logFile.print(",");
    logFile.println(actual);
    logFile.close();
  } else {
    Serial.println("Error opening log file!");
  }
}

void handleCANMessage(const CAN_message_t &message) {
  // Handle incoming CAN messages (e.g., Statusword)
  if (message.id == 0x580 + EPOS_NODE_ID) {
    uint16_t index = message.buf[1] | (message.buf[2] << 8);
    uint8_t subindex = message.buf[3];

    if (index == STATUSWORD_INDEX) {
      uint16_t statusword = message.buf[4] | (message.buf[5] << 8);
      Serial.print("Statusword: ");
      Serial.println(statusword, HEX);
    }
  }
}
