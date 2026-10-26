#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <FFat.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Adafruit_BME280.h>
#include "lsm6dsv320x_reg.h"

// Arduino Nano ESP32 payload wiring from the supplied schematic.
constexpr uint8_t GPS_RX_PIN = D0;
constexpr uint8_t GPS_TX_PIN = D1;
constexpr uint8_t I2C_SDA_PIN = A4;
constexpr uint8_t I2C_SCL_PIN = A5;
constexpr uint8_t SPI_SCK_PIN = D13;
constexpr uint8_t SPI_MISO_PIN = D12;
constexpr uint8_t SPI_MOSI_PIN = D11;
constexpr uint8_t SD_CS_PIN = D4;
constexpr uint8_t LORA_CS_PIN = D8;
constexpr uint8_t LORA_RST_PIN = D6;
constexpr uint8_t LORA_DIO0_PIN = D7;

constexpr uint8_t LSM6DSV320X_ADDRESS = 0x6A;
constexpr uint8_t BME_SENSOR_ADDRESS = 0x76;
constexpr uint32_t GPS_BAUD = 9600;

constexpr long LORA_FREQUENCY_HZ = 915000000L;
constexpr uint8_t LORA_SPREADING_FACTOR = 8;
constexpr long LORA_BANDWIDTH_HZ = 125000L;
constexpr uint8_t LORA_CODING_RATE = 5;
constexpr uint16_t LORA_PREAMBLE_LENGTH = 12;
constexpr uint8_t LORA_SYNC_WORD = 0x34;
constexpr int8_t LORA_TX_POWER_DBM = 17;

constexpr uint32_t SENSOR_INTERVAL_MS = 10;
constexpr uint32_t BAROMETER_INTERVAL_MS = 100;
constexpr uint32_t LOG_INTERVAL_MS = 20;
constexpr uint8_t LOG_BATCH_ROWS = 10;
constexpr uint32_t LOG_BATCH_MAX_AGE_MS = 200;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
constexpr uint32_t HEALTH_INTERVAL_MS = 5000;
constexpr uint8_t INTERNAL_FLIGHT_SLOTS = 5;
constexpr uint64_t INTERNAL_LOW_SPACE_BYTES = 512ULL * 1024ULL;
constexpr uint8_t STATUS_LED_BRIGHTNESS = 51;  // 20% of 255.
constexpr float WIFI_THERMAL_CUTOFF_C = 75.0f;

const char ROCKET_ID[] = "1333877";
const char WIFI_SSID[] = "XenoCam-1333877";
const char WIFI_PASSWORD[] = "XenoCam915";
String rocketName = "XenoCam";

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Adafruit_BME280 bme;
stmdev_ctx_t imuContext = {};
File sdLogFile;
Preferences preferences;
WebServer fileServer(80);

enum class StorageMode : uint8_t {
  INTERNAL = 0,
  SD_CARD = 1,
  BOTH = 2
};

bool imuReady = false;
bool bmeReady = false;
bool sdReady = false;
bool internalReady = false;
bool internalRecordingActive = false;
bool loraReady = false;
bool loraTxInProgress = false;
volatile bool loraTxDone = false;
bool recording = false;
bool wifiActive = false;
bool wifiStopRequested = false;
uint8_t pendingStopAcks = 0;
uint32_t nextStopAckAt = 0;
StorageMode storageMode = StorageMode::BOTH;
float referencePressurePa = 0.0f;
uint32_t recordsWritten = 0;
uint16_t pendingLogRows = 0;
uint32_t logBatchStartedAt = 0;
uint64_t sdFreeKB = 0;
uint8_t flightState = 0;  // No automatic launch detection.
uint32_t wifiStartedAt = 0;
uint32_t wifiLastClientAt = 0;
String sdLogPath;
String internalLogPath;
String pendingLogBatch;
uint8_t activeInternalSlot = 0xFF;

float lowZeroX = 0.0f;
float lowZeroY = 0.0f;
float lowZeroZ = 0.0f;
float highZeroX = 0.0f;
float highZeroY = 0.0f;
float highZeroZ = 0.0f;

struct Measurements {
  uint32_t timeMs = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitudeM = 0.0f;
  uint8_t satellites = 0;
  float temperatureC = 0.0f;
  float pressureHpa = 0.0f;
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  float highAccelX = 0.0f;
  float highAccelY = 0.0f;
  float highAccelZ = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
} sample;

void flushLogBatch();
bool verifyInternalBytes(size_t offset, const String &expected);

void onRadioTxDone() {
  loraTxDone = true;
}

void setRecordingLed(bool enabled) {
  static int8_t previousState = -1;
  if (previousState == static_cast<int8_t>(enabled)) {
    return;
  }
  previousState = static_cast<int8_t>(enabled);
  // The Nano ESP32 RGB LED is active-low. Orange is limited to 20% brightness.
  const uint8_t red = enabled ? STATUS_LED_BRIGHTNESS : 0;
  const uint8_t green = enabled ? static_cast<uint8_t>(STATUS_LED_BRIGHTNESS * 40 / 100) : 0;
  const uint8_t blue = 0;
  analogWrite(LED_RED, 255 - red);
  analogWrite(LED_GREEN, 255 - green);
  analogWrite(LED_BLUE, 255 - blue);
}

int32_t imuWrite(void *handle, uint8_t reg, const uint8_t *data, uint16_t length) {
  const uint8_t address = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(handle));
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, length);
  return Wire.endTransmission() == 0 ? 0 : -1;
}

int32_t imuRead(void *handle, uint8_t reg, uint8_t *data, uint16_t length) {
  const uint8_t address = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(handle));
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return -1;
  }

  const size_t received = Wire.requestFrom(
    static_cast<uint8_t>(address),
    static_cast<size_t>(length),
    true);
  if (received != length) {
    return -1;
  }
  for (uint16_t index = 0; index < length; ++index) {
    data[index] = static_cast<uint8_t>(Wire.read());
  }
  return 0;
}

void imuDelay(uint32_t milliseconds) {
  delay(milliseconds);
}

bool initializeImu() {
  imuContext.write_reg = imuWrite;
  imuContext.read_reg = imuRead;
  imuContext.mdelay = imuDelay;
  imuContext.handle = reinterpret_cast<void *>(static_cast<uintptr_t>(LSM6DSV320X_ADDRESS));

  uint8_t deviceId = 0;
  if (lsm6dsv320x_device_id_get(&imuContext, &deviceId) != 0 ||
      deviceId != LSM6DSV320X_ID) {
    Serial.printf("IMU ERROR: expected 0x%02X, read 0x%02X\n",
                  LSM6DSV320X_ID, deviceId);
    return false;
  }

  if (lsm6dsv320x_sw_reset(&imuContext) != 0) {
    Serial.println("IMU ERROR: reset failed.");
    return false;
  }
  delay(20);

  int32_t status = 0;
  status += lsm6dsv320x_auto_increment_set(&imuContext, PROPERTY_ENABLE);
  status += lsm6dsv320x_block_data_update_set(&imuContext, PROPERTY_ENABLE);
  status += lsm6dsv320x_xl_full_scale_set(&imuContext, LSM6DSV320X_16g);
  status += lsm6dsv320x_gy_full_scale_set(&imuContext, LSM6DSV320X_2000dps);
  status += lsm6dsv320x_hg_xl_full_scale_set(&imuContext, LSM6DSV320X_320g);
  status += lsm6dsv320x_xl_data_rate_set(&imuContext, LSM6DSV320X_ODR_AT_240Hz);
  status += lsm6dsv320x_gy_data_rate_set(&imuContext, LSM6DSV320X_ODR_AT_240Hz);
  status += lsm6dsv320x_hg_xl_data_rate_set(
    &imuContext, LSM6DSV320X_HG_XL_ODR_AT_960Hz, PROPERTY_ENABLE);

  if (status != 0) {
    Serial.printf("IMU ERROR: configuration status=%ld\n", static_cast<long>(status));
    return false;
  }

  Serial.println("LSM6DSV320X ready: low-g 16g/240Hz, gyro 2000dps/240Hz, high-g 320g/960Hz.");
  return true;
}

bool readImuRaw(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz,
                float &hax, float &hay, float &haz) {
  if (!imuReady) {
    return false;
  }

  int16_t lowRaw[3] = {};
  int16_t gyroRaw[3] = {};
  int16_t highRaw[3] = {};
  if (lsm6dsv320x_acceleration_raw_get(&imuContext, lowRaw) != 0 ||
      lsm6dsv320x_angular_rate_raw_get(&imuContext, gyroRaw) != 0 ||
      lsm6dsv320x_hg_acceleration_raw_get(&imuContext, highRaw) != 0) {
    return false;
  }

  ax = lsm6dsv320x_from_fs16_to_mg(lowRaw[0]) / 1000.0f;
  ay = lsm6dsv320x_from_fs16_to_mg(lowRaw[1]) / 1000.0f;
  az = lsm6dsv320x_from_fs16_to_mg(lowRaw[2]) / 1000.0f;
  gx = lsm6dsv320x_from_fs2000_to_mdps(gyroRaw[0]) / 1000.0f;
  gy = lsm6dsv320x_from_fs2000_to_mdps(gyroRaw[1]) / 1000.0f;
  gz = lsm6dsv320x_from_fs2000_to_mdps(gyroRaw[2]) / 1000.0f;
  hax = lsm6dsv320x_from_fs320_to_mg(highRaw[0]) / 1000.0f;
  hay = lsm6dsv320x_from_fs320_to_mg(highRaw[1]) / 1000.0f;
  haz = lsm6dsv320x_from_fs320_to_mg(highRaw[2]) / 1000.0f;
  return true;
}

void zeroAccelerometers() {
  if (!imuReady) {
    return;
  }

  float lowSumX = 0.0f;
  float lowSumY = 0.0f;
  float lowSumZ = 0.0f;
  float highSumX = 0.0f;
  float highSumY = 0.0f;
  float highSumZ = 0.0f;
  uint16_t valid = 0;

  for (uint16_t index = 0; index < 120; ++index) {
    float ax, ay, az, gx, gy, gz, hax, hay, haz;
    if (readImuRaw(ax, ay, az, gx, gy, gz, hax, hay, haz)) {
      lowSumX += ax;
      lowSumY += ay;
      lowSumZ += az;
      highSumX += hax;
      highSumY += hay;
      highSumZ += haz;
      valid++;
    }
    delay(5);
  }

  if (valid > 0) {
    lowZeroX = lowSumX / valid;
    lowZeroY = lowSumY / valid;
    lowZeroZ = lowSumZ / valid - 1.0f;
    highZeroX = highSumX / valid;
    highZeroY = highSumY / valid;
    highZeroZ = highSumZ / valid - 1.0f;
  }
}

void updateImu() {
  float ax, ay, az, gx, gy, gz, hax, hay, haz;
  if (!readImuRaw(ax, ay, az, gx, gy, gz, hax, hay, haz)) {
    return;
  }

  sample.accelX = ax - lowZeroX;
  sample.accelY = ay - lowZeroY;
  sample.accelZ = az - lowZeroZ;
  sample.gyroX = gx;
  sample.gyroY = gy;
  sample.gyroZ = gz;
  sample.highAccelX = hax - highZeroX;
  sample.highAccelY = hay - highZeroY;
  sample.highAccelZ = haz - highZeroZ;
  sample.roll = atan2f(sample.accelY, sample.accelZ) * 180.0f / PI;
  sample.pitch = atan2f(
    -sample.accelX,
    sqrtf(sample.accelY * sample.accelY + sample.accelZ * sample.accelZ)) *
    180.0f / PI;
}

void zeroAltitude() {
  if (!bmeReady) {
    referencePressurePa = 0.0f;
    return;
  }

  float total = 0.0f;
  uint16_t valid = 0;
  for (uint16_t index = 0; index < 50; ++index) {
    const float pressure = bme.readPressure();
    if (isfinite(pressure) && pressure > 30000.0f && pressure < 120000.0f) {
      total += pressure;
      valid++;
    }
    delay(20);
  }
  referencePressurePa = valid > 0 ? total / valid : 0.0f;
  Serial.printf("Altitude reference pressure: %.2f hPa\n", referencePressurePa / 100.0f);
}

void updateBarometer() {
  if (!bmeReady) {
    return;
  }

  const float pressure = bme.readPressure();
  const float temperature = bme.readTemperature();
  if (!isfinite(pressure) || pressure < 30000.0f || pressure > 120000.0f) {
    return;
  }

  sample.pressureHpa = pressure / 100.0f;
  if (isfinite(temperature)) {
    sample.temperatureC = temperature;
  }
  if (referencePressurePa > 0.0f) {
    sample.altitudeM =
      44330.0f * (1.0f - powf(pressure / referencePressurePa, 0.190294957f));
  }
}

bool wantsInternalStorage() {
  return storageMode == StorageMode::INTERNAL || storageMode == StorageMode::BOTH;
}

bool wantsSdStorage() {
  return storageMode == StorageMode::SD_CARD || storageMode == StorageMode::BOTH;
}

const char *storageModeName() {
  switch (storageMode) {
    case StorageMode::INTERNAL: return "INTERNAL";
    case StorageMode::SD_CARD: return "SD";
    default: return "BOTH";
  }
}

String internalSlotKey(uint8_t slot) {
  return "slot" + String(slot);
}

String defaultInternalSlotPath(uint8_t slot) {
  return "/flight" + String(slot + 1) + ".csv";
}

String internalSlotPath(uint8_t slot) {
  const String key = internalSlotKey(slot);
  String stored = preferences.getString(key.c_str(), "");
  stored.trim();
  if (stored.startsWith("/") && stored.endsWith(".csv") &&
      stored.indexOf("..") < 0 && stored.indexOf('\\') < 0) {
    return stored;
  }
  return defaultInternalSlotPath(slot);
}

void saveInternalSlotPath(uint8_t slot, const String &path) {
  const String key = internalSlotKey(slot);
  if (path.length() > 0) {
    preferences.putString(key.c_str(), path);
  } else {
    preferences.remove(key.c_str());
  }
}

int8_t findInternalSlot(const String &path) {
  for (uint8_t slot = 0; slot < INTERNAL_FLIGHT_SLOTS; ++slot) {
    if (internalSlotPath(slot) == path) {
      return static_cast<int8_t>(slot);
    }
  }
  return -1;
}

bool deleteOldestCompletedInternalFlight() {
  if (!internalReady) {
    return false;
  }
  const uint8_t oldest = preferences.getUChar("nextFlight", 0) % INTERNAL_FLIGHT_SLOTS;
  for (uint8_t offset = 0; offset < INTERNAL_FLIGHT_SLOTS; ++offset) {
    const uint8_t slot = (oldest + offset) % INTERNAL_FLIGHT_SLOTS;
    const String path = internalSlotPath(slot);
    if (slot == activeInternalSlot || path == internalLogPath || !FFat.exists(path)) {
      continue;
    }
    if (FFat.remove(path)) {
      saveInternalSlotPath(slot, "");
      Serial.printf("Internal memory low: deleted oldest completed flight %s.\n", path.c_str());
      return true;
    }
  }
  return false;
}

bool reclaimInternalSpace() {
  if (!internalReady) {
    return false;
  }
  for (uint8_t attempt = 0;
       attempt < INTERNAL_FLIGHT_SLOTS && FFat.freeBytes() < INTERNAL_LOW_SPACE_BYTES;
       ++attempt) {
    if (!deleteOldestCompletedInternalFlight()) {
      break;
    }
  }
  return FFat.freeBytes() >= INTERNAL_LOW_SPACE_BYTES;
}

bool verifyInternalBytes(size_t offset, const String &expected) {
  if (!internalReady || internalLogPath.length() == 0) {
    return false;
  }
  File verify = FFat.open(internalLogPath, FILE_READ);
  if (!verify || verify.isDirectory() || verify.size() != offset + expected.length() ||
      !verify.seek(offset)) {
    if (verify) verify.close();
    return false;
  }
  for (size_t index = 0; index < expected.length(); ++index) {
    if (verify.read() != static_cast<uint8_t>(expected[index])) {
      verify.close();
      return false;
    }
  }
  const bool exactEnd = verify.available() == 0;
  verify.close();
  return exactEnd;
}

bool initializeStorage() {
  internalReady = FFat.begin(false);
  if (!internalReady) {
    // A stale preference must not permanently disable internal logging.  The
    // format-on-failure retry repairs an unformatted or damaged FAT partition.
    Serial.println("Internal FAT mount failed: repairing the flight partition.");
    internalReady = FFat.begin(true);
  }
  if (internalReady) {
    preferences.putBool("fatInitialized", true);
    Serial.printf("Internal FAT ready: %llu KB free.\n",
                  static_cast<unsigned long long>(FFat.freeBytes() / 1024ULL));
  } else {
    Serial.println("INTERNAL ERROR: FAT partition mount failed; continuing with SD if available.");
  }

  digitalWrite(LORA_CS_PIN, HIGH);
  sdReady = SD.begin(SD_CS_PIN, SPI, 20000000);
  if (sdReady) {
    sdFreeKB = (SD.totalBytes() - SD.usedBytes()) / 1024ULL;
    Serial.printf("SD ready: %llu KB free.\n", static_cast<unsigned long long>(sdFreeKB));
  } else {
    Serial.println("SD ERROR: card mount failed; continuing with internal flash if available.");
  }
  return internalReady || sdReady;
}

bool startRecording() {
  if (recording && (internalRecordingActive || sdLogFile)) {
    return true;
  }

  if (wantsInternalStorage() && !internalReady) {
    internalReady = FFat.begin(false);
  }
  if (wantsSdStorage() && !sdReady) {
    digitalWrite(LORA_CS_PIN, HIGH);
    sdReady = SD.begin(SD_CS_PIN, SPI, 20000000);
  }
  bool recordInternal = wantsInternalStorage() && internalReady;
  bool recordSd = wantsSdStorage() && sdReady;
  if (!recordInternal && !recordSd) {
    // Never discard a flight solely because the selected medium is absent.
    recordInternal = internalReady;
    recordSd = !recordInternal && sdReady;
    if (recordInternal || recordSd) {
      Serial.println("Selected storage unavailable; using the available fallback.");
    }
  }

  static const char CSV_HEADER[] =
    "time_ms,latitude,longitude,altitude_m,satellites,temp_c,pressure_hpa,"
    "accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,"
    "high_accel_x_g,high_accel_y_g,high_accel_z_g,pitch_deg,roll_deg";

  internalLogPath = "";
  sdLogPath = "";
  pendingLogBatch = "";
  pendingLogBatch.reserve(2400);
  pendingLogRows = 0;
  logBatchStartedAt = millis();
  activeInternalSlot = 0xFF;
  internalRecordingActive = false;

  if (recordInternal) {
    const uint8_t slot = preferences.getUChar("nextFlight", 0) % INTERNAL_FLIGHT_SLOTS;
    const String previousPath = internalSlotPath(slot);
    internalLogPath = defaultInternalSlotPath(slot);
    activeInternalSlot = slot;
    if (FFat.exists(previousPath)) {
      FFat.remove(previousPath);
    }
    if (previousPath != internalLogPath && FFat.exists(internalLogPath)) {
      FFat.remove(internalLogPath);
    }
    reclaimInternalSpace();
    const String header = String(CSV_HEADER) + '\n';
    File create = FFat.open(internalLogPath, FILE_WRITE);
    const size_t headerBytes = create
      ? create.write(reinterpret_cast<const uint8_t *>(header.c_str()), header.length())
      : 0;
    if (create) {
      create.flush();
      create.close();
    }
    if (headerBytes == header.length() && verifyInternalBytes(0, header)) {
      internalRecordingActive = true;
      saveInternalSlotPath(slot, internalLogPath);
      preferences.putUChar("nextFlight", (slot + 1) % INTERNAL_FLIGHT_SLOTS);
      Serial.printf("Internal recording slot %u verified: %s\n", slot + 1, internalLogPath.c_str());
    } else {
      Serial.println("INTERNAL ERROR: could not create and verify flight log.");
      internalLogPath = "";
      activeInternalSlot = 0xFF;
    }
  }

  if (recordSd) {
    char path[24] = {};
    bool found = false;
    for (uint16_t index = 0; index < 1000; ++index) {
      snprintf(path, sizeof(path), "/flight%03u.csv", index);
      if (!SD.exists(path)) {
        found = true;
        break;
      }
    }
    if (found) {
      sdLogFile = SD.open(path, FILE_WRITE);
      if (sdLogFile && sdLogFile.println(CSV_HEADER) > 0) {
        sdLogPath = path;
        sdLogFile.flush();
        Serial.print("SD recording: ");
        Serial.println(sdLogPath);
      } else {
        Serial.println("SD ERROR: could not create flight log.");
        if (sdLogFile) sdLogFile.close();
      }
    } else {
      Serial.println("SD ERROR: no free flight filename.");
    }
  }

  recordsWritten = 0;
  recording = internalRecordingActive || sdLogFile;
  if (!recording) {
    Serial.printf("RECORDING ERROR: no writable %s destination.\n", storageModeName());
  } else {
    // Commit one real snapshot immediately so a short recording can never be
    // left as a header-only file. Both destinations receive the same row.
    sample.timeMs = millis();
    writeRecord();
    flushLogBatch();
  }
  setRecordingLed(recording);
  return recording;
}

void stopRecording() {
  const bool hadRecording = recording || internalRecordingActive || sdLogFile || pendingLogRows > 0;
  if (internalRecordingActive || sdLogFile || pendingLogRows > 0) {
    flushLogBatch();
  }
  recording = false;
  internalRecordingActive = false;
  if (sdLogFile) {
    sdLogFile.flush();
    sdLogFile.close();
  }
  activeInternalSlot = 0xFF;
  setRecordingLed(false);
  if (!hadRecording) {
    Serial.println("Recording was already stopped.");
    return;
  }
  Serial.printf("Recording stopped. Direct sensor log finalized with %lu rows.\n",
                static_cast<unsigned long>(recordsWritten));
}

void setStorageMode(StorageMode requestedMode) {
  if (storageMode == requestedMode) {
    preferences.putUChar("storage", static_cast<uint8_t>(storageMode));
    Serial.printf("Storage mode already %s.\n", storageModeName());
    return;
  }
  const bool wasRecording = recording;
  if (recording) {
    stopRecording();
  }
  storageMode = requestedMode;
  preferences.putUChar("storage", static_cast<uint8_t>(storageMode));
  Serial.printf("Storage mode: %s\n", storageModeName());
  if (wasRecording) {
    startRecording();
  }
}

void flushLogBatch() {
  if (pendingLogRows == 0 || pendingLogBatch.length() == 0) {
    return;
  }

  const size_t batchBytes = pendingLogBatch.length();
  const uint16_t batchRows = pendingLogRows;
  bool wroteBatch = false;

  if (internalRecordingActive) {
    if (FFat.freeBytes() < INTERNAL_LOW_SPACE_BYTES) {
      reclaimInternalSpace();
    }
    bool internalBatchSaved = false;
    size_t lastWritten = 0;
    for (uint8_t attempt = 0; attempt < 3 && !internalBatchSaved; ++attempt) {
      File existing = FFat.open(internalLogPath, FILE_READ);
      if (!existing || existing.isDirectory()) {
        if (existing) existing.close();
        delay(2);
        continue;
      }
      const size_t previousSize = existing.size();
      existing.close();

      File append = FFat.open(internalLogPath, FILE_APPEND);
      if (!append) {
        reclaimInternalSpace();
        delay(2);
        continue;
      }
      lastWritten = append.write(
        reinterpret_cast<const uint8_t *>(pendingLogBatch.c_str()), batchBytes);
      append.flush();
      append.close();

      if (lastWritten == batchBytes && verifyInternalBytes(previousSize, pendingLogBatch)) {
        internalBatchSaved = true;
        break;
      }

      // A partial append must never be retried because that would duplicate or
      // splice sensor rows. Stop this destination and preserve its exact bytes.
      if (lastWritten > 0) {
        break;
      }
      delay(2);
    }
    if (internalBatchSaved) {
      wroteBatch = true;
    } else {
      Serial.printf("INTERNAL ERROR: exact batch verification failed (%u/%u bytes).\n",
                    static_cast<unsigned>(lastWritten), static_cast<unsigned>(batchBytes));
      internalRecordingActive = false;
      activeInternalSlot = 0xFF;
    }
  }

  if (sdLogFile) {
    const size_t written = sdLogFile.print(pendingLogBatch);
    if (written == batchBytes) {
      sdLogFile.flush();
      wroteBatch = true;
    } else {
      Serial.printf("SD ERROR: batch write failed (%u/%u bytes).\n",
                    static_cast<unsigned>(written), static_cast<unsigned>(batchBytes));
      sdLogFile.close();
    }
  }

  if (wroteBatch) {
    recordsWritten += batchRows;
  }
  pendingLogBatch = "";
  pendingLogRows = 0;
  logBatchStartedAt = millis();
  recording = internalRecordingActive || sdLogFile;
  setRecordingLed(recording);
}

void appendSensorCsvFields(String &line) {
  // Internal flash and LoRa must serialize the exact same sensor snapshot with
  // the exact same precision. LoRa adds only its protocol/status fields after
  // these 18 fields.
  line += String(sample.timeMs) + ",";
  line += String(sample.latitude, 6) + ",";
  line += String(sample.longitude, 6) + ",";
  line += String(sample.altitudeM, 2) + ",";
  line += String(sample.satellites) + ",";
  line += String(sample.temperatureC, 2) + ",";
  line += String(sample.pressureHpa, 2) + ",";
  line += String(sample.accelX, 3) + ",";
  line += String(sample.accelY, 3) + ",";
  line += String(sample.accelZ, 3) + ",";
  line += String(sample.gyroX, 2) + ",";
  line += String(sample.gyroY, 2) + ",";
  line += String(sample.gyroZ, 2) + ",";
  line += String(sample.highAccelX, 3) + ",";
  line += String(sample.highAccelY, 3) + ",";
  line += String(sample.highAccelZ, 3) + ",";
  line += String(sample.pitch, 2) + ",";
  line += String(sample.roll, 2);
}

void writeRecord() {
  if (!recording || (!internalRecordingActive && !sdLogFile)) {
    return;
  }

  String line;
  line.reserve(220);
  appendSensorCsvFields(line);

  if (pendingLogRows == 0) {
    logBatchStartedAt = millis();
  }
  pendingLogBatch += line;
  pendingLogBatch += '\n';
  pendingLogRows++;

  if (pendingLogRows >= LOG_BATCH_ROWS ||
      millis() - logBatchStartedAt >= LOG_BATCH_MAX_AGE_MS) {
    flushLogBatch();
  }
}

bool initializeRadio() {
  digitalWrite(SD_CS_PIN, HIGH);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    Serial.println("LORA ERROR: RFM95 initialization failed.");
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.receive();
  Serial.println("RFM95 ready: 915MHz SF8 BW125 CR4/5 P12 CRC.");
  return true;
}

String htmlEscape(String value) {
  value.replace("&", "&amp;");
  value.replace("<", "&lt;");
  value.replace(">", "&gt;");
  value.replace("\"", "&quot;");
  value.replace("'", "&#39;");
  return value;
}

String urlEncode(const String &value) {
  const char hexDigits[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 3);
  for (size_t index = 0; index < value.length(); ++index) {
    const uint8_t c = static_cast<uint8_t>(value[index]);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.') {
      encoded += static_cast<char>(c);
    } else {
      encoded += '%';
      encoded += hexDigits[c >> 4];
      encoded += hexDigits[c & 0x0F];
    }
  }
  return encoded;
}

bool isSafeCsvPath(String path) {
  path.toLowerCase();
  return path.startsWith("/") && path.endsWith(".csv") &&
         path.indexOf("..") < 0 && path.indexOf('\\') < 0;
}

fs::FS *filesystemForSource(const String &source) {
  if (source == "internal" && internalReady) {
    return &FFat;
  }
  if (source == "sd" && sdReady) {
    return &SD;
  }
  return nullptr;
}

bool isActiveLog(const String &source, const String &path) {
  return recording &&
    ((source == "internal" && internalRecordingActive && path == internalLogPath) ||
     (source == "sd" && sdLogFile && path == sdLogPath));
}

String sanitizedCsvName(const String &requested) {
  String clean;
  clean.reserve(52);
  for (size_t index = 0; index < requested.length() && clean.length() < 48; ++index) {
    const char c = requested[index];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.') {
      clean += c;
    } else if (c == ' ') {
      clean += '_';
    }
  }
  while (clean.startsWith(".")) {
    clean.remove(0, 1);
  }
  String lower = clean;
  lower.toLowerCase();
  if (!lower.endsWith(".csv")) {
    if (clean.length() > 44) clean.remove(44);
    clean += ".csv";
  }
  return clean;
}

void redirectToFileIndex() {
  fileServer.sendHeader("Location", "/", true);
  fileServer.send(303, "text/plain", "Returning to flight files.");
}

void deleteCsv() {
  wifiLastClientAt = millis();
  const String source = fileServer.arg("src");
  String path = fileServer.arg("file");
  if (!path.startsWith("/")) path = "/" + path;
  fs::FS *filesystem = filesystemForSource(source);
  if (filesystem == nullptr || !isSafeCsvPath(path)) {
    fileServer.send(400, "text/plain", "Invalid CSV path.");
    return;
  }
  if (isActiveLog(source, path)) {
    fileServer.send(409, "text/plain", "Stop recording before deleting the active flight.");
    return;
  }
  if (!filesystem->exists(path) || !filesystem->remove(path)) {
    fileServer.send(404, "text/plain", "CSV file was not found or could not be deleted.");
    return;
  }
  if (source == "internal") {
    const int8_t slot = findInternalSlot(path);
    if (slot >= 0) saveInternalSlotPath(static_cast<uint8_t>(slot), "");
  }
  redirectToFileIndex();
}

void renameCsv() {
  wifiLastClientAt = millis();
  const String source = fileServer.arg("src");
  String path = fileServer.arg("file");
  if (!path.startsWith("/")) path = "/" + path;
  const String cleanName = sanitizedCsvName(fileServer.arg("newName"));
  const String newPath = "/" + cleanName;
  fs::FS *filesystem = filesystemForSource(source);
  if (filesystem == nullptr || !isSafeCsvPath(path) || cleanName.length() <= 4 ||
      !isSafeCsvPath(newPath)) {
    fileServer.send(400, "text/plain", "Invalid CSV name.");
    return;
  }
  if (isActiveLog(source, path)) {
    fileServer.send(409, "text/plain", "Stop recording before renaming the active flight.");
    return;
  }
  if (!filesystem->exists(path)) {
    fileServer.send(404, "text/plain", "CSV file not found.");
    return;
  }
  if (newPath != path && filesystem->exists(newPath)) {
    fileServer.send(409, "text/plain", "A CSV with that name already exists.");
    return;
  }
  if (newPath != path && !filesystem->rename(path, newPath)) {
    fileServer.send(500, "text/plain", "Rename failed.");
    return;
  }
  if (source == "internal") {
    const int8_t slot = findInternalSlot(path);
    if (slot >= 0) saveInternalSlotPath(static_cast<uint8_t>(slot), newPath);
  }
  redirectToFileIndex();
}

void appendCsvFileCard(String &page, const String &path, size_t fileSize,
                       const char *sourceArgument) {
  const String displayName = path.substring(1);
  page += "<div class='file'><div class='meta'><strong>";
  page += htmlEscape(displayName);
  page += "</strong><span>";
  page += String(static_cast<unsigned long>(fileSize));
  page += " bytes</span></div><div class='actions'><a class='download' href='/download?src=";
  page += sourceArgument;
  page += "&file=";
  page += urlEncode(path);
  page += "' data-name='";
  page += htmlEscape(displayName);
  page += "' onclick='downloadFlight(event,this)'>Download</a>";
  page += "<form action='/rename' method='post'><input type='hidden' name='src' value='";
  page += sourceArgument;
  page += "'><input type='hidden' name='file' value='";
  page += htmlEscape(path);
  page += "'><input name='newName' value='";
  page += htmlEscape(displayName);
  page += "' maxlength='48' aria-label='New filename'><button type='submit'>Rename</button></form>";
  page += "<form action='/delete' method='post' onsubmit=\"return confirm('Delete this flight?')\">";
  page += "<input type='hidden' name='src' value='";
  page += sourceArgument;
  page += "'><input type='hidden' name='file' value='";
  page += htmlEscape(path);
  page += "'><button class='delete' type='submit'>Delete</button></form></div></div>";
}

void appendInternalCsvFiles(String &page) {
  page += "<h2>Internal flash (five rotating slots)</h2><div class='files'>";
  bool found = false;
  String listedPaths[INTERNAL_FLIGHT_SLOTS];
  uint8_t listedCount = 0;

  for (uint8_t slot = 0; slot < INTERNAL_FLIGHT_SLOTS; ++slot) {
    String path = internalSlotPath(slot);
    if (!FFat.exists(path)) {
      const String fallbackPath = defaultInternalSlotPath(slot);
      if (!FFat.exists(fallbackPath)) continue;
      path = fallbackPath;
    }

    bool duplicate = false;
    for (uint8_t index = 0; index < listedCount; ++index) {
      if (listedPaths[index] == path) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;

    File entry = FFat.open(path, FILE_READ);
    if (!entry || entry.isDirectory()) {
      if (entry) entry.close();
      continue;
    }
    appendCsvFileCard(page, path, entry.size(), "internal");
    entry.close();
    listedPaths[listedCount++] = path;
    found = true;
  }

  if (!found) page += "<p>No CSV flights saved yet.</p>";
  page += "</div>";
}

void appendCsvFiles(String &page, fs::FS &filesystem, const char *sourceLabel,
                    const char *sourceArgument) {
  page += "<h2>";
  page += sourceLabel;
  page += "</h2><div class='files'>";
  File root = filesystem.open("/");
  if (!root || !root.isDirectory()) {
    page += "<p>Storage is unavailable.</p></div>";
    return;
  }

  bool found = false;
  File entry = root.openNextFile();
  while (entry) {
    String path = entry.name();
    if (!path.startsWith("/")) {
      path = "/" + path;
    }
    String lowerPath = path;
    lowerPath.toLowerCase();
    if (!entry.isDirectory() && lowerPath.endsWith(".csv")) {
      found = true;
      appendCsvFileCard(page, path, entry.size(), sourceArgument);
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  if (!found) {
    page += "<p>No CSV flights saved yet.</p>";
  }
  page += "</div>";
}

void sendFileIndex() {
  wifiLastClientAt = millis();
  // Make an in-progress flight visible through Wi-Fi without losing the most
  // recent partial batch that is still buffered in RAM.
  if (pendingLogRows > 0) flushLogBatch();
  if (sdLogFile) sdLogFile.flush();

  String page;
  page.reserve(8000);
  page =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>XenoCam Flights</title><style>body{font-family:Arial,sans-serif;background:#07111f;"
    "color:#e5e7eb;margin:0;padding:20px;max-width:760px}h1{margin-bottom:4px}.note{color:#9ca3af}"
    ".files{display:grid;gap:10px}.file{display:flex;flex-direction:column;gap:10px;padding:14px;"
    "background:#172033;border:1px solid #334155;border-radius:9px}.meta{display:flex;justify-content:space-between;gap:10px}"
    ".meta span{color:#9ca3af}.actions{display:flex;gap:8px;flex-wrap:wrap}.actions form{display:flex;gap:6px;margin:0}"
    ".actions input{min-width:120px;background:#0f172a;color:#fff;border:1px solid #475569;border-radius:6px;padding:8px}"
    ".actions button,.download{border:0;border-radius:6px;padding:8px 10px;color:#fff;background:#2563eb;text-decoration:none;font:14px Arial}"
    ".actions .delete{background:#b91c1c}.off{display:inline-block;"
    "margin-top:22px;padding:11px 14px;border-radius:8px;background:#b91c1c;color:#fff;text-decoration:none}"
    "</style></head><body><h1>XenoCam flight files</h1><p class='note'>Storage mode: ";
  page += storageModeName();
  page += ". Stop recording before downloading for a final, complete CSV.</p>";
  if (internalReady) {
    // Scan FFat itself so a valid CSV can never be hidden by stale slot
    // preferences after a reset or rename.
    appendCsvFiles(page, FFat, "Internal flash (five rotating slots)", "internal");
  }
  if (sdReady) {
    appendCsvFiles(page, SD, "SD card", "sd");
  }
  page +=
    "<p id='downloadStatus' class='note'></p><a class='off' href='/off'>Turn Wi-Fi off</a>"
    "<script>function downloadFlight(e,a){if(!(window.AndroidBridge&&AndroidBridge.saveCsv))return;"
    "e.preventDefault();var s=document.getElementById('downloadStatus');s.textContent='Downloading '+a.dataset.name+'...';"
    "fetch(a.href).then(function(r){if(!r.ok)throw new Error('HTTP '+r.status);return r.text()})"
    ".then(function(csv){var saved=AndroidBridge.saveCsv(a.dataset.name,csv);s.textContent='Saved: '+saved})"
    ".catch(function(err){s.textContent='Download failed: '+err.message})}</script></body></html>";
  fileServer.send(200, "text/html", page);
}

void downloadCsv() {
  wifiLastClientAt = millis();
  // The downloaded file must contain every sample collected up to this request.
  if (pendingLogRows > 0) flushLogBatch();
  String source = fileServer.arg("src");
  String path = fileServer.arg("file");
  if (!path.startsWith("/")) {
    path = "/" + path;
  }
  if (!isSafeCsvPath(path)) {
    fileServer.send(400, "text/plain", "Invalid CSV path.");
    return;
  }

  fs::FS *filesystem = filesystemForSource(source);
  if (source == "sd" && sdReady) {
    if (sdLogFile) sdLogFile.flush();
  }
  if (filesystem == nullptr || !filesystem->exists(path)) {
    fileServer.send(404, "text/plain", "CSV file not found.");
    return;
  }

  File download = filesystem->open(path, FILE_READ);
  if (!download || download.isDirectory()) {
    fileServer.send(404, "text/plain", "CSV file could not be opened.");
    return;
  }
  String filename = path.substring(path.lastIndexOf('/') + 1);
  fileServer.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  fileServer.sendHeader("Cache-Control", "no-store");
  fileServer.streamFile(download, "text/csv");
  download.close();
  wifiLastClientAt = millis();
  Serial.printf("Downloaded %s; Wi-Fi remains on.\n", filename.c_str());
}

void stopWifi() {
  if (!wifiActive) {
    return;
  }
  fileServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiActive = false;
  wifiStopRequested = false;
  Serial.println("Wi-Fi off.");
}

bool startWifi() {
  if (wifiActive) {
    wifiLastClientAt = millis();
    return true;
  }

  const IPAddress address(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  if (!WiFi.softAPConfig(address, gateway, subnet) ||
      !WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, false, 2)) {
    Serial.println("WIFI ERROR: access point failed to start.");
    WiFi.mode(WIFI_OFF);
    return false;
  }
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  static bool routesConfigured = false;
  if (!routesConfigured) {
    fileServer.on("/", HTTP_GET, sendFileIndex);
    fileServer.on("/download", HTTP_GET, downloadCsv);
    fileServer.on("/rename", HTTP_POST, renameCsv);
    fileServer.on("/delete", HTTP_POST, deleteCsv);
    fileServer.on("/off", HTTP_GET, []() {
      fileServer.send(200, "text/html", "<h2>Wi-Fi is turning off.</h2><p>You may close this page.</p>");
      wifiStopRequested = true;
    });
    fileServer.onNotFound([]() {
      wifiLastClientAt = millis();
      fileServer.send(404, "text/plain", "Not found. Open http://192.168.4.1/");
    });
    routesConfigured = true;
  }
  fileServer.begin();
  wifiStartedAt = millis();
  wifiLastClientAt = wifiStartedAt;
  wifiActive = true;
  wifiStopRequested = false;
  Serial.printf("Wi-Fi on: connect to %s and open http://192.168.4.1/\n", WIFI_SSID);
  return true;
}

void serviceWifi(uint32_t now) {
  if (!wifiActive) {
    return;
  }
  fileServer.handleClient();
  if (wifiStopRequested) {
    stopWifi();
    return;
  }

  static uint32_t lastTemperatureCheckAt = 0;
  if (now - lastTemperatureCheckAt >= 5000) {
    lastTemperatureCheckAt = now;
    const float chipTemperatureC = temperatureRead();
    if (isfinite(chipTemperatureC) && chipTemperatureC >= WIFI_THERMAL_CUTOFF_C) {
      Serial.printf("Wi-Fi thermal cutoff at %.1f C.\n", chipTemperatureC);
      stopWifi();
    }
  }
}

void applyCommand(String command) {
  command.trim();
  if (!command.startsWith("CMD:")) {
    return;
  }
  Serial.print("Command: ");
  Serial.println(command);

  if (command.startsWith("CMD:START_RECORD") || command.startsWith("CMD:START_REC")) {
    if (command.endsWith(",INTERNAL")) {
      setStorageMode(StorageMode::INTERNAL);
    } else if (command.endsWith(",SD")) {
      setStorageMode(StorageMode::SD_CARD);
    } else if (command.endsWith(",BOTH")) {
      setStorageMode(StorageMode::BOTH);
    }
    startRecording();
  } else if (command == "CMD:STOP_RECORD" || command == "CMD:STOP_REC") {
    stopRecording();
    // Repeat stopped telemetry only after the half-duplex receiver finishes
    // sending its three command copies and returns to receive mode.
    pendingStopAcks = 3;
    nextStopAckAt = millis() + 700;
  } else if (command == "CMD:STORAGE,INTERNAL") {
    setStorageMode(StorageMode::INTERNAL);
  } else if (command == "CMD:STORAGE,SD") {
    setStorageMode(StorageMode::SD_CARD);
  } else if (command == "CMD:STORAGE,BOTH") {
    setStorageMode(StorageMode::BOTH);
  } else if (command == "CMD:WIFI_ON") {
    startWifi();
  } else if (command == "CMD:WIFI_OFF") {
    stopWifi();
  } else if (command == "CMD:ZERO_ALT") {
    zeroAltitude();
  } else if (command == "CMD:ZERO_ACCEL") {
    zeroAccelerometers();
  } else if (command == "CMD:ZERO_ALL") {
    zeroAltitude();
    zeroAccelerometers();
  } else if (command == "CMD:RESET_FLIGHT") {
    flightState = 0;
  } else if (command.startsWith("CMD:RENAME,")) {
    const int comma = command.lastIndexOf(',');
    if (comma >= 0 && comma + 1 < static_cast<int>(command.length())) {
      String requested = command.substring(comma + 1);
      requested.trim();
      if (requested.length() > 0) {
        requested.remove(12);
        rocketName = requested;
      }
    }
  }
}

void receiveRadioCommand() {
  if (!loraReady || loraTxInProgress) {
    return;
  }
  const int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  String command;
  command.reserve(packetSize);
  while (LoRa.available()) {
    command += static_cast<char>(LoRa.read());
  }
  applyCommand(command);
  LoRa.receive();
}

void serviceRadioTransmit() {
  if (!loraReady || !loraTxInProgress) {
    return;
  }
  if (loraTxDone) {
    loraTxDone = false;
    loraTxInProgress = false;
    LoRa.onTxDone(nullptr);
    LoRa.receive();
  }
}

bool transmitTelemetry() {
  if (!loraReady || loraTxInProgress) {
    return false;
  }

  String packet;
  packet.reserve(240);
  packet = "RT2,";
  appendSensorCsvFields(packet);
  packet += ",";
  packet += String(flightState) + ",";
  packet += recording ? "1," : "0,";
  packet += String(recordsWritten) + ",";
  packet += String(static_cast<unsigned long long>(sdFreeKB)) + ",";
  packet += ROCKET_ID;
  packet += ",";
  packet += rocketName;

  if (packet.length() > 255) {
    Serial.printf("LORA ERROR: telemetry packet is %u bytes.\n",
                  static_cast<unsigned>(packet.length()));
    return false;
  }

  digitalWrite(SD_CS_PIN, HIGH);
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(packet);
  // Async transmit keeps GPS, sensors, and the 20 ms internal logger running
  // during the comparatively long 915 MHz LoRa packet airtime.
  loraTxDone = false;
  LoRa.onTxDone(onRadioTxDone);
  LoRa.endPacket(true);
  loraTxInProgress = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 3000) {
    delay(10);
  }
  Serial.println("Xeno payload production telemetry starting.");
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setRecordingLed(false);
  WiFi.mode(WIFI_OFF);
  preferences.begin("xenoPayload", false);
  const uint8_t savedStorageMode = preferences.getUChar(
    "storage", static_cast<uint8_t>(StorageMode::BOTH));
  storageMode = savedStorageMode <= static_cast<uint8_t>(StorageMode::BOTH)
                  ? static_cast<StorageMode>(savedStorageMode)
                  : StorageMode::BOTH;
  Serial.printf("Saved storage mode: %s\n", storageModeName());

  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(LORA_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(LORA_CS_PIN, HIGH);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, LORA_CS_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  imuReady = initializeImu();
  bmeReady = bme.begin(BME_SENSOR_ADDRESS, &Wire);
  Serial.println(bmeReady ? "BME280 ready." : "BME280 ERROR: not detected at 0x76.");

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS listening at 9600 baud on D0/D1.");

  initializeStorage();
  loraReady = initializeRadio();

  if (bmeReady) {
    zeroAltitude();
  }
  if (imuReady) {
    zeroAccelerometers();
  }
  Serial.println("Recorder idle: waiting for START_RECORD.");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid() && gps.location.age() < 5000) {
    sample.latitude = gps.location.lat();
    sample.longitude = gps.location.lng();
  }
  if (gps.satellites.isValid() && gps.satellites.age() < 5000) {
    sample.satellites = static_cast<uint8_t>(
      min(static_cast<uint32_t>(gps.satellites.value()), static_cast<uint32_t>(255)));
  }

  serviceRadioTransmit();
  receiveRadioCommand();
  sample.timeMs = millis();
  const uint32_t now = sample.timeMs;
  serviceWifi(now);

  static uint32_t lastSensorAt = 0;
  static uint32_t lastBarometerAt = 0;
  static uint32_t lastLogAt = 0;
  static uint32_t lastFlushAt = 0;
  static uint32_t lastTelemetryAt = 0;
  static uint32_t lastHealthAt = 0;
  static uint32_t lastFreeSpaceAt = 0;

  if (now - lastSensorAt >= SENSOR_INTERVAL_MS) {
    lastSensorAt = now;
    updateImu();
  }
  if (now - lastBarometerAt >= BAROMETER_INTERVAL_MS) {
    lastBarometerAt = now;
    updateBarometer();
  }
  if (now - lastLogAt >= LOG_INTERVAL_MS) {
    lastLogAt = now;
    writeRecord();
  }
  if (recording && now - lastFlushAt >= LOG_BATCH_MAX_AGE_MS) {
    lastFlushAt = now;
    flushLogBatch();
  }
  if (sdReady && now - lastFreeSpaceAt >= 5000) {
    lastFreeSpaceAt = now;
    sdFreeKB = (SD.totalBytes() - SD.usedBytes()) / 1024ULL;
  }
  if (pendingStopAcks > 0 && static_cast<int32_t>(now - nextStopAckAt) >= 0) {
    if (transmitTelemetry()) {
      lastTelemetryAt = now;
      pendingStopAcks--;
      nextStopAckAt = millis() + 900;
    }
  } else if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS) {
    if (transmitTelemetry()) {
      lastTelemetryAt = now;
    }
  }
  if (now - lastHealthAt >= HEALTH_INTERVAL_MS) {
    lastHealthAt = now;
    Serial.printf(
      "HEALTH imu=%s bme=%s internal=%s sd=%s mode=%s rec=%s wifi=%s lora=%s gpsChars=%lu sats=%u alt=%.2f records=%lu\n",
      imuReady ? "OK" : "FAIL",
      bmeReady ? "OK" : "FAIL",
      internalReady ? "OK" : "FAIL",
      sdReady ? "OK" : "FAIL",
      storageModeName(),
      recording ? "ON" : "OFF",
      wifiActive ? "ON" : "OFF",
      loraReady ? "OK" : "FAIL",
      static_cast<unsigned long>(gps.charsProcessed()),
      sample.satellites,
      sample.altitudeM,
      static_cast<unsigned long>(recordsWritten));
  }

  delay(1);
}
