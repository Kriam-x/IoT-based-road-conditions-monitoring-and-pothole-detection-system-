/*
 * Road Surface Condition Detection and Mapping System
 * ESP32 DevKit V1 (ESP32-WROOM-32) firmware  -  Arduino IDE
 *
 * What it does
 *   1. Reads the MPU6050 at 100 Hz.
 *   2. Calibrates at start-up (gravity direction + gyro offset), so the sensor
 *      can be mounted in any orientation as long as it is fixed rigidly.
 *   3. Detects impact events in a short time window and classifies them as
 *      pothole / bump / rough_road using peak values, the order of the peaks
 *      and the gyroscope.
 *   4. Attaches GPS position, speed and time, and queues the event.
 *   5. A second FreeRTOS task uploads queued events over Wi-Fi (HTTP POST, JSON).
 *      If upload fails, the event is saved on the SD card and uploaded later.
 *
 * Libraries needed (Library Manager):  TinyGPSPlus (by Mikal Hart)
 * Everything else (Wire, WiFi, HTTPClient, SD, SPI) comes with the ESP32 board package.
 * Board: "ESP32 Dev Module"   Baud for Serial Monitor: 115200
 *
 * Serial commands:  send  c  to re-calibrate (keep the device still).
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ======================================================================
// 1. USER CONFIGURATION  -  EDIT THESE
// ======================================================================
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Address of your backend. Use http://<PC-IP>:3000/api/events on a local network,
// or https://your-app.example.com/api/events when deployed.
const char* SERVER_URL    = "http://192.168.1.50:3000/api/events";
const char* API_KEY       = "change-this-secret-key";   // must match backend API_KEY
const char* DEVICE_ID     = "esp32-road-01";

// Optional features (true / false)
const bool USE_SD_CARD = true;    // offline queue on MicroSD
const bool USE_BUZZER  = true;    // beep when an event is detected
const bool USE_LED     = true;    // status LED

// Bench testing indoors: if there is no GPS fix, use the fixed position below
// instead of discarding the event. Set to false for real road use.
const bool   TEST_FAKE_LOCATION = false;
const double FAKE_LAT = 28.6139;
const double FAKE_LNG = 77.2090;

// true = print "hp,gyro" every sample for Arduino IDE Serial Plotter (tuning)
const bool DEBUG_STREAM_SAMPLES = false;

// ======================================================================
// 2. PIN MAP  (must match the wiring diagram)
// ======================================================================
#define I2C_SDA     21    // MPU6050 SDA
#define I2C_SCL     22    // MPU6050 SCL
#define GPS_RX_PIN  16    // ESP32 RX2  <-  NEO-6M TX
#define GPS_TX_PIN  17    // ESP32 TX2  ->  NEO-6M RX
#define SD_CS        5
#define SD_MOSI     23
#define SD_MISO     19
#define SD_SCK      18
#define BUZZER_PIN  25    // active buzzer, HIGH = on
#define LED_PIN     26    // LED through 220 ohm, HIGH = on

// ======================================================================
// 3. DETECTION SETTINGS  (tune these - see README "Tuning")
// ======================================================================
const uint32_t SAMPLE_INTERVAL_MS = 10;     // 10 ms = 100 Hz sampling
const uint16_t CALIBRATION_SAMPLES = 300;   // 3 s at 100 Hz

const float HP_ALPHA = 0.02f;               // baseline tracking speed (high-pass filter)
const float TRIGGER_G = 0.35f;              // |vertical accel change| that opens a window
const uint32_t WINDOW_MS = 300;             // observation window after trigger
const uint32_t COOLDOWN_MS = 800;           // dead time after a window (avoid ringing)

const float MIN_P2P_G = 0.70f;              // min peak-to-peak (g) to count as an impact at all
const float POTHOLE_P2P_G = 0.90f;          // min peak-to-peak (g) for a pothole
const float GYRO_CONFIRM_DPS = 8.0f;        // min tilt-rate (pitch/roll) to confirm real impact; 0 = off

// Rough road = several impacts close together
const uint8_t  ROUGH_MIN_EVENTS = 3;        // impacts needed ...
const uint32_t ROUGH_PERIOD_MS = 5000;      // ... within this period
const uint32_t ROUGH_REPORT_GAP_MS = 10000; // report rough road at most this often

// Severity labels are based on impact strength (peak-to-peak g). NOT pothole depth.
const float SEV_MEDIUM_P2P_G = 1.5f;
const float SEV_HIGH_P2P_G = 2.5f;

// Ignore events when slower than this (km/h). 0 = disabled (use 0 for bench tests,
// 8-10 for real driving so engine idling / handling is ignored).
const float MIN_SPEED_KMPH = 0.0f;

const uint32_t GPS_MAX_AGE_MS = 2500;       // GPS fix older than this is "stale"

// Network / SD
const uint32_t WIFI_RETRY_MS = 10000;
const uint32_t HTTP_TIMEOUT_MS = 6000;
const uint32_t FLUSH_INTERVAL_MS = 20000;   // try to upload SD backlog this often
const uint8_t  FLUSH_MAX_PER_CYCLE = 10;    // upload at most this many backlog events per try
const char* PENDING_FILE = "/pending.txt";
const char* PENDING_TMP  = "/pending.tmp";

// ======================================================================
// 4. TYPES AND GLOBALS
// ======================================================================
struct Imu      { float ax, ay, az, gx, gy, gz; };   // accel in g, gyro in deg/s
struct GpsSnap  { bool fix; double lat; double lng; float speed; int sats; float hdop; char time[24]; };
struct EventMsg { char json[400]; };                 // one JSON payload in the queue

TinyGPSPlus gps;                 // NMEA parser
HardwareSerial gpsSerial(2);     // UART2 for the GPS

const int MPU_ADDR = 0x68;
const float ACCEL_LSB_PER_G = 4096.0f;   // +-8 g range
const float GYRO_LSB_PER_DPS = 65.5f;    // +-500 deg/s range

// Calibration results
float upX = 0, upY = 0, upZ = 1;         // unit vector pointing "up" (direction of gravity reading)
float gRest = 1.0f;                      // measured gravity magnitude at rest (g)
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float baseline = 0;                      // slow moving average of vertical accel (high-pass)

// Detection state machine
enum DetectState { ST_IDLE, ST_WINDOW, ST_COOLDOWN };
DetectState state = ST_IDLE;
uint32_t winStartMs = 0, cooldownStartMs = 0;
float winMax = 0, winMin = 0, winGyroPeak = 0;
int winFirstSign = 0;                    // +1 = up first (bump-like), -1 = down first (pothole-like)
GpsSnap winGps;                          // GPS snapshot taken at the moment of trigger

const uint8_t RECENT_N = 6;
uint32_t recentTimes[RECENT_N] = {0};    // times of recent valid impacts (for rough-road logic)
uint8_t recentIdx = 0;
uint32_t lastRoughReport = 0;

uint32_t lastSampleMs = 0, lastStatusMs = 0, ledFlashUntil = 0;
uint8_t imuFailCount = 0;

QueueHandle_t eventQueue;
bool sdOk = false;

// Non-blocking buzzer
struct Beeper { uint8_t left; uint16_t onMs; uint16_t offMs; bool on; uint32_t t; };
Beeper beeper = {0, 0, 0, false, 0};

// ======================================================================
// 5. SMALL HELPERS
// ======================================================================
void feedGps() {                         // move bytes from the GPS UART into the parser
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
}

bool gpsFixValid() {                     // true only for a recent, valid position
  return gps.location.isValid() && gps.location.age() < GPS_MAX_AGE_MS;
}

bool speedAllowed() {                    // speed gate: skip events when parked / too slow
  if (MIN_SPEED_KMPH <= 0.0f) return true;
  return gps.speed.isValid() && gps.speed.age() < 2000 && gps.speed.kmph() >= MIN_SPEED_KMPH;
}

GpsSnap takeGpsSnapshot() {              // copy the current GPS data into one struct
  GpsSnap g;
  g.fix = gpsFixValid();
  g.lat = g.fix ? gps.location.lat() : 0.0;
  g.lng = g.fix ? gps.location.lng() : 0.0;
  g.speed = gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f;
  g.sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  g.hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 0.0f;
  g.time[0] = '\0';
  if (gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2024) {
    snprintf(g.time, sizeof(g.time), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             (int)gps.date.year(), (int)gps.date.month(), (int)gps.date.day(),
             (int)gps.time.hour(), (int)gps.time.minute(), (int)gps.time.second());
  }
  return g;
}

void startBeeps(uint8_t count, uint16_t onMs, uint16_t offMs) {
  if (!USE_BUZZER) return;
  digitalWrite(BUZZER_PIN, LOW);
  beeper.left = count; beeper.onMs = onMs; beeper.offMs = offMs;
  beeper.on = false;
  beeper.t = millis() - offMs;           // so the first beep starts immediately
}

void updateBeeper() {                    // call every loop; never blocks
  if (!USE_BUZZER) return;
  uint32_t now = millis();
  if (beeper.on) {
    if (now - beeper.t >= beeper.onMs) {
      digitalWrite(BUZZER_PIN, LOW);
      beeper.on = false; beeper.t = now;
      if (beeper.left > 0) beeper.left--;
    }
  } else if (beeper.left > 0 && now - beeper.t >= beeper.offMs) {
    digitalWrite(BUZZER_PIN, HIGH);
    beeper.on = true; beeper.t = now;
  }
}

void updateLed() {
  if (!USE_LED) return;
  uint32_t now = millis();
  bool on;
  if (now < ledFlashUntil) on = true;                  // event flash
  else if (gpsFixValid()) on = true;                   // solid = GPS fix OK
  else on = (now / 500) % 2;                           // blinking = waiting for GPS fix
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// ======================================================================
// 6. MPU6050 (raw I2C, no library needed)
// ======================================================================
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool initMpu() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);                                    // WHO_AM_I register
  if (Wire.endTransmission(false) != 0) return false;  // no ACK = wiring problem
  if (Wire.requestFrom(MPU_ADDR, 1, true) != 1) return false;
  uint8_t who = Wire.read();
  Serial.printf("[IMU] WHO_AM_I = 0x%02X\n", who);
  if (who == 0x00 || who == 0xFF) return false;
  if (who != 0x68) Serial.println("[IMU] Unusual ID (clone or MPU6500?) - continuing anyway");

  if (!mpuWrite(0x6B, 0x00)) return false;             // wake up
  delay(100);
  if (!mpuWrite(0x1C, 0x10)) return false;             // accel range +-8 g
  if (!mpuWrite(0x1B, 0x08)) return false;             // gyro range +-500 deg/s
  if (!mpuWrite(0x1A, 0x03)) return false;             // low-pass filter ~44 Hz (noise filtering)
  return true;
}

bool readImu(Imu &s) {                                 // read accel + gyro in one 14-byte burst
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  s.ax = v[0] / ACCEL_LSB_PER_G;
  s.ay = v[1] / ACCEL_LSB_PER_G;
  s.az = v[2] / ACCEL_LSB_PER_G;
  // v[3] = temperature (unused)
  s.gx = v[4] / GYRO_LSB_PER_DPS;
  s.gy = v[5] / GYRO_LSB_PER_DPS;
  s.gz = v[6] / GYRO_LSB_PER_DPS;
  return true;
}

// Average ~3 s of readings while the vehicle is still. Gives the "up" direction,
// the gravity magnitude and the gyro offset. Returns false if the device was moving.
bool calibrate() {
  Serial.println("[CAL] Keep the device still for about 3 seconds...");
  double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
  float magMin = 100.0f, magMax = 0.0f;
  uint16_t n = 0;
  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    Imu s;
    if (readImu(s)) {
      sax += s.ax; say += s.ay; saz += s.az;
      sgx += s.gx; sgy += s.gy; sgz += s.gz;
      float mag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
      if (mag < magMin) magMin = mag;
      if (mag > magMax) magMax = mag;
      n++;
    }
    if (USE_LED) digitalWrite(LED_PIN, (i / 25) % 2);
    feedGps();
    delay(SAMPLE_INTERVAL_MS);
  }
  if (n < CALIBRATION_SAMPLES / 2) { Serial.println("[CAL] FAILED: too many IMU read errors"); return false; }

  float mx = sax / n, my = say / n, mz = saz / n;
  gRest = sqrtf(mx * mx + my * my + mz * mz);
  if (gRest < 0.5f) { Serial.println("[CAL] FAILED: gravity reading too small"); return false; }
  upX = mx / gRest; upY = my / gRest; upZ = mz / gRest;
  gyroBiasX = sgx / n; gyroBiasY = sgy / n; gyroBiasZ = sgz / n;
  baseline = 0;

  bool still = (magMax - magMin) < 0.25f;
  Serial.printf("[CAL] gravity=%.3f g  up=(%.2f,%.2f,%.2f)  gyroBias=(%.2f,%.2f,%.2f) dps  %s\n",
                gRest, upX, upY, upZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                still ? "OK" : "WARNING: device moved during calibration");
  return still;
}

// ======================================================================
// 7. NETWORK + SD (runs in its own task on core 0, so sampling never stalls)
// ======================================================================
bool postJson(const char* json) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClient plain;
  WiFiClientSecure secure;
  HTTPClient http;
  bool began;
  if (strncmp(SERVER_URL, "https://", 8) == 0) {
    secure.setInsecure();      // skips certificate check - fine for a college project
    began = http.begin(secure, SERVER_URL);
  } else {
    began = http.begin(plain, SERVER_URL);
  }
  if (!began) return false;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", API_KEY);
  int code = http.POST((uint8_t*)json, strlen(json));
  http.end();
  Serial.printf("[NET] POST -> HTTP %d\n", code);
  if (code >= 200 && code < 300) return true;
  if (code == 400 || code == 422) {                    // server says the data itself is bad: retrying is pointless
    Serial.println("[NET] Server rejected the data - discarding it");
    return true;
  }
  return false;                                        // network error, 401/403 (wrong key), 5xx ... keep for retry
}

bool saveToSD(const char* json) {
  if (!sdOk) { Serial.println("[SD] not available - event lost"); return false; }
  File f = SD.open(PENDING_FILE, FILE_APPEND);
  if (!f) { Serial.println("[SD] cannot open pending file"); return false; }
  f.println(json);
  f.close();
  Serial.println("[SD] event saved for later upload");
  return true;
}

void flushPending() {                                  // upload backlog from the SD card
  if (!sdOk || !SD.exists(PENDING_FILE)) return;
  File in = SD.open(PENDING_FILE, FILE_READ);
  if (!in) return;
  SD.remove(PENDING_TMP);
  File tmp = SD.open(PENDING_TMP, FILE_WRITE);
  if (!tmp) { in.close(); return; }

  uint8_t sent = 0;
  bool stop = false;
  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() < 5) continue;
    if (!stop && sent < FLUSH_MAX_PER_CYCLE) {
      if (postJson(line.c_str())) { sent++; continue; }
      stop = true;                                     // server unreachable: stop trying this round
    }
    tmp.println(line);                                 // keep unsent lines
  }
  in.close(); tmp.close();
  SD.remove(PENDING_FILE);

  File check = SD.open(PENDING_TMP, FILE_READ);
  bool empty = !check || check.size() == 0;
  if (check) check.close();
  if (empty) SD.remove(PENDING_TMP); else SD.rename(PENDING_TMP, PENDING_FILE);
  Serial.printf("[SD] backlog upload: %u sent, backlog %s\n", sent, empty ? "empty" : "remaining");
}

void networkTask(void* param) {
  uint32_t lastWifiTry = 0, lastFlush = 0;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > WIFI_RETRY_MS) {
      lastWifiTry = millis();
      Serial.println("[WIFI] disconnected - retrying");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    EventMsg msg;
    if (xQueueReceive(eventQueue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
      if (postJson(msg.json)) Serial.println("[NET] event uploaded");
      else { Serial.println("[NET] upload failed"); saveToSD(msg.json); }
    } else if (USE_SD_CARD && sdOk && WiFi.status() == WL_CONNECTED &&
               millis() - lastFlush > FLUSH_INTERVAL_MS) {
      lastFlush = millis();
      flushPending();
    }
  }
}

// ======================================================================
// 8. EVENT REPORTING
// ======================================================================
void reportEvent(const char* type, float p2p, const GpsSnap &g) {
  const char* severity = p2p >= SEV_HIGH_P2P_G ? "high" : (p2p >= SEV_MEDIUM_P2P_G ? "medium" : "low");
  EventMsg msg;
  snprintf(msg.json, sizeof(msg.json),
    "{\"device_id\":\"%s\",\"event_type\":\"%s\",\"severity\":\"%s\","
    "\"latitude\":%.6f,\"longitude\":%.6f,\"gps_fix\":%s,\"speed_kmph\":%.1f,"
    "\"satellites\":%d,\"hdop\":%.1f,\"device_time\":\"%s\","
    "\"max_g\":%.2f,\"min_g\":%.2f,\"p2p_g\":%.2f,\"gyro_dps\":%.1f}",
    DEVICE_ID, type, severity, g.lat, g.lng, g.fix ? "true" : "false", g.speed,
    g.sats, g.hdop, g.time, winMax, winMin, p2p, winGyroPeak);

  Serial.printf("[EVENT] %s (%s)  lat=%.6f lng=%.6f  speed=%.1f km/h  p2p=%.2f g\n",
                type, severity, g.lat, g.lng, g.speed, p2p);

  if (xQueueSend(eventQueue, &msg, 0) != pdTRUE) Serial.println("[EVENT] queue full - event dropped");

  ledFlashUntil = millis() + 400;
  if (strcmp(type, "pothole") == 0) startBeeps(3, 80, 80);
  else if (strcmp(type, "bump") == 0) startBeeps(2, 80, 80);
  else startBeeps(1, 400, 0);
}

// ======================================================================
// 9. DETECTION ALGORITHM
// ======================================================================
void evaluateWindow(uint32_t now) {
  float p2p = winMax - winMin;                         // peak-to-peak change in vertical accel
  Serial.printf("[DETECT] window: max=%.2f min=%.2f p2p=%.2f gyro=%.1f dps first=%s\n",
                winMax, winMin, p2p, winGyroPeak, winFirstSign < 0 ? "DOWN" : "UP");

  // 1) Reject weak or "no rotation" windows (electrical noise, handling, door slam ...)
  if (p2p < MIN_P2P_G)              { Serial.println("  -> ignored: too weak"); return; }
  if (winGyroPeak < GYRO_CONFIRM_DPS) { Serial.println("  -> ignored: no body rotation"); return; }

  // 2) Remember this impact; several close together = rough road
  recentTimes[recentIdx] = now;
  recentIdx = (recentIdx + 1) % RECENT_N;
  uint8_t recent = 0;
  for (uint8_t i = 0; i < RECENT_N; i++)
    if (recentTimes[i] != 0 && now - recentTimes[i] <= ROUGH_PERIOD_MS) recent++;

  const char* type = nullptr;
  if (recent >= ROUGH_MIN_EVENTS) {
    if (lastRoughReport == 0 || now - lastRoughReport >= ROUGH_REPORT_GAP_MS) {
      type = "rough_road";
      lastRoughReport = now;
    } else {
      Serial.println("  -> part of an already reported rough stretch");
      return;
    }
  }
  // 3) Single impact: the ORDER of the peaks tells pothole from bump
  //    pothole: wheel drops (reading goes DOWN first), then hits the far edge (UP)
  //    bump   : wheel rises (reading goes UP first), then drops (DOWN)
  else if (winFirstSign < 0 && p2p >= POTHOLE_P2P_G) type = "pothole";
  else if (winFirstSign > 0) type = "bump";
  else { Serial.println("  -> ignored: down-first but below pothole threshold"); return; }

  GpsSnap g = winGps;
  if (!g.fix) {
    if (TEST_FAKE_LOCATION) { g.lat = FAKE_LAT; g.lng = FAKE_LNG; Serial.println("[GPS] no fix - using FAKE test location"); }
    else { Serial.println("[GPS] no fix - event discarded"); return; }
  }
  reportEvent(type, p2p, g);
}

void processSample(const Imu &s, uint32_t now) {
  // Vertical acceleration relative to rest (g): project onto the calibrated "up" axis
  float vert = (s.ax * upX + s.ay * upY + s.az * upZ) - gRest;

  // High-pass: subtract slow baseline so slopes / drift do not trigger anything
  float hp = vert - baseline;
  if (state != ST_WINDOW) baseline += HP_ALPHA * (vert - baseline);

  // Gyro: remove bias, then remove the component around the vertical axis
  // (that is turning left/right). What remains is pitch/roll rate = suspension movement.
  float gx = s.gx - gyroBiasX, gy = s.gy - gyroBiasY, gz = s.gz - gyroBiasZ;
  float along = gx * upX + gy * upY + gz * upZ;
  float hx = gx - along * upX, hy = gy - along * upY, hz = gz - along * upZ;
  float gyroH = sqrtf(hx * hx + hy * hy + hz * hz);

  if (DEBUG_STREAM_SAMPLES)
    Serial.printf("hp:%.3f,gyro:%.1f,trig:%.2f,ntrig:%.2f\n", hp, gyroH, TRIGGER_G, -TRIGGER_G);

  switch (state) {
    case ST_IDLE:
      if (fabsf(hp) > TRIGGER_G && speedAllowed()) {
        winStartMs = now;
        winMax = winMin = hp;
        winGyroPeak = gyroH;
        winFirstSign = hp > 0 ? 1 : -1;
        winGps = takeGpsSnapshot();                    // GPS position at the moment of impact
        state = ST_WINDOW;
      }
      break;

    case ST_WINDOW:
      if (hp > winMax) winMax = hp;
      if (hp < winMin) winMin = hp;
      if (gyroH > winGyroPeak) winGyroPeak = gyroH;
      if (now - winStartMs >= WINDOW_MS) {
        evaluateWindow(now);
        state = ST_COOLDOWN;
        cooldownStartMs = now;
      }
      break;

    case ST_COOLDOWN:
      if (now - cooldownStartMs >= COOLDOWN_MS) state = ST_IDLE;
      break;
  }
}

// ======================================================================
// 10. SETUP / LOOP
// ======================================================================
void printStatus() {
  Serial.printf("[STATUS] GPS: %s sats=%d speed=%.1f km/h (NMEA chars=%lu) | WiFi: %s | SD: %s | queued=%u\n",
                gpsFixValid() ? "FIX" : "no fix",
                gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
                gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f,
                (unsigned long)gps.charsProcessed(),
                WiFi.status() == WL_CONNECTED ? "connected" : "OFFLINE",
                sdOk ? "ok" : "not available",
                (unsigned)uxQueueMessagesWaiting(eventQueue));
  if (gps.charsProcessed() < 10 && millis() > 8000)
    Serial.println("[STATUS] No data from GPS - check wiring: GPS TX -> GPIO16, GPS RX -> GPIO17");
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') { state = ST_IDLE; calibrate(); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Road Surface Monitor ===");

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);    digitalWrite(LED_PIN, LOW);

  // --- MPU6050: retry until found (fast LED blink = problem) ---
  while (!initMpu()) {
    Serial.println("[IMU] MPU6050 not found. Check SDA->GPIO21, SCL->GPIO22, VCC->3V3, GND. Retrying...");
    for (int i = 0; i < 10; i++) { digitalWrite(LED_PIN, i % 2); delay(100); }
    delay(1000);
  }
  Serial.println("[IMU] MPU6050 ready");

  // --- GPS ---
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] UART2 started at 9600 baud (first fix can take several minutes outdoors)");

  // --- Calibration (up to 3 tries) ---
  for (int attempt = 0; attempt < 3; attempt++) if (calibrate()) break;

  // --- SD card (optional) ---
  if (USE_SD_CARD) {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdOk = SD.begin(SD_CS, SPI) && SD.cardType() != CARD_NONE;
    Serial.println(sdOk ? "[SD] card ready" : "[SD] card NOT found - offline logging disabled");
  }

  // --- Wi-Fi (wait up to 10 s, then continue offline) ---
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WIFI] connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(250); Serial.print("."); feedGps(); }
  if (WiFi.status() == WL_CONNECTED) Serial.printf("\n[WIFI] connected, IP %s\n", WiFi.localIP().toString().c_str());
  else Serial.println("\n[WIFI] not connected - running offline, will keep retrying");

  // --- Network task on core 0; sampling stays on core 1 (Arduino loop) ---
  eventQueue = xQueueCreate(10, sizeof(EventMsg));
  xTaskCreatePinnedToCore(networkTask, "net", 12288, NULL, 1, NULL, 0);

  startBeeps(1, 150, 0);
  Serial.println("[READY] Monitoring road surface...  (send 'c' to recalibrate)");
}

void loop() {
  feedGps();
  handleSerialCommands();
  updateBeeper();
  updateLed();

  uint32_t now = millis();
  if (!DEBUG_STREAM_SAMPLES && now - lastStatusMs >= 5000) { lastStatusMs = now; printStatus(); }

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;   // 100 Hz sampling
  lastSampleMs = now;

  Imu s;
  if (!readImu(s)) {                                      // sensor read error handling
    if (++imuFailCount >= 25) {
      Serial.println("[IMU] repeated read errors - re-initialising sensor");
      initMpu();
      imuFailCount = 0;
    }
    return;
  }
  imuFailCount = 0;
  processSample(s, now);
}
