// ESP8266 Sender: QMC5883L -> deltaYaw -> (dir,steps) over UDP
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

// ===== WiFi =====
const char* SSID = "concord.hippo_5ghz";
const char* PASS = "n!/(n-k)!";

// ===== Receiver (Nano 33 IoT) =====
IPAddress RECEIVER_IP(192,168,1,142);   // 換成 Nano 33 IoT 串口印出的 IP
const uint16_t UDP_PORT = 9000;

WiFiUDP udp;
QMC5883LCompass compass;

float previousYaw = 0.0f;
const float STEP_ANGLE_DEG = 1.8f;  // A4988 全步 1.8°
const float MAX_DELTA = 90.0f;      // 一次最多修正 ±90°
const float THRESHOLD = 3.0f;       // 小於 3° 就不送，防抖
const unsigned SEND_INTERVAL_MS = 100; // 10Hz

float getYawDeg() {
  compass.read();
  int x = compass.getX();
  int z = compass.getZ();
  float yaw = atan2((float)x, (float)z) * 180.0f / PI;
  if (yaw < 0) yaw += 360.0f;
  return yaw;
}

void setup() {
  Serial.begin(115200);
  // I2C
  Wire.begin(4, 5); // SDA=D2, SCL=D1
  delay(200);
  compass.init();
  // WiFi
  WiFi.begin(SSID, PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP8266 IP: "); Serial.println(WiFi.localIP());

  previousYaw = getYawDeg();
}

void loop() {
  static unsigned long t0 = 0;
  if (millis() - t0 < SEND_INTERVAL_MS) return;
  t0 = millis();

  float currentYaw = getYawDeg();
  float delta = currentYaw - previousYaw;
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;

  // 防極端 & 防抖
  if (fabs(delta) < THRESHOLD) return;
  if (delta >  MAX_DELTA) delta =  MAX_DELTA;
  if (delta < -MAX_DELTA) delta = -MAX_DELTA;

  int direction = (delta > 0) ? 1 : -1;
  int steps = (int)(fabs(delta) / STEP_ANGLE_DEG + 0.5f); // 四捨五入

  // 封包格式："dir,steps"
  char msg[32];
  snprintf(msg, sizeof(msg), "%d,%d", direction, steps);

  udp.beginPacket(RECEIVER_IP, UDP_PORT);
  udp.write((const uint8_t*)msg, strlen(msg));
  udp.endPacket();

  Serial.print("TX -> "); Serial.println(msg);
  previousYaw = currentYaw;
}
