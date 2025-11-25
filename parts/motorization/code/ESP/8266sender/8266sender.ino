// ESP8266 Sender: QMC5883L -> deltaYaw -> (dir,steps) over UDP (1/16 microstep)
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

const char* SSID = "Jimmys";
const char* PASS = "0937333355";

IPAddress RECEIVER_IP(172,20,10,4); // 換成 Nano 33 IoT 的 IP
const uint16_t UDP_PORT = 9000;

WiFiUDP udp;
QMC5883LCompass compass;

float previousYaw = 0.0f;

// ------- 微步設定 -------
const int   MICROSTEP_DIV     = 16;            // 1/16 微步
const float STEP_ANGLE_DEG    = 1.8f / MICROSTEP_DIV; // 每個「微步」角度
const float MAX_DELTA         = 90.0f;         // 單次最大修正角度
const float THRESHOLD         = 3.0f;          // 小於3度不傳(防抖)
const unsigned SEND_INTERVAL_MS = 100;         // 10 Hz

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
  Wire.begin(4, 5); // SDA=D2, SCL=D1
  delay(200);
  compass.init();

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

  if (fabs(delta) < THRESHOLD) return;
  delta = constrain(delta, -MAX_DELTA, MAX_DELTA);

  int direction = (delta > 0) ? 1 : -1;
  // 1/16 微步：步數 = 角度 / (1.8/16)
  int steps = (int)lround(fabs(delta) / STEP_ANGLE_DEG);

  if (steps > 0) {
    char msg[32];
    snprintf(msg, sizeof(msg), "%d,%d", direction, steps);
    udp.beginPacket(RECEIVER_IP, UDP_PORT);
    udp.write((const uint8_t*)msg, strlen(msg));
    udp.endPacket();
    Serial.print("TX -> "); Serial.println(msg);
    previousYaw = currentYaw;
  }
}

