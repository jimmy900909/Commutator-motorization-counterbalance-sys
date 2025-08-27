// Nano 33 IoT Receiver: UDP "dir,steps" -> drive A4988
#include <WiFiNINA.h>
#include <WiFiUdp.h>

const char* SSID = "concord.hippo_5ghz";
const char* PASS = "n!/(n-k)!";

WiFiUDP udp;
const uint16_t UDP_PORT = 9000;

const int DIR_PIN  = 2;
const int STEP_PIN = 3;
const int EN_PIN   = 4;      // 若未接EN，程式可忽略此腳
const int STEP_US  = 1200;   // 初始保守速度；數值越小越快

void stepN(int n){
  for(int i=0;i<n;i++){
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(STEP_US);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(STEP_US);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // 使能 A4988（LOW=Enable）

  Serial.print("WiFi connecting");
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\nWiFi connected!");
  Serial.print("Nano 33 IoT IP: "); Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.print("UDP listening on "); Serial.println(UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  char buf[32];
  int len = udp.read(buf, sizeof(buf)-1);
  if (len <= 0) return;
  buf[len] = '\0';

  int dir=0, steps=0;
  if (sscanf(buf, "%d,%d", &dir, &steps) == 2) {
    Serial.print("RX <- dir="); Serial.print(dir);
    Serial.print(", steps="); Serial.println(steps);

    digitalWrite(DIR_PIN, (dir > 0) ? HIGH : LOW);
    stepN(steps);
  } else {
    Serial.print("Bad packet: "); Serial.println(buf);
  }
}
