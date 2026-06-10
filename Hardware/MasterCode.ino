#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ===== WiFi =====
#define WIFI_SSID     "ColdArea"
#define WIFI_PASSWORD "12345678"

// ===== Firebase =====
#define FIREBASE_HOST "https://robotics-project-c1746-default-rtdb.europe-west1.firebasedatabase.app"
#define FIREBASE_API_KEY "AIzaSyAPLUSMP3kX0kfZcUIH9z_60LyO8f2c1X0"

// ===== Pins =====
#define HALL_EFFECT_PIN  6
#define JOINT1_POT_PIN   4
#define JOINT2_POT_PIN   5

#define HALL_CLOSED_THRESHOLD 2100

void sendToFirebase(int j1Raw, int j1Angle, int j2Raw, int j2Angle, int hallRaw, String gripperState) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] WiFi not connected, skipping...");
    return;
  }

  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/robot.json?auth=" + FIREBASE_API_KEY;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<256> doc;
  doc["joint1_raw"]      = j1Raw;
  doc["joint1_angle"]    = j1Angle;
  doc["joint2_raw"]      = j2Raw;
  doc["joint2_angle"]    = j2Angle;
  doc["hall_raw"]        = hallRaw;
  doc["gripper_state"]   = gripperState;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.PATCH(payload);

  if (httpCode == 200) {
    Serial.println("[Firebase] Sent OK");
  } else {
    Serial.print("[Firebase] Error: ");
    Serial.println(httpCode);
  }

  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
}

void loop() {
  int hallRaw   = analogRead(HALL_EFFECT_PIN);
  int joint1Raw = analogRead(JOINT1_POT_PIN);
  int joint2Raw = analogRead(JOINT2_POT_PIN);

  int joint1Angle = (int)map((long)joint1Raw, 0L, 4095L, 0L, 270L);
  int joint2Angle = (int)map((long)joint2Raw, 0L, 4095L, 0L, 270L);

  bool gripperClosed = (hallRaw < HALL_CLOSED_THRESHOLD);
  String gripperState = gripperClosed ? "CLOSED" : "OPEN";

  // Print to serial
  Serial.println("--------------------------------");
  Serial.print("Joint 1  | Raw: "); Serial.print(joint1Raw);
  Serial.print(" | Angle: "); Serial.print(joint1Angle); Serial.println(" deg");

  Serial.print("Joint 2  | Raw: "); Serial.print(joint2Raw);
  Serial.print(" | Angle: "); Serial.print(joint2Angle); Serial.println(" deg");

  Serial.print("Gripper  | Raw: "); Serial.print(hallRaw);
  Serial.print(" | State: "); Serial.println(gripperState);

  // Send to Firebase
  sendToFirebase(joint1Raw, joint1Angle, joint2Raw, joint2Angle, hallRaw, gripperState);

  delay(500); // Send every 500ms
}