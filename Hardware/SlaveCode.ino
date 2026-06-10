#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// ===== WiFi =====
#define WIFI_SSID     "xxxxxxxxx"
#define WIFI_PASSWORD "xxxxxxxxx"

// ===== Firebase =====
#define FIREBASE_HOST    "https://robotics-project-c1746-default-rtdb.europe-west1.firebasedatabase.app"
#define FIREBASE_API_KEY "xxxxxxxxx"

// ===== Servo Pins =====
#define SERVO1_PIN  18
#define SERVO2_PIN  19
#define GRIPPER_PIN 37

// ===== Continuous Servo Timing =====
const float DEGREES_PER_MS = 0.36;

// ===== Pot calibration => servo angle mapping =====
// Pot home = 120 => servo 0°, Pot max = 230 => servo 110°
#define POT_HOME  120
#define POT_MAX   230

Servo servo1;
Servo servo2;
Servo gripperServo;

float servo1CurrentAngle  = 0;
float servo2CurrentAngle  = 0;
bool  gripperClosed       = false;

// ===== Rotate continuous servo by relative degrees =====
void rotateServo(Servo &servo, float degrees) {
  if (abs(degrees) < 1.0) return;
  int direction = (degrees > 0) ? 180 : 0;
  unsigned long duration = (unsigned long)(abs(degrees) / DEGREES_PER_MS);
  servo.write(direction);
  delay(duration);
  servo.write(90); // stop
  delay(200);      // settle
}

// ===== Map pot angle to servo angle =====
float potToServo(int potAngle) {
  potAngle = constrain(potAngle, POT_HOME, POT_MAX);
  return (float)(potAngle - POT_HOME); // 0 to 110
}

bool fetchFromFirebase(int &j1Angle, int &j2Angle, String &gripperState) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] WiFi disconnected");
    return false;
  }

  HTTPClient http;
  http.setReuse(false); // force fresh connection every time
  String url = String(FIREBASE_HOST) + "/robot.json?auth=" + FIREBASE_API_KEY;
  http.begin(url);
  http.addHeader("Connection", "close");
  http.setTimeout(10000);

  int httpCode = http.GET();
  String payload = "";
  if (httpCode == 200) {
    payload = http.getString();
  }
  http.end();
  WiFi.setSleep(false); // prevent WiFi from sleeping between requests

  if (httpCode != 200) {
    Serial.print("[Firebase] GET failed, code: ");
    Serial.println(httpCode);
    return false;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("[Firebase] JSON parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  j1Angle      = doc["joint1_angle"].as<int>();
  j2Angle      = doc["joint2_angle"].as<int>();
  gripperState = doc["gripper_state"].as<String>();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());

  // Init servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  gripperServo.setPeriodHertz(50);

  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  gripperServo.attach(GRIPPER_PIN, 500, 2400);

  servo1.write(90);
  servo2.write(90);
  gripperServo.write(90);
  delay(2000);

  Serial.println("Slave ready. Listening to Firebase...");
}

void loop() {
  int j1Angle, j2Angle;
  String gripperState;

  if (WiFi.status() != WL_CONNECTED) {
  Serial.println("WiFi lost, reconnecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();
}

  if (!fetchFromFirebase(j1Angle, j2Angle, gripperState)) {
    delay(500);
    return;
  }

  Serial.print("Firebase -> J1: "); Serial.print(j1Angle);
  Serial.print(" | J2: ");          Serial.print(j2Angle);
  Serial.print(" | Gripper: ");     Serial.println(gripperState);

  // ===== Joint 1 =====
  float target1 = potToServo(j1Angle);
  float delta1  = target1 - servo1CurrentAngle;
  if (abs(delta1) >= 1.0) {
    Serial.print("Moving Servo1 by: "); Serial.println(delta1);
    rotateServo(servo1, delta1);
    servo1CurrentAngle = target1;
  }

  // ===== Joint 2 =====
  float target2 = potToServo(j2Angle);
  float delta2  = target2 - servo2CurrentAngle;
  if (abs(delta2) >= 1.0) {
    Serial.print("Moving Servo2 by: "); Serial.println(delta2);
    rotateServo(servo2, delta2);
    servo2CurrentAngle = target2;
  }

  // ===== Gripper =====
  bool shouldClose = (gripperState == "CLOSED");
  if (shouldClose != gripperClosed) {
    if (shouldClose) {
      Serial.println("Gripper -> CLOSING");
      gripperServo.write(180);
      delay(600);
    } else {
      Serial.println("Gripper -> OPENING");
      gripperServo.write(0);
      delay(600);
    }
    gripperServo.write(90); // stop
    delay(200);
    gripperClosed = shouldClose;
  }

  delay(300); // Poll Firebase every 300ms
}
