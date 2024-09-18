#include <Wire.h>
#include <MPU6050.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

WebServer server(80);

//Network configuration
const char* default_ssid;
const char* default_password;

//initialize sensor, web service and non-volatile storage
MPU6050 mpu;
HTTPClient http;

//Millis implementation - instead using delay()
unsigned long lastReconnectAttempt = 0;
const long reconnectInterval = 1000;

//Sensor variables (tunning)
const float freeFallThreshold = 0.5; // Threshold for free-fall detection
const float impactThreshold = 2.7; // Threshold for impact detection
const unsigned long fallTime = 250; // Time in milliseconds to confirm a fall
unsigned long fallDetectedTime = 0;
const float gyroThreshold = 210.0; // Threshold for significant gyroscope change

//EEEPROM
const int EEPROM_SIZE = 512;
const int SSID_ADDR = 0;           
const int PASSWORD_ADDR = 32;     

//Bool for triggers
bool potentialFall = false;
bool fallRedLED = false;
bool hotSpotPurple = false;
bool isConnected = false;

//LED + button pins
const int RED_PIN = 25;    
const int BLUE_PIN = 26;
const int GREEN_PIN = 27;
const int BUTTON_PIN = 33;

//Functions
bool checkForFall(float accelTotal, float gyroX, float gyroY, float gyroZ);
void setColor(int redValue, int greenValue, int blueValue);
void sendFallNotification();
void sendFallCollectionData(float accelTotal, float gyroX, float gyroY, float gyroZ);
void enterDeepSleep();
void sendResetFallNotification();
bool isButtonPressedFor(unsigned long duration);
void startHotspotMode();
void writeToEEPROM(int startAddr, const String &data);
String readFromEEPROM(int startAddr);
void sleepWakeUp();
void connectToWiFi();


void setup() {

  //begin serial communication
  Serial.begin(4800);

  //brgin communicatiin
  Wire.begin();

  //initialize sensor
  mpu.initialize();

  //Initialize EEEPROM
  EEPROM.begin(EEPROM_SIZE);

  //Pins for LEDS
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  //Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  //sensor availability check
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connection successful");

  // Handle deep sleep wakeup
  sleepWakeUp();

  //Connect to WiFi if not woken up from deep sleep
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
    connectToWiFi();
  }
}

void loop() {
  static unsigned long pressStartTime = 0;
  static bool buttonPressed = false;

  // Check button state
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      pressStartTime = millis();
    }
  } else {
    if (buttonPressed) {
      unsigned long holdTime = millis() - pressStartTime;
      if (holdTime < 5000) {
        if (fallRedLED) {
          fallRedLED = false;
          setColor(0, 255, 0);
          sendResetFallNotification();
        }
      } else if (holdTime >= 5000 && holdTime < 10000) {
        enterDeepSleep();
      } else if (holdTime >= 10000) {
        startHotspotMode();
      }
      buttonPressed = false;
    }
  }

  server.handleClient();

  //Keep LED red after a fall is detected
  if (fallRedLED) {
    return;
  }
  //Keep purple LED on when entering AP mode
  if (hotSpotPurple) {
    return;
  }

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReconnectAttempt >= reconnectInterval) {
      lastReconnectAttempt = currentMillis;
      WiFi.disconnect(true);
      String ssid = readFromEEPROM(SSID_ADDR);
      String password = readFromEEPROM(PASSWORD_ADDR);
      WiFi.begin(ssid.c_str(), password.c_str());
    }
    setColor(0, 0, 255);
  } else {
    if (!isConnected) {
      isConnected = true;
      setColor(0, 255, 0);
    }
  }

  //Reset isConnected flag if disconnected
  if (WiFi.status() != WL_CONNECTED && isConnected) {
    isConnected = false;
  }
  if (WiFi.status() == WL_CONNECTED) {
    setColor(0, 255, 0);
    if (WiFi.getMode() & WIFI_MODE_AP) {
      WiFi.softAPdisconnect(true);
    }

    // Accelerometer and gyroscope code
    int16_t ax, ay, az; // Accelerometer
    int16_t gx, gy, gz; // Gyroscope

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw values to physical units
    float aRes = 2.0 / 32768.0;
    float accelX = aRes * ax;
    float accelY = aRes * ay;
    float accelZ = aRes * az;
    float accelTotal = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    float gRes = 250.0 / 32768.0;
    float gyroX = gRes * gx;
    float gyroY = gRes * gy;
    float gyroZ = gRes * gz;

    // Fall detection and notification logic
    if (checkForFall(accelTotal, gyroX, gyroY, gyroZ)) {
      setColor(255, 0, 0);
      sendFallNotification();
      sendFallCollectionData(accelTotal, gyroX, gyroY, gyroZ);
      fallRedLED = true;
    } else {
      setColor(0, 255, 0); // Set LED to green
    }
  }
}


//Function to detect free fall or a strong impact
bool checkForFall(float accelTotal, float gyroX, float gyroY, float gyroZ) {
  bool freeFall = accelTotal < freeFallThreshold;
  bool impact = fabs(accelTotal) > impactThreshold;
  bool angularChange = fabs(gyroX) > gyroThreshold || fabs(gyroY) > gyroThreshold || fabs(gyroZ) > gyroThreshold;
  if ((freeFall && angularChange) || (impact && angularChange)) {
    if (!potentialFall) {
      potentialFall = true;
      fallDetectedTime = millis();
    } else if (millis() - fallDetectedTime > fallTime) {
      potentialFall = false; 
      return true;
    }
  } else {
    potentialFall = false;
  }

  return false;
}

void setColor(int redValue, int greenValue, int blueValue) {
    //assigning values to colours (0-255, 0,0,0)
    analogWrite(RED_PIN, redValue);
    analogWrite(GREEN_PIN, greenValue);
    analogWrite(BLUE_PIN, blueValue);
}

void sendFallNotification() {
  http.begin("http://maker.ifttt.com/trigger/Fall_detect/with/key/"KEY"");
  int httpCode = http.GET();

  //chech if the message was sent succesfuly (blue flash - succes and 4 times red unsuccessfuk)
  if (httpCode > 0) {
    setColor(0,0,255);
    delay(500);
    setColor(255, 0, 0);
  } else {
    setColor(255,0,0);
    delay(500);
    setColor(255,0,0);
    delay(500);
    setColor(255,0,0);
    delay(500);
    setColor(255,0,0);
    delay(500);
  }
  http.end();
}

void sendResetFallNotification() {
  http.begin("url to IFTTT");
  int httpCode = http.GET(); //Make a request

  //chech if the message was sent succesfuly (blue flash - succes and 4 times red unsuccessfuk)
  if (httpCode > 0) {
    setColor(0,0,255);
    delay(500);
    setColor(255, 0, 0);
  } else {
    setColor(255,0,0);
    delay(500);
    setColor(255,0,0);
    delay(500);
    setColor(255,0,0);
    delay(500);
    setColor(255,0,0);
    delay(500);
  }
  http.end();
}

void sendFallCollectionData(float accelTotal, float gyroX, float gyroY, float gyroZ) {
  String url = "URL to IFTTT";
  url += "?value1=" + String(accelTotal);
  url += "&value2=" + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ);
  http.begin(url);
  int httpCode = http.GET(); 
  http.end();
}

bool isButtonPressedFor(unsigned long duration) {
  unsigned long pressStartTime = millis();
  while (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - pressStartTime > duration) {
      return true;
    }
  }
  return false;
}

void enterDeepSleep() {
    delay(1000);
    setColor(0, 0, 0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW); 
    esp_deep_sleep_start();
}

void startHotspotMode() {
  hotSpotPurple = true;
  setColor(255, 0, 255);

  WiFi.softAP("Fall detection sensor", "12345678", 6);

  //A simple web server
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><title>Wi-Fi Settings</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; background-color: #f0f0f0; text-align: center; margin-top: 50px; }";
    html += "h2 { color: #0d6efd; }";
    html += "form { background-color: #ffffff; padding: 20px; border-radius: 10px; display: inline-block; }";
    html += "input[type='text'], input[type='password'] { width: 200px; padding: 10px; margin: 10px 0; border-radius: 5px; border: 1px solid #ddd; }";
    html += "input[type='submit'] { width: 220px; padding: 10px; border-radius: 5px; background-color: #0d6efd; color: white; border: none; cursor: pointer; }";
    html += "input[type='submit']:hover { background-color: #0b5ed7; }";
    html += "</style></head><body>";
    html += "<h2>Set Wi-Fi Credentials</h2>";
    html += "<form action='/set' method='POST'>";
    html += "SSID: <br><input type='text' name='ssid'><br>";
    html += "Password: <br><input type='password' name='password'><br>";
    html += "<input type='submit' value='Save'>";
    html += "</form></body></html>";

    server.send(200, "text/html", html);
  });
  server.on("/set", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    String password = server.arg("password");

    writeToEEPROM(SSID_ADDR, ssid);
    writeToEEPROM(PASSWORD_ADDR, password);

    server.send(200, "text/plain", "Saved. Restarting...");
    delay(1000);
    ESP.restart();
  });
  server.begin();
}

void writeToEEPROM(int startAddr, const String &data) {
  int i;
  for (i = 0; i < data.length(); ++i) {
    EEPROM.write(startAddr + i, data[i]);
  }
  EEPROM.write(startAddr + i, '\0');
  EEPROM.commit();
}

String readFromEEPROM(int startAddr) {
    String data = "";
    char ch;
    for (int i = 0; i < EEPROM_SIZE; ++i) {
        ch = EEPROM.read(startAddr + i);
        if (ch == '\0') {
            break;
        }
        data += ch;
    }
    return data;
}

void connectToWiFi(){
  String ssid = readFromEEPROM(SSID_ADDR);
  String password = readFromEEPROM(PASSWORD_ADDR);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
  }
}

void sleepWakeUp() {
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    mpu.setSleepEnabled(false);
    unsigned long pressStartTime = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
      if (millis() - pressStartTime > 5000) { 
        if (WiFi.status() != WL_CONNECTED) {
          setColor(0, 0, 255);
          connectToWiFi();
        } else {
          setColor(0, 255, 0);
        }
        break;
      }
    }
    if (millis() - pressStartTime < 5000) {
      enterDeepSleep();
    }
  }
}
