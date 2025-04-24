#include <Arduino.h> // fro the board
#include <LiquidCrystal_I2C.h> // for LCD
#include <Keypad.h> //keypad
#include <DHT.h> // for temp and humidity
#include <RTClib.h> // for rtc module.
#include <Wire.h> // this is for I2C communication pins
// #include <WiFi.h> // COMMENTED OUT - WiFi not used
// #include <HTTPClient.h> // COMMENTED OUT - WiFi HTTP client not used
#include <ArduinoJson.h> // for JSON parsing
#include <TimeLib.h> // Include TimeLib.h for time conversion
#include <HardwareSerial.h> // for sim800l

// Forward declarations
int getDistance();
float waterFlowSensor();
void humidityTemp(float &temperature, float &humidity);
bool verifyNationalId(String nationalId);  // declare forward so we can define after
void sendDataToServer(float temperature, float humidity, int soilMoisture,
                      String valveStatus, float waterFlow, String currentTime, int distance);
void sendWeatherDataToServer();  // forward declaration for sim-based
String readResponse(int timeout);
bool waitForResponse(String expected, int timeout);
bool sendHttpRequest(String url, String method, String payload = "");
String extractJsonFromResponse(String response);
bool sendSMS(String phoneNumber, String message);

// >>>> COMMENTED OUT: Wi-Fi credentials <<<<
// const char* ssid = "Nexacore";
// const char* password ="Nexacore.@123";

//Define LCD Settings
LiquidCrystal_I2C lcd(0x27,16,2);

//define object for RTC
RTC_DS3231 rtc; 

//sim800l (Use pins that you have physically wired up)
HardwareSerial sim800(1); // use Serial1 for sim800L

//Define keypad.
const byte ROWS = 4; // has 4 rows
const byte COLS =  3; // has 3 columns
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// Define row and column pin connections
byte rowPins[ROWS] = {2,4,5,18};    // row pins
byte colPins[COLS] = {15,32,33};    // column pins

// initialize the keypad Library
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// define constraints
#define TRIG_PIN 13 // connected to send signal on the sonar sensor
#define ECHO_PIN 12 // act as an input
#define BUZZER_PIN 25 // this is the out of the pin to send signal
#define FLOW_SENSOR_PIN 23 // this is the pins for the waterflow signal
#define RELAY_PIN 26 // this is for the 5v relay pin.
const int soilMoisturePin = 34;  // Define the analog pin
#define SIM800L_RESET_PIN 5 // conect as a reset pin
#define SIM800_RX_PIN 16
#define SIM800_TX_PIN 17

// Timing for the screen timeout
unsigned long lastKeyPressTime = 0;
const unsigned long backlightTimeout = 7000;  

// timing of the sonar sensor
unsigned long lastDistanceTime = 0; 
const unsigned long SonarInterval = 500;

// count pulse from flow sensor
volatile int pulseCount = 0; 
unsigned long lastFlowReadTime = 0;
const unsigned long flowInterval = 1000;
float calibrationFactor = 450.0;  // Pulses per liter for the flow sensor

//time for sensor data.
unsigned long lastSendTime = 0;  
const unsigned long sendInterval = 600000;  // 10 min

//esp32 on time
unsigned long lastFetchTime = 0; 
const unsigned long fetchInterval = 30000; // 30 seconds

//time for weather
unsigned long lastpullTime = 0;  
const unsigned long sendWeatherInterval = 600000;  // 10min

//time for prevoius keypad get
unsigned long previousKeypadTime = 0;
const unsigned long keypadInterval = 50; // Poll every 50ms

// time for measuring soilmoisture value
unsigned long lastSoilCheckTime = 0;
const unsigned long soilCheckInterval = 10 * 60 * 1000; // checks after 10min

// Declare global variables
int soilMoistureValue = 0;
int minReading = 0;
int maxReading = 500;
float maxSoilDepthMM = 50;
float temperature = 0, humidity = 0;
int distanceGlobal = 0;
float waterFlowGlobal = 0;
unsigned long lastSensorUpdateTime = 0;
const unsigned long sensorUpdateInterval = 10 * 60 * 1000; // 10 minutes

void updateSensorData() {
    soilMoistureValue = analogRead(soilMoisturePin);
    humidityTemp(temperature, humidity);
    distanceGlobal = getDistance();
    waterFlowGlobal = waterFlowSensor();
}

//check if user is logged in
bool userLoggedIn = false;

void IRAM_ATTR countPulse(){
  pulseCount ++;
}

// Define DHT Sensor Settings
#define DHTPIN 14 
#define DHTTYPE DHT22 
DHT dht(DHTPIN,DHTTYPE); 

// For smoothing out the distance
const int numReadings = 10;
long readings[numReadings];
int readIndex = 0;
long total = 0;
long averageDistance = 0;

// Variables for weather data
String sunshineDuration = "";
String weatherTimestamp = "";
float weatherTemperature = 0.0;
float weatherHumidity = 0.0;
float rainfall = 0.0;
float windSpeed = 0.0;
int cloudCover = 0;
String forecast = "";
String weatherId = "";

//data from user table
String cropId = "";
String cropsPlanted = "";
String datePlanted = "";
String userName = "";

// turn on/off variable
bool isControllerOn = true;

//irrigation variables
enum IrrigationState {
    IRRIGATION_IDLE,
    IRRIGATION_RUNNING,
    IRRIGATION_WAITING
};

IrrigationState irrigationState = IRRIGATION_IDLE;
unsigned long irrigationStartTime = 0;
unsigned long irrigationDurationMillis = 10000; // 10 seconds
int soilMoistureThreshold = 1072; // Adjust as needed

//---------------------------------------------------------------------
// SIM800L  startup functions
//---------------------------------------------------------------------
// Enhanced Serial Debug Function
void printHexDebug(const String& message) {
  Serial.println(message);
  for (unsigned int i = 0; i < message.length(); i++) {
    Serial.print("0x");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

bool setupSIM800L() {
  Serial.println("Detailed SIM800L Initialization...");
  
  // Reset Sequence
  pinMode(SIM800L_RESET_PIN, OUTPUT);
  digitalWrite(SIM800L_RESET_PIN, LOW);
  delay(200);
  digitalWrite(SIM800L_RESET_PIN, HIGH);
  delay(3000);  // Longer delay for module to boot

  // Flush serial buffer
  while(sim800.available()) {
    sim800.read();
  }

  // Multiple initialization attempts with different strategies
  const char* initCommands[] = {
    "AT",           // Basic test
    "ATE0",         // Disable echo
    "AT+IPR=9600",  // Set fixed baud rate
    "AT+CMEE=2",    // Verbose error codes
    "AT+CPIN?"      // Check SIM status
  };

  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.print("Initialization Attempt ");
    Serial.println(attempt + 1);

    // Try each command
    for (const char* cmd : initCommands) {
      Serial.print("Sending command: ");
      Serial.println(cmd);

      sim800.println(cmd);
      delay(1000);

      String response = "";
      unsigned long startTime = millis();
      
      while (millis() - startTime < 2000) {
        if (sim800.available()) {
          char c = sim800.read();
          response += c;
          Serial.print(c);
        }
      }

      Serial.println("\nFull Response:");
      printHexDebug(response);

      // Look for positive responses
      if (response.indexOf("OK") != -1 || 
          response.indexOf("+CPIN: READY") != -1) {
        Serial.println("Positive response received!");
        return true;
      }
    }

    delay(1000);  // Pause between attempts
  }

  Serial.println("SIM800L Initialization FAILED");
  return false;
}

//---------------------------------------------------------------------
// Wi-Fi CODE COMMENTED-OUT / REPLACED WITH SIM800L
//---------------------------------------------------------------------

/*
// connect to wifi
void connectWifi(){
   lcd.setCursor(0,0);
   lcd.print("Connecting to ");
   lcd.setCursor(0,1);
   lcd.print("wifi ....");
   WiFi.begin(ssid, password);
   delay(3000);

   while (WiFi.status() != WL_CONNECTED) {
     delay(1000);
     Serial.print("trying..");
   }

   if (WiFi.status() != WL_CONNECTED) {
     Serial.println("\nFailed to connect to Wi-Fi!");
     lcd.setCursor(0,0);
     lcd.print("connection zii");
     delay(3000);
   } else {
     Serial.println("\nConnected to Wi-Fi!");
     Serial.print("IP Address: ");
     Serial.println(WiFi.localIP());
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print("wifi connected!");
     lcd.setCursor(0,1);
     lcd.print(WiFi.localIP());
     delay(4000);
   }
   lcd.clear();
   delay(3000);
}
*/

//---------------------------------------------------------------------
// FETCH CONTROLLER STATE (previously used Wi-Fi, now using SIM800L)
//---------------------------------------------------------------------
void fetchControllerState() {
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Fetching state...");

    // COMMENTED OUT Wi-Fi CODE:
    /*
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("https://midtermdocs-production.up.railway.app/api/controller/state");
        int httpResponseCode = http.GET();
         ...
    } else {
        Serial.println("WiFi not connected");
    }
    */

    // --- SIM800L version ---
    Serial.println("Using SIM800L to fetch controller state...");

    // 1. Initialize HTTP
    sim800.println("AT+HTTPINIT");
    delay(1000);

    // 2. Set GPRS CID
    sim800.println("AT+HTTPPARA=\"CID\",1");
    delay(1000);

    // 3. Set the URL
    String url = "https://midtermdocs-production.up.railway.app/api/controller/state";
    sim800.print("AT+HTTPPARA=\"URL\",\"");
    sim800.print(url);
    sim800.println("\"");
    delay(2000);

    // 4. Start the GET action
    sim800.println("AT+HTTPACTION=0"); // 0 = GET, 1=POST
    delay(8000);

    // 5. Read the response
    sim800.println("AT+HTTPREAD");
    delay(2000);

    // Read what we can from SIM800 buffer
    String response = "";
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
    }
    Serial.print("Server Response: ");
    Serial.println(response);

    // End HTTP
    sim800.println("AT+HTTPTERM");
    delay(1000);

    // Try to parse the JSON from 'response'
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    if (!error) {
        isControllerOn = doc["isControllerOn"];
        Serial.print("Controller State: ");
        Serial.println(isControllerOn ? "ON" : "OFF");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Controller:");
        lcd.setCursor(0, 1);
        lcd.print(isControllerOn ? "ON" : "OFF");

        if (!isControllerOn) {
            // go to deep sleep in real scenario
            Serial.println("Controller set to OFF, going deep sleep...");
            lcd.clear();
            lcd.print("Shutting down...");
            delay(2000);
            // ESP sleep code, if relevant
            // ...
        }
    } else {
        Serial.println("JSON parsing failed or missing data");
        lcd.clear();
        lcd.print("JSON parse fail!");
    }

    delay(3000);
}


//---------------------------------------------------------------------
// BOARD STATUS (no HTTP here, just prints to LCD/Serial)
//---------------------------------------------------------------------
void boardStatus() {
    // Flash memory size
    lcd.backlight();
    uint32_t flashSize = ESP.getFlashChipSize();
    Serial.print("Flash chip size: ");
    Serial.print(flashSize / 1024 / 1024); 
    Serial.println(" MB");
    lcd.setCursor(0, 0);
    lcd.print("Flash Size:");
    lcd.print(flashSize / 1024 / 1024);
    lcd.print("MB");

    uint32_t flashFrequency = ESP.getFlashChipSpeed();
    Serial.print("Flash chip freq: ");
    Serial.print(flashFrequency / 1000000);
    Serial.println(" MHz");
    lcd.setCursor(0, 1);
    lcd.print("Freq: ");
    lcd.print(flashFrequency / 1000000);
    lcd.print(" MHz");

    delay(3000);

    // Flash usage
    size_t sketchSize = ESP.getSketchSize();
    size_t freeFlash = flashSize - sketchSize;
    float flashUsagePercentage = ((float)sketchSize / flashSize) * 100;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Flash Used:");
    lcd.print((int)flashUsagePercentage);
    lcd.print("%");

    delay(3000);

    // SRAM usage
    size_t freeHeap = ESP.getFreeHeap();
    size_t totalHeap = ESP.getHeapSize();
    float heapUsagePercentage = ((float)(totalHeap - freeHeap) / totalHeap) * 100;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SRAM Used:");
    lcd.print((int)heapUsagePercentage);
    lcd.print("%");

    delay(3000);
    lcd.clear();
}

//---------------------------------------------------------------------
// LOGIN
//---------------------------------------------------------------------
void loginSound(String pattern) {
  if (pattern == "success") {
    // Two high-pitched beeps
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, 1000);
      delay(200);
      noTone(BUZZER_PIN);
      delay(200);
    }
  } 
  else if (pattern == "failure") {
    // Three low-pitched beeps
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 500);
      delay(300);
      noTone(BUZZER_PIN);
      delay(150);
    }
  } 
  else if (pattern == "alternate") {
    // High and low-pitched beeps alternating
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, 1000);
      delay(200);
      noTone(BUZZER_PIN);
      delay(100);
      tone(BUZZER_PIN, 500);
      delay(300);
      noTone(BUZZER_PIN);
      delay(200);
    }
  } 
  else {
    Serial.println("Invalid login sound pattern.");
  }
}

void userLogin() {
    lcd.clear();
    unsigned long lastSoundTime = 0;
    const unsigned long soundInterval = 2000;
    String enteredId = "";

    while (true) {
        if (millis() - lastSoundTime > soundInterval) {
            loginSound("alternate");
            lastSoundTime = millis();
        }

        lcd.setCursor(0, 0);
        lcd.print("Enter Ntl ID:");

        char key = keypad.getKey();
        if (key) {
            if (key == '#') { // Submit
                if (enteredId.length() == 0) {
                    lcd.setCursor(0, 1);
                    lcd.print("ID cannot be empty");
                    delay(2000);
                    lcd.clear();
                    continue;
                }
                lcd.setCursor(0, 1);
                lcd.print("Checking...");
                delay(500);

                if (verifyNationalId(enteredId)) {
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Login Success!");
                    loginSound("success");
                    delay(2000);

                    userLoggedIn = true;

                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Crops: ");
                    lcd.print(cropsPlanted);
                    lcd.setCursor(0, 1);
                    lcd.print("Planted: ");
                    lcd.print(datePlanted);

                    delay(5000);
                    lcd.clear();
                    return;
                } else {
                    lcd.setCursor(0, 1);
                    lcd.print("Invalid ID!");
                    loginSound("failure");
                    delay(2000);
                    lcd.clear();
                    enteredId = "";
                }
            } 
            else if (key == '*') {
                enteredId = "";
                lcd.setCursor(0, 1);
                lcd.print("                ");
            } 
            else {
                enteredId += key;
                lcd.setCursor(0, 1);
                lcd.print(enteredId);
            }
        }
    }
}

//---------------------------------------------------------------------
// VERIFY NATIONAL ID via SIM800L (replaces Wi-Fi code)
//---------------------------------------------------------------------
bool verifyNationalId(String nationalId) {
  Serial.println("Verifying National ID using SIM800L...");
  
  // Construct the URL and payload
  String url = "https://midtermdocs-production.up.railway.app/esp/auth";
  String payload = "{\"nationalid\":\"" + nationalId + "\"}";
  
  // Send HTTP POST request
  if (sendHttpRequest(url, "POST", payload)) {
      // Read the response
      sim800.println("AT+HTTPREAD");
      delay(2000);
      String response = readResponse(3000);
      
      // End the HTTP session
      sim800.println("AT+HTTPTERM");
      delay(1000);
      
      // Extract JSON from response
      String jsonData = extractJsonFromResponse(response);
      Serial.println("Extracted JSON: " + jsonData);
      
      // Parse the JSON
      JsonDocument doc ;
      DeserializationError error = deserializeJson(doc, jsonData);
      
      if (!error && jsonData.indexOf("\"status\":\"success\"") != -1) {
          // Extract user data
          cropsPlanted = doc["user"]["cropsplanted"].as<String>();
          datePlanted = doc["user"]["dateplanted"].as<String>();
          userName = doc["user"]["name"].as<String>();
          
          // Crop ID
          cropId = "";
          if (cropsPlanted == "Maize") {
              cropId = "C001";
          }
          
          Serial.println("Welcome " + userName);
          Serial.println("Crops Planted: " + cropsPlanted);
          Serial.println("Date Planted: " + datePlanted);
          
          // Send SMS notification
          sendSMS("+254114881057", "User " + userName + " logged in to Farm_yangu!");
          
          return true;
      } else {
          Serial.println("Authentication failed or JSON parsing error");
      }
  } else {
      Serial.println("HTTP request failed");
  }
  
  return false;
}


//---------------------------------------------------------------------
// sendMessage()
//---------------------------------------------------------------------

bool sendSMS(String phoneNumber, String message) {
  Serial.println("Sending SMS to: " + phoneNumber);
  
  // Set SMS text mode
  sim800.println("AT+CMGF=1");
  if (!waitForResponse("OK", 2000)) {
    Serial.println("Failed to set SMS text mode");
    return false;
  }
  
  // Set SMS character set
  sim800.println("AT+CSCS=\"GSM\"");
  if (!waitForResponse("OK", 2000)) {
    Serial.println("Failed to set SMS character set");
    return false;
  }
  
  // Set recipient phone number
  sim800.print("AT+CMGS=\"");
  sim800.print(phoneNumber);
  sim800.println("\"");
  
  if (!waitForResponse(">", 5000)) {
    Serial.println("Failed to get SMS prompt");
    return false;
  }
  
  // Send message content
  sim800.print(message);
  sim800.write(26);  // Ctrl+Z to end message
  
  // Wait for response (may take time to send)
  if (waitForResponse("+CMGS:", 10000)) {
    Serial.println("SMS sent successfully!");
    
    // Display on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SMS Sent!");
    lcd.setCursor(0, 1);
    lcd.print("Login Notified");
    delay(2000);
    
    return true;
  } else {
    Serial.println("Failed to send SMS");
    return false;
  }
  return false;
}

//---------------------------------------------------------------------
// PLAY SIREN
//---------------------------------------------------------------------
void playSiren(){
  tone(BUZZER_PIN,1000);
  delay(300);
  tone(BUZZER_PIN,500);
  delay(300);
}

//---------------------------------------------------------------------
// GET DISTANCE (Ultrasonic) + smoothing
//---------------------------------------------------------------------
int getDistance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 60000);
  long distance = duration * 0.034 / 2;
  if (distance < 2 || distance > 300) {
    // invalid
    distance = 0;
  }

  // Smoothing
  if (distance > 0 && distance < 300) {
    total -= readings[readIndex];
    readings[readIndex] = distance;
    total += readings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    averageDistance = total / numReadings;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm   Avg: ");
    Serial.println(averageDistance);
  }

  // Alarm if too close
  if (averageDistance > 0 && averageDistance < 4) {
    digitalWrite(LED_BUILTIN, HIGH);
    playSiren();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,3);
    lcd.print("Too close!");
    delay(3000);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    noTone(BUZZER_PIN);
  }

  return averageDistance;
}

//---------------------------------------------------------------------
// DHT CHECK
//---------------------------------------------------------------------
void checkDHT() {
    float tempTest = dht.readTemperature();
    float humTest  = dht.readHumidity();

    if (isnan(tempTest) || isnan(humTest)) {
        Serial.println("DHT sensor initialization failed. Please check wiring.");
    } else {
        Serial.println("DHT sensor initialized successfully.");
    }
    delay(2000);
}

//---------------------------------------------------------------------
// HUMIDITY & TEMPERATURE
//---------------------------------------------------------------------
void humidityTemp(float &temperature, float &humidity){
  for (int i = 0; i < 3; i++) { 
    temperature = dht.readTemperature();
    humidity    = dht.readHumidity();
    if (!isnan(temperature) && !isnan(humidity)) {
        break;
    }
    delay(500);
  }

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temp: ");
    Serial.print(temperature);
    Serial.println("*C");

    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity);
    lcd.print("%");
  }
  delay(5000);
}

//---------------------------------------------------------------------
// RTC CHECK
//---------------------------------------------------------------------
void timeModuleCheck(){
  if(!rtc.begin()){
    Serial.println("Couldn't find RTC");
    lcd.setCursor(0,0);
    lcd.clear();
    lcd.print("RTC not found");
    while(1);
  }
  // If the RTC lost power, we could set it
  if(rtc.lostPower()){
    // rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  }

  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("RTC Module");
  lcd.setCursor(0,1);
  lcd.print("Starting..");
  delay(3000);
  lcd.clear();
}

//---------------------------------------------------------------------
// DISPLAY TIME
//---------------------------------------------------------------------
String displayTime() {
    DateTime now = rtc.now();
    int hour = now.hour();
    String meridian = "AM";
    if (hour >= 12) {
        meridian = "PM";
        if (hour > 12) hour -= 12;
    } else if (hour == 0) {
        hour = 12;
    }
    int month = now.month();
    int year  = now.year();
    int day   = now.day();

    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    if (hour < 10) lcd.print('0');
    lcd.print(hour); lcd.print(':');
    if (now.minute() < 10) lcd.print('0');
    lcd.print(now.minute()); lcd.print(':');
    if (now.second() < 10) lcd.print('0');
    lcd.print(now.second()); lcd.print(" ");
    lcd.print(meridian);

    lcd.setCursor(0, 1);
    lcd.print("Date: ");
    if (month < 10) lcd.print('0');
    lcd.print(month); lcd.print('/');
    if (day < 10) lcd.print('0');
    lcd.print(day); lcd.print('/');
    lcd.print(year);

    Serial.print("Time: ");
    if (hour < 10) Serial.print('0');
    Serial.print(hour); Serial.print(':');
    if (now.minute() < 10) Serial.print('0');
    Serial.print(now.minute()); Serial.print(':');
    if (now.second() < 10) Serial.print('0');
    Serial.print(now.second());
    Serial.print(" ");
    Serial.print(meridian);
    Serial.print(" | Date: ");
    if (month < 10) Serial.print('0');
    Serial.print(month); Serial.print('/');
    if (day < 10) Serial.print('0');
    Serial.print(day); Serial.print('/');
    Serial.println(year);

    char buffer[25];
    sprintf(buffer, "%02d:%02d:%02d %s | %02d/%02d/%04d",
            hour, now.minute(), now.second(), meridian.c_str(), month, day, year);
    return String(buffer);
}

//---------------------------------------------------------------------
// TIMESTAMP
//---------------------------------------------------------------------
String getISO8601TimestampUTC() {
    DateTime utcTime = rtc.now();
    char buffer[20];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
            utcTime.year(), utcTime.month(), utcTime.day(),
            utcTime.hour(), utcTime.minute(), utcTime.second());
    return String(buffer);
}

//---------------------------------------------------------------------
// FLOW SENSOR
//---------------------------------------------------------------------
float waterFlowSensor() {
  noInterrupts();
  int pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  if (pulses < 5) { 
      Serial.println("Noise detected on flow sensor!");
      return 0.0;
  }
  float waterFlow = (pulses / calibrationFactor);
  Serial.print("Water Flow: ");
  Serial.print(waterFlow);
  Serial.println(" L/min");
  return waterFlow;
}

//---------------------------------------------------------------------
// BASIC SOIL MOISTURE
//---------------------------------------------------------------------
int soilMoisture(){
  pinMode(soilMoisturePin, INPUT);
  int rawValue = analogRead(soilMoisturePin);
  if (rawValue < 50) {
      Serial.println("Warning: Floating sensor input detected!");
      return -1;
  }
  Serial.print("Soil Raw Moisture: ");
  Serial.println(rawValue);
  delay(1000);
  return rawValue;
}

//---------------------------------------------------------------------
// START SENSORS
//---------------------------------------------------------------------
void startSensors(){
  lcd.setCursor(0,0);
  lcd.print("Welcome ,");
  lcd.print(userName);
  lcd.setCursor(4,1);
  lcd.print("Starting....");
  delay(5000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ultrasonic ");
  lcd.setCursor(0,1);
  lcd.print("Starting....");
  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("DHT  ");
  lcd.setCursor(0,1);
  lcd.print("Starting....");
  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Soil Moisture ");
  lcd.setCursor(0,1);
  lcd.print("Starting....");
  delay(3000);
}

//---------------------------------------------------------------------
// SEND SENSOR DATA -> using SIM800L
//---------------------------------------------------------------------
void sendDataToServer(float temperature, float humidity, int soilMoisture,
                      String valveStatus, float waterFlow, String currentTime, int distance)
{
    Serial.println("Sending data using SIM800L...");
    // 1. Initialize
    sim800.println("AT+HTTPINIT");
    delay(1000);

    // 2. Set GPRS CID
    sim800.println("AT+HTTPPARA=\"CID\",1");
    delay(1000);

    // 3. Set the URL
    String url = "https://midtermdocs-production.up.railway.app/add/sensor_data";
    sim800.print("AT+HTTPPARA=\"URL\",\"");
    sim800.print(url);
    sim800.println("\"");
    delay(2000);

    // 4. Set content type
    sim800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    delay(1000);

    // 5. Start sending data
    sim800.println("AT+HTTPDATA=300,10000");
    delay(2000);

    // Prepare JSON
    DateTime now = rtc.now();
    String timestamp = String(now.year()) + "-" + String(now.month()) + "-" +
                       String(now.day()) + "T" + String(now.hour()) + ":" +
                       String(now.minute()) + ":" + String(now.second()) + "Z";

    String dataID = "D-" + String(now.unixtime());

    String jsonPayload = "{";
    jsonPayload += "\"dataid\":\"" + dataID + "\",";
    jsonPayload += "\"timestamp\":\"" + timestamp + "\",";
    jsonPayload += "\"soilmoisture\":" + String(soilMoisture) + ",";
    jsonPayload += "\"temperature\":" + String(temperature) + ",";
    jsonPayload += "\"humidity\":" + String(humidity) + ",";
    jsonPayload += "\"valvestatus\":\"" + valveStatus + "\",";
    jsonPayload += "\"waterflow\":" + String(waterFlow) + ",";
    jsonPayload += "\"currentTime\":\"" + currentTime + "\",";
    jsonPayload += "\"distance\":" + String(distance);
    jsonPayload += "}";

    Serial.print("Sending JSON Payload: ");
    Serial.println(jsonPayload);

    // 6. Send the JSON
    sim800.println(jsonPayload);
    delay(1000);

    // 7. Post
    sim800.println("AT+HTTPACTION=1"); 
    delay(10000);

    // 8. Read response
    sim800.println("AT+HTTPREAD");
    delay(3000);
    String response = "";
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
    }
    Serial.print("Server response: ");
    Serial.println(response);

    // 9. Close HTTP
    sim800.println("AT+HTTPTERM");
    delay(1000);
}

//---------------------------------------------------------------------
// READ & DISPLAY SENSOR DATA, THEN SEND
//---------------------------------------------------------------------
void readSensorData(float temperature, float humidity, int soilMoistureValue, 
                    String valveStatus, float waterFlow, String currentTime,int distance)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reading Data...");
    delay(1500);

    // Print each to Serial & show on LCD briefly
    Serial.print("Temperature: ");
    Serial.println(temperature);
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("C");
    delay(3000);

    Serial.print("Humidity: ");
    Serial.println(humidity);
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity);
    lcd.print("%");
    delay(3000);

    Serial.print("Soil Moisture: ");
    Serial.println(soilMoistureValue);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SoilMoist: ");
    lcd.print(soilMoistureValue);
    delay(3000);

    Serial.print("Valve Status: ");
    Serial.println(valveStatus);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Valve: ");
    lcd.print(valveStatus);
    delay(3000);

    Serial.print("Water Flow: ");
    Serial.println(waterFlow);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Flow: ");
    lcd.print(waterFlow);
    lcd.print(" L/min");
    delay(3000);

    Serial.print("Tank distance: ");
    Serial.println(distance);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T Distance: ");
    lcd.print(distance);
    lcd.print("cm");
    delay(3000);

    // Now send to server via SIM800L
    sendDataToServer(temperature, humidity, soilMoistureValue,
                     valveStatus, waterFlow, currentTime, distance);

    delay(5000);
}

//---------------------------------------------------------------------
// CROP AGE
//---------------------------------------------------------------------
int calculateCropAge(DateTime plantingDate, DateTime currentDate) {
    TimeSpan age = currentDate - plantingDate;
    return age.days();
}

//---------------------------------------------------------------------
// FETCH WEATHER DATA (SIM800L instead of WiFi)
//---------------------------------------------------------------------
void fetchWeatherData() {
  Serial.println("Fetching weather data via SIM800L...");
  
  // Weather API details
  String apiKey = "81f360187f497f7d31d712d1c08bd0e1";
  String lat = "-0.4500318";
  String lon = "36.9163043";
  String units = "metric";
  String apiUrl = "https://api.openweathermap.org/data/2.5/weather?lat=" + lat +
                  "&lon=" + lon + "&units=" + units + "&appid=" + apiKey;
  
  // Send HTTP GET request
  if (sendHttpRequest(apiUrl, "GET")) {
      // Read the response
      sim800.println("AT+HTTPREAD");
      delay(2000);
      String response = readResponse(3000);
      
      // End the HTTP session
      sim800.println("AT+HTTPTERM");
      delay(1000);
      
      // Extract and parse JSON
      String jsonData = extractJsonFromResponse(response);
      
      const size_t capacity = 1024;
      DynamicJsonDocument doc(capacity);
      DeserializationError error = deserializeJson(doc, jsonData);
      
      if (!error) {
          // Extract weather data
          weatherTimestamp = getISO8601TimestampUTC();
          weatherTemperature = doc["main"]["temp"].as<float>();
          weatherHumidity = doc["main"]["humidity"].as<float>();
          windSpeed = doc["wind"]["speed"].as<float>();
          cloudCover = doc["clouds"]["all"].as<int>();
          forecast = doc["weather"][0]["description"].as<String>();
          weatherId = String(doc["weather"][0]["id"].as<int>());
          rainfall = doc["rain"]["1h"] | 0.0;
          sunshineDuration = "N/A"; // Not provided by OpenWeatherMap
          
          // Print data for debugging
          Serial.println("Weather data retrieved successfully:");
          Serial.println("Temperature: " + String(weatherTemperature) + "°C");
          Serial.println("Humidity: " + String(weatherHumidity) + "%");
          Serial.println("Forecast: " + forecast);
      } else {
          Serial.println("JSON parsing failed: " + String(error.c_str()));
      }
  } else {
      Serial.println("Failed to fetch weather data");
  }
}

//---------------------------------------------------------------------
// SEND WEATHER DATA TO SERVER (SIM800L)
//---------------------------------------------------------------------
void sendWeatherDataToServer() {
  Serial.println("Sending weather data via SIM800L...");
  // 1. init
  sim800.println("AT+HTTPINIT");
  delay(1000);
  // 2. set param
  sim800.println("AT+HTTPPARA=\"CID\",1");
  delay(1000);

  // 3. set URL
  String url = "https://midtermdocs-production.up.railway.app/add/weather_data";
  sim800.print("AT+HTTPPARA=\"URL\",\"");
  sim800.print(url);
  sim800.println("\"");
  delay(2000);

  // 4. set content
  sim800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(1000);

  // 5. data
  sim800.println("AT+HTTPDATA=300,10000");
  delay(2000);

  String jsonPayload = "{";
  jsonPayload += "\"sunshineduration\":\"" + sunshineDuration + "\",";
  jsonPayload += "\"timestamp\":\"" + weatherTimestamp + "\",";
  jsonPayload += "\"temperature\":" + String(weatherTemperature) + ",";
  jsonPayload += "\"humidity\":" + String(weatherHumidity) + ",";
  jsonPayload += "\"rainfall\":" + String(rainfall) + ",";
  jsonPayload += "\"windspeed\":" + String(windSpeed) + ",";
  jsonPayload += "\"cloudcover\":" + String(cloudCover) + ",";
  jsonPayload += "\"forecast\":\"" + forecast + "\",";
  jsonPayload += "\"weatherid\":\"" + weatherId + "\"";
  jsonPayload += "}";

  Serial.print("Sending JSON Payload: ");
  Serial.println(jsonPayload);

  sim800.println(jsonPayload);
  delay(2000);

  // 6. Post
  sim800.println("AT+HTTPACTION=1");
  delay(8000);

  // 7. read
  sim800.println("AT+HTTPREAD");
  delay(3000);

  String response = "";
  while (sim800.available()) {
    char c = sim800.read();
    response += c;
  }
  Serial.println("Server Response:");
  Serial.println(response);

  sim800.println("AT+HTTPTERM");
  delay(1000);

  // minimal LCD feedback
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wthr Resp...");
  delay(2000);
  lcd.clear();
}

//---------------------------------------------------------------------
// IRRIGATION ALGORITHM
//---------------------------------------------------------------------
float calculateETo(float T_mean, float T_max, float T_min) {
  float ETo = 0.0023 * (T_mean + 17.8) * sqrt(T_max - T_min);
  return ETo; 
}

void fetchLatestData(float &T_max, float &T_min, int &cloudCoverPercentage) {
  Serial.println("Fetching latest weather data via SIM800L...");
  
  // 1. Initialize HTTP
  sim800.println("AT+HTTPINIT");
  delay(1000);
  
  // 2. Set GPRS CID
  sim800.println("AT+HTTPPARA=\"CID\",1");
  delay(1000);
  
  // 3. Set the URL
  String url = "https://midtermdocs-production.up.railway.app/api/latest_data";
  sim800.print("AT+HTTPPARA=\"URL\",\"");
  sim800.print(url);
  sim800.println("\"");
  delay(2000);
  
  // 4. Perform GET request
  sim800.println("AT+HTTPACTION=0"); // 0 = GET
  delay(5000);
  
  // 5. Read response
  sim800.println("AT+HTTPREAD");
  delay(2000);
  
  String response = "";
  while (sim800.available()) {
    char c = sim800.read();
    response += c;
  }
  
  // 6. Close HTTP
  sim800.println("AT+HTTPTERM");
  delay(1000);
  
  Serial.println("Response: " + response);
  
  // Extract JSON from response
  int jsonStart = response.indexOf("{");
  int jsonEnd = response.lastIndexOf("}") + 1;
  
  if (jsonStart > 0 && jsonEnd > jsonStart) {
    String jsonData = response.substring(jsonStart, jsonEnd);
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);
    
    if (!error) {
      T_max = doc["max_temperature"].as<float>();
      T_min = doc["min_temperature"].as<float>();
      cloudCoverPercentage = doc["cloudCover"].as<int>();
      
      Serial.println("T_max: " + String(T_max));
      Serial.println("T_min: " + String(T_min));
      Serial.println("Cloud Cover: " + String(cloudCoverPercentage));
    } else {
      // Default values if parsing fails
      T_max = 28.0;
      T_min = 15.0;
      cloudCoverPercentage = 50;
    }
  } else {
    // Default values if JSON not found
    T_max = 28.0;
    T_min = 15.0;
    cloudCoverPercentage = 50;
  }
}

float calculateETc(float ETo, float Kc) {
    return (ETo * Kc);
}

String getCropId(String cropName) {
    if (cropName == "Maize") {
        return "C001";
    } else {
        return "";
    }
}

int calculateCropAgeWeeks(String datePlanted) {
    int year, month, day;
    sscanf(datePlanted.c_str(), "%d-%d-%d", &year, &month, &day);
    DateTime plantingDate(year, month, day);
    DateTime currentDate = rtc.now();
    TimeSpan age = currentDate - plantingDate;
    return age.days() / 7;
}

float fetchKcValue(String cropId, int cropAgeWeeks) {
  Serial.println("Fetching Kc value via SIM800L...");
  float kcValue = 0.65; // Default value
  
  // 1. Initialize HTTP
  sim800.println("AT+HTTPINIT");
  delay(1000);
  
  // 2. Set GPRS CID
  sim800.println("AT+HTTPPARA=\"CID\",1");
  delay(1000);
  
  // 3. Set the URL
  String url = "https://midtermdocs-production.up.railway.app/api/get_kc?cropid=" + cropId + "&cropageweeks=" + String(cropAgeWeeks);
  sim800.print("AT+HTTPPARA=\"URL\",\"");
  sim800.print(url);
  sim800.println("\"");
  delay(2000);
  
  // 4. Perform GET request
  sim800.println("AT+HTTPACTION=0"); // 0 = GET
  delay(5000);
  
  // 5. Read response
  sim800.println("AT+HTTPREAD");
  delay(2000);
  
  String response = "";
  while (sim800.available()) {
    char c = sim800.read();
    response += c;
  }
  
  // 6. Close HTTP
  sim800.println("AT+HTTPTERM");
  delay(1000);
  
  // Extract JSON from response
  int jsonStart = response.indexOf("{");
  int jsonEnd = response.lastIndexOf("}") + 1;
  
  if (jsonStart > 0 && jsonEnd > jsonStart) {
    String jsonData = response.substring(jsonStart, jsonEnd);
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, jsonData);
    
    if (!error) {
      kcValue = doc["kc"].as<float>();
      Serial.println("Kc value: " + String(kcValue));
    }
  }
  
  return kcValue;
}

float calculateSoilMoistureInMM(int soilMoisturePin, int minReading, int maxReading,
                                float maxSoilDepthMM, int numSamples = 1)
{
    int totalMoisture = 0;
    int validReadings = 0;

    for (int i = 0; i < numSamples; i++) {
        int currentReading = analogRead(soilMoisturePin);
        if (currentReading > 0) {
            totalMoisture += currentReading;
            validReadings++;
        }
        delay(100);
    }
    if (validReadings == 0) {
        Serial.println("No valid readings available!");
        return 0;
    }
    int averageMoistureValue = totalMoisture / validReadings;
    Serial.print("Average Soil Moisture: ");
    Serial.println(averageMoistureValue);

    float moisturePercentage = map(averageMoistureValue, minReading, maxReading, 0, 100);
    moisturePercentage = constrain(moisturePercentage, 0, 100);
    float moistureInMM = (moisturePercentage / 100.0) * maxSoilDepthMM;

    Serial.print("Calculated Soil Moisture (mm): ");
    Serial.println(moistureInMM);
    return moistureInMM;
}

//---------------------------------------------------------------------
// HANDLE KEYPAD
//---------------------------------------------------------------------
void handleKeyPress(char key) {
    switch (key) {
        case '1': // Display time
            lcd.backlight();
            lcd.clear();
            displayTime();
            delay(3500);
            break;

        case '2': // temp & humidity
            lcd.backlight();
            humidityTemp(temperature, humidity);
            lcd.noBacklight();
            lcd.clear();
            break;

        case '3': // valve status
        {
            String valveStatus = digitalRead(RELAY_PIN) ? "Open" : "Closed";
            lcd.clear();
            lcd.backlight();
            lcd.setCursor(0, 0);
            lcd.print("Valve:");
            lcd.setCursor(0, 1);
            lcd.print(valveStatus);
            delay(5000);
            lcd.noBacklight();
            lcd.clear();
        }
        break;

        case '4': // tank distance
        {
            int distanceVal = getDistance();
            lcd.clear();
            lcd.backlight();
            lcd.setCursor(0, 0);
            lcd.print("Distance :");
            lcd.setCursor(0, 1);
            lcd.print(distanceVal);
            lcd.print("cm");
            delay(3500);
            lcd.noBacklight();
            lcd.clear();
        }
        break;

        case '5': // flow rate
        {
            float wf = waterFlowSensor();
            lcd.clear();
            lcd.backlight();
            lcd.setCursor(0, 0);
            lcd.print("Flow  :");
            lcd.setCursor(0, 1);
            lcd.print(wf);
            lcd.print("L/min");
            delay(3500);
            lcd.noBacklight();
            lcd.clear();
        }
        break;

        case '6': // test solenoid
            lcd.backlight();
            lcd.clear();
            lcd.print("Solenoid Test");
            digitalWrite(RELAY_PIN, HIGH);
            delay(10000);
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("Solenoid Deactivated");
            lcd.clear();
            break;

        case '7': // soil moisture
        {
            lcd.backlight();
            lcd.clear();
            int rawSoil = soilMoisture();
            lcd.setCursor(0,0);
            lcd.print("Moisture Raw:");
            lcd.setCursor(0, 1);
            lcd.print(rawSoil);
            delay(3000);
            lcd.clear();
        }
        break;

        case '#': // Logout
            userLoggedIn = false;
            lcd.backlight();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Logged Out");
            delay(3000);
            lcd.clear();
            break;

        default:
            Serial.println("Invalid key pressed");
            break;
    }
}

//---------------------------------------------------------------------
// Helper functions
//---------------------------------------------------------------------


// Helper to read response with timeout
String readResponse(int timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (sim800.available()) {
      char c = sim800.read();
      response += c;
    }
  }
  
  return response;
}

// Helper to wait for specific response
bool waitForResponse(String expected, int timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (sim800.available()) {
      char c = sim800.read();
      response += c;
      
      if (response.indexOf(expected) >= 0) {
        return true;
      }
    }
  }
  
  return false;
}

// Helper for HTTP requests
bool sendHttpRequest(String url, String method, String payload ) {
  bool success = false;
  
  // Initialize HTTP
  sim800.println("AT+HTTPINIT");
  delay(1000);
  
  // Check for OK response
  if (!waitForResponse("OK", 2000)) {
    Serial.println("Error initializing HTTP");
    return false;
  }
  
  // Set GPRS CID
  sim800.println("AT+HTTPPARA=\"CID\",1");
  delay(1000);
  
  // Set URL
  sim800.print("AT+HTTPPARA=\"URL\",\"");
  sim800.print(url);
  sim800.println("\"");
  delay(2000);
  
  // If POST method, set content type and send data
  if (method == "POST" && payload.length() > 0) {
    sim800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    delay(1000);
    
    // Send data
    sim800.print("AT+HTTPDATA=");
    sim800.print(payload.length());
    sim800.println(",10000"); // 10 sec timeout
    delay(2000);
    
    // Wait for DOWNLOAD prompt
    if (waitForResponse("DOWNLOAD", 5000)) {
      sim800.println(payload);
      delay(1000);
    } else {
      Serial.println("Failed to get DOWNLOAD prompt");
      sim800.println("AT+HTTPTERM");
      return false;
    }
  }
  
  // Send action (0=GET, 1=POST)
  if (method == "GET") {
    sim800.println("AT+HTTPACTION=0");
  } else {
    sim800.println("AT+HTTPACTION=1");
  }
  
  // Wait for response
  if (waitForResponse("+HTTPACTION:", 10000)) {
    // Check status code
    String actionResponse = readResponse(1000);
    if (actionResponse.indexOf(",200,") > 0) {
      success = true;
    } else {
      Serial.println("HTTP error: " + actionResponse);
    }
  }
  
  return success;
}

// Extract JSON from SIM800L response
String extractJsonFromResponse(String response) {
  // Look for JSON start/end markers
  int jsonStart = response.indexOf("{");
  int jsonEnd = response.lastIndexOf("}") + 1;
  
  if (jsonStart >= 0 && jsonEnd > jsonStart) {
    return response.substring(jsonStart, jsonEnd);
  }
  
  return "{}"; // Empty JSON if none found
}

//---------------------------------------------------------------------
// SETUP
//---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

   // Then initialize I2C with specific pins
   Wire.begin(21, 22); // Use your actual SDA, SCL pins
   delay(500);
   
   // Then initialize LCD
   lcd.init();
   lcd.backlight();
   delay(500);

  // Relay
  pinMode(RELAY_PIN,OUTPUT); 
  digitalWrite(RELAY_PIN,LOW);

  Serial.println("SIM800L Configuration:");
  Serial.print("RX Pin: "); Serial.println(SIM800_RX_PIN);
  Serial.print("TX Pin: "); Serial.println(SIM800_TX_PIN);
  Serial.print("Reset Pin: "); Serial.println(SIM800L_RESET_PIN); 

  // SIM800
  sim800.begin(9600, SERIAL_8N1, SIM800_RX_PIN,SIM800_TX_PIN); // match wiring

  Serial.print("SIM800L Serial RX: ");
  Serial.println(16);
  Serial.print("SIM800L Serial TX: ");
  Serial.println(17);

  if (!setupSIM800L()) {
    Serial.println("Critical: SIM800L Failed to Initialize");
  } else {
    Serial.println("SIM800L Successfully Initialized!");
  }

  Serial.println("flow Sensor & relay Setup Complete");

  // Flow sensor
  pinMode(FLOW_SENSOR_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countPulse, RISING);

  // >>> COMMENTED OUT Wi-Fi
  // connectWifi();

  // Prompt login if needed
  if (!userLoggedIn) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Please Login");
      delay(4000);
      userLogin();
  }

  startSensors();
  boardStatus();
  timeModuleCheck();

  // Additional pins
  pinMode(soilMoisturePin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // DHT
  dht.begin();
  checkDHT();

  // init distance array
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

//---------------------------------------------------------------------
// LOOP
//---------------------------------------------------------------------
void loop() {
  // Check controller state every fetchInterval
  if (millis() - lastFetchTime >= fetchInterval) {
    Serial.println("Refreshing controller state...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Refreshing...");
    fetchControllerState();
    lastFetchTime = millis();
    lcd.clear();
  }

  // Keypad
  if (millis() - previousKeypadTime >= keypadInterval) {
    previousKeypadTime = millis();
    char key = keypad.getKey();
    if (key) {
      handleKeyPress(key);
    }
  }

  // If user logs out
  if (!userLoggedIn) {
    userLogin();  
    return;       
  }

  // Turn off LCD if no activity
  if (millis() - lastKeyPressTime > backlightTimeout) {
    lcd.noBacklight();
  }

  // measure distance periodically
  if(millis() - lastDistanceTime >= SonarInterval){
    getDistance();
    lastDistanceTime = millis();
  }

  // Pull & Send Weather data
  if(millis() - lastpullTime >= sendWeatherInterval){
    fetchWeatherData();
    sendWeatherDataToServer();
    lastpullTime = millis();
  }

  // Send sensor data
  if (millis() - lastSendTime >= sendInterval) {
    float temperature, humidity;
    humidityTemp(temperature, humidity);
    String currentTime = displayTime();
    int soilMoistVal = analogRead(soilMoisturePin);
    String valveStatus = digitalRead(RELAY_PIN) ? "Open" : "Closed";
    int distanceVal = getDistance();
    float wFlow = waterFlowSensor();

    readSensorData(temperature, humidity, soilMoistVal,
                   valveStatus, wFlow, currentTime, distanceVal);

    lastSendTime = millis();
  }

  // Periodic soil check for irrigation
  soilMoistureValue = analogRead(soilMoisturePin);
  if (millis() - lastSoilCheckTime >= soilCheckInterval) {
    lastSoilCheckTime = millis();
    if (soilMoistureValue < soilMoistureThreshold) {
      Serial.println("Soil moisture below threshold, fetching data...");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Soil moisture");
      lcd.setCursor(0,1);
      lcd.print("Low..");
      delay(3000);

      irrigationState = IRRIGATION_RUNNING;

      // Here we do the calculations
      int cropAgeWeeks = calculateCropAgeWeeks(datePlanted);
      float Kc = fetchKcValue(cropId, cropAgeWeeks);

      float T_max, T_min;
      int cloudCoverPct;
      fetchLatestData(T_max, T_min, cloudCoverPct);

      float T_mean = (T_max + T_min)/2.0;
      float ETo = calculateETo(T_mean, T_max, T_min);
      Serial.print("Calculated ETo: ");
      Serial.println(ETo);

      cropAgeWeeks = calculateCropAgeWeeks(datePlanted);
      Kc = fetchKcValue(cropId, cropAgeWeeks);
      Serial.print("Kc Value: ");
      Serial.println(Kc);

      float ETc = calculateETc(ETo, Kc);
      Serial.print("Calculated ETc: ");
      Serial.println(ETc);

    } else {
      Serial.println("Soil moisture OK. No irrigation needed.");
    }
  }

  // IRRIGATION STATE
  switch(irrigationState) {
    case IRRIGATION_IDLE:
      if (soilMoistureValue < soilMoistureThreshold) {
        digitalWrite(RELAY_PIN, HIGH); 
        irrigationStartTime = millis();
        irrigationState = IRRIGATION_RUNNING;
        Serial.println("Irrigation started");
        lcd.clear();
        lcd.backlight();
        lcd.print("Irrigation ON");
      }
      break;

    case IRRIGATION_RUNNING:
      if (millis() - irrigationStartTime >= irrigationDurationMillis) {
        digitalWrite(RELAY_PIN, LOW);
        irrigationState = IRRIGATION_WAITING;
        Serial.println("Irrigation stopped");
        lcd.clear();
        lcd.print("Irrigation OFF");
        irrigationStartTime = millis();
      }
      break;

    case IRRIGATION_WAITING:
      if (millis() - irrigationStartTime >= 5000) {
        // check again
        soilMoistureValue = analogRead(soilMoisturePin);
        if (soilMoistureValue < soilMoistureThreshold) {
          digitalWrite(RELAY_PIN, HIGH);
          irrigationStartTime = millis();
          irrigationState = IRRIGATION_RUNNING;
          Serial.println("Irrigation restarted");
          lcd.clear();
          lcd.print("Irrigation ON");
        } else {
          irrigationState = IRRIGATION_IDLE;
          Serial.println("Soil moisture sufficient");
          lcd.clear();
          lcd.print("Soil Moist OK");
          delay(2000);
          lcd.noBacklight();
        }
      }
      break;
  }

  delay(50);
}
