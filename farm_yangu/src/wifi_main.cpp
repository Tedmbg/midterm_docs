#include <Arduino.h> // fro the board
#include <LiquidCrystal_I2C.h> // for LCD
#include <Keypad.h> //keypad
#include <DHT.h> // for temp and humidity
#include <RTClib.h> // for rtc module.
#include <Wire.h> // this is for I2C communication pins
#include <WiFi.h> // this is the wifi module
#include <HTTPClient.h> // for api's
#include <ArduinoJson.h> // for JSON parsing
#include <TimeLib.h> // Include TimeLib.h for time conversion



// Forward declarations of functions used before defined
int getDistance();
float waterFlowSensor();
void humidityTemp(float &temperature, float &humidity);

const char* ssid = "Nexacore";
const char* password ="Nexacore.@123";

// wifi connection details
// const char* ssid = "iot";
// const char* password ="123456789";

// const char* ssid = "Searching";
// const char* password ="Ndogogio1";

// const char* ssid = "iPhone (5)";
// const char* password ="kipkuruz";


//Define LCD Settings
LiquidCrystal_I2C lcd(0x27,16,2);

//define object for RTC
RTC_DS3231 rtc; 


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
byte rowPins[ROWS] = {2,4,5,18};    // this are the pins that tell the esp which key row has been pressed.
byte colPins[COLS] = {15,32,33};  // Connect to the column pins of the keypad

// initialize the keypad Library
Keypad keypad = Keypad(makeKeymap(keys),rowPins,colPins,ROWS,COLS);


// define constraints
#define TRIG_PIN 13 // connected to send signal on the sonar sensor
#define ECHO_PIN 12 // act as an input
#define BUZZER_PIN 25 // this is the out of the pin to send signal
#define FLOW_SENSOR_PIN 23 // this is the pins for the waterflow signal
#define RELAY_PIN 26 // this is for the 5v relay pin.



const int soilMoisturePin = 34;  // Define the analog pin

// Timing for the screen timeout
unsigned long lastKeyPressTime = 0;  // Stores the time of the last key press
const unsigned long backlightTimeout = 7000;  // 7 seconds timeout

// timing of the sonic sensor
unsigned long lastDistanceTime = 0; // last time distance was measured
const unsigned long SonarInterval = 500; // measure after 3 seconds

// count pulse from flow sensor
volatile int pulseCount = 0 ; 
unsigned long lastFlowReadTime = 0;
const unsigned long flowInterval = 1000;  // Measure flow rate every 1 second
float calibrationFactor = 450.0;  // Pulses per liter for the flow sensor

//time for sensor data.
unsigned long lastSendTime = 0;  // Tracks the last time data was sent
const unsigned long sendInterval = 600000;  // 10 min interval

//esp32 on time
unsigned long lastFetchTime = 0; // Store the last time the function was called
const unsigned long fetchInterval = 30000; // 30 seconds in milliseconds

//time for weather
unsigned long lastpullTime = 0;  // Tracks the last time data was pulled and sent
const unsigned long sendWeatherInterval = 600000;  // 10min interval

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
int distance = 0;
float waterFlow = 0;
unsigned long lastSensorUpdateTime = 0;
const unsigned long sensorUpdateInterval = 10 * 60 * 1000; // 10 minutes

void updateSensorData() {
    soilMoistureValue = analogRead(soilMoisturePin);
    humidityTemp(temperature, humidity); // Update temperature and humidity
    distance = getDistance();           // Update water tank distance
    waterFlow = waterFlowSensor();      // Update water flow
}


//check if user is logged in
bool userLoggedIn = false;

void IRAM_ATTR countPulse(){
  pulseCount ++;
}


// Define DHT Sensor Settings
#define DHTPIN 14 // pin connected to the DHT sensor
#define DHTTYPE DHT22 
DHT dht(DHTPIN,DHTTYPE); // create a DHT sensor object

// Smoothing out the distance
// Variables for moving average filter
const int numReadings = 10;  // Number of readings for averaging
long readings[numReadings];  // Array to hold distance readings
int readIndex = 0;           // Current index in the array
long total = 0;              // Sum of all readings
long averageDistance = 0;    // Calculated moving average


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


//declared functions
bool verifyNationalId(String nationalId);
void loginSound(String type);
String getCropId(String cropName);

// turn on off variable
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

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>fuctions<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Function to turn off the ESP32 (Deep Sleep)
void fetchControllerState() {
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0);

    if (WiFi.status() == WL_CONNECTED) {
        lcd.print("Fetching state...");
        HTTPClient http;
        http.begin("https://midtermdocs-production.up.railway.app/api/controller/state"); // Replace with your API endpoint
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("Server Response: " + response);

            // Parse JSON response
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, response);

            if (!error) {
                isControllerOn = doc["isControllerOn"];
                Serial.print("Controller State: ");
                Serial.println(isControllerOn ? "ON" : "OFF");

                // Update LCD with state
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Controller:");
                lcd.setCursor(0, 1);
                lcd.print(isControllerOn ? "ON" : "OFF");

                // Enter deep sleep only if controller is OFF
                if (!isControllerOn) {
                    Serial.println("Entering Deep Sleep...");
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Shutting down...");
                    delay(2000);

                    // Enter deep sleep mode
                    esp_sleep_enable_timer_wakeup(10 * 1000000); // Wake after 10 seconds
                    esp_deep_sleep_start();
                }
            } else {
                Serial.println("JSON parsing failed");
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("JSON parse fail!");
            }
        } else {
            Serial.print("HTTP GET failed, code: ");
            Serial.println(httpResponseCode);

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("HTTP GET Error:");
            lcd.setCursor(0, 1);
            lcd.print(httpResponseCode);
        }

        http.end();
    } else {
        Serial.println("WiFi not connected");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi not");
        lcd.setCursor(0, 1);
        lcd.print("connected");
    }

    delay(3000);
}

// get memory
void boardStatus() {
    // Flash memory size
    lcd.backlight();
    uint32_t flashSize = ESP.getFlashChipSize();
    Serial.print("Flash chip size: ");
    Serial.print(flashSize / 1024 / 1024); // Convert bytes to MB
    Serial.println(" MB");
    lcd.setCursor(0, 0);
    lcd.print("Flash Size:");
    lcd.print(flashSize / 1024 / 1024);
    lcd.print("MB");

    // Flash frequency
    uint32_t flashFrequency = ESP.getFlashChipSpeed();
    Serial.print("Flash chip frequency: ");
    Serial.print(flashFrequency / 1000000); // Convert Hz to MHz
    Serial.println(" MHz");
    lcd.setCursor(0, 1);
    lcd.print("Freq: ");
    lcd.print(flashFrequency / 1000000);
    lcd.print(" MHz");

    delay(3000); // Allow time for LCD to display the data

    // Flash memory usage
    size_t sketchSize = ESP.getSketchSize();
    size_t freeFlash = flashSize - sketchSize;
    float flashUsagePercentage = ((float)sketchSize / flashSize) * 100;
    
    Serial.print("Sketch size: ");
    Serial.print(sketchSize / 1024); // Convert bytes to KB
    Serial.println(" KB");
    Serial.print("Free flash memory: ");
    Serial.print(freeFlash / 1024); // Convert bytes to KB
    Serial.println(" KB");
    Serial.print("Flash usage: ");
    Serial.print(flashUsagePercentage);
    Serial.println(" %");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Flash Used:");
    lcd.print((int)flashUsagePercentage);
    lcd.print("%");

    delay(3000); // Allow time for LCD to display the data

    // SRAM usage
    size_t freeHeap = ESP.getFreeHeap();
    size_t totalHeap = ESP.getHeapSize();
    float heapUsagePercentage = ((float)(totalHeap - freeHeap) / totalHeap) * 100;

    Serial.print("Total SRAM: ");
    Serial.print(totalHeap / 1024); // Convert bytes to KB
    Serial.println(" KB");
    Serial.print("Free SRAM: ");
    Serial.print(freeHeap / 1024); // Convert bytes to KB
    Serial.println(" KB");
    Serial.print("SRAM usage: ");
    Serial.print(heapUsagePercentage);
    Serial.println(" %");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SRAM Used:");
    lcd.print((int)heapUsagePercentage);
    lcd.print("%");

    delay(3000); // Allow time for LCD to display the data
    lcd.clear();


}

// login function
void userLogin() {
    lcd.clear();
    unsigned long lastSoundTime = 0;
    const unsigned long soundInterval = 2000; // Interval for the beeping sound
    String enteredId = "";

    while (true) {
        // Play the beep sound at intervals
        if (millis() - lastSoundTime > soundInterval) {
            loginSound("alternate");
            lastSoundTime = millis();
        }

        // Prompt the user to enter their National ID
        lcd.setCursor(0, 0);
        lcd.print("Enter Ntl ID:");

        // Check for key input
        char key = keypad.getKey();
        if (key) {
            if (key == '#') { // Submit
                if (enteredId.length() == 0) {
                    // If no input, prompt user again
                    lcd.setCursor(0, 1);
                    lcd.print("ID cannot be empty");
                    delay(2000);
                    lcd.clear();
                    continue;
                }

                // Display "Checking" and validate
                lcd.setCursor(0, 1);
                lcd.print("Checking...");
                delay(500);

                if (verifyNationalId(enteredId)) {
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Login Success!");
                    loginSound("success"); // Play success sound
                    delay(2000);

                    userLoggedIn = true; // Mark user as logged in

                    // Display crop and planting date
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Crops: ");
                    Serial.print("Crops: ");
                    Serial.println(cropsPlanted);
                    lcd.print(cropsPlanted);
                    lcd.setCursor(0, 1);
                    Serial.print("datePlanted: ");
                    Serial.println(datePlanted);
                    lcd.print("Planted: ");
                    lcd.print(datePlanted);

                    delay(5000);
                    lcd.clear();
                    return; // Exit login loop
                } else {
                    // Invalid ID
                    lcd.setCursor(0, 1);
                    lcd.print("Invalid ID!");
                    loginSound("failure"); // Play failure sound
                    delay(2000);
                    lcd.clear();
                    enteredId = ""; // Reset input
                }
            } else if (key == '*') { // Clear input
                enteredId = "";
                lcd.setCursor(0, 1);
                lcd.print("                "); // Clear the second line
            } else { // Append key to entered ID
                enteredId += key;
                lcd.setCursor(0, 1);
                lcd.print(enteredId);
            }
        }
    }
}

// login beep sound
void loginSound(String pattern) {
  if (pattern == "success") {
    // Two high-pitched beeps
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, 1000); // High tone
      delay(200);
      noTone(BUZZER_PIN);    // Turn off tone
      delay(200);            // Pause between beeps
    }
  } 
  else if (pattern == "failure") {
    // Three low-pitched beeps
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 500); // Low tone
      delay(300);
      noTone(BUZZER_PIN);
      delay(150);            // Short pause between beeps
    }
  } 
  else if (pattern == "alternate") {
    // High and low-pitched beeps alternating
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, 1000); // High tone
      delay(200);
      noTone(BUZZER_PIN);
      delay(100);
      tone(BUZZER_PIN, 500);  // Low tone
      delay(300);
      noTone(BUZZER_PIN);
      delay(200);
    }
  } 
  else {
    // Default case if the pattern doesn't match
    Serial.println("Invalid login sound pattern.");
  }
}

//verify user data
bool verifyNationalId(String nationalId) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected");
        return false;
    }

    HTTPClient http;
    http.begin("https://midtermdocs-production.up.railway.app/esp/auth"); // Replace with your API endpoint
    http.addHeader("Content-Type", "application/json");

    // Create JSON payload
    String payload = "{\"nationalid\":\"" + nationalId + "\"}";
    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Server Response: " + response);

        // Check for success in the response
        if (response.indexOf("\"status\":\"success\"") != -1) {
            // Parse JSON response using ArduinoJson library
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, response);

            if (error) {
                Serial.print("JSON parsing failed: ");
                Serial.println(error.c_str());
                return false;
            }

            // Extract cropsPlanted and datePlanted from JSON
            cropsPlanted = doc["user"]["cropsplanted"].as<String>();
            datePlanted = doc["user"]["dateplanted"].as<String>();
            userName = doc["user"]["name"].as<String>();

            //Map crop name to cropId
            cropId = getCropId(cropsPlanted);
            if (cropId == ""){
              Serial.println("unknown crop planted");
              lcd.clear();
              lcd.print("Unkown crop");
              delay(3000);
              return false;
            }

            // Print to Serial for debugging
            Serial.println("Welcome " + userName);
            Serial.println("Crops Planted: " + cropsPlanted);
            Serial.println("Date Planted: " + datePlanted);

            http.end();
            return true;
        } else {
            Serial.println("Authentication failed");
        }
    } else {
        Serial.print("HTTP Request failed, Error code: ");
        Serial.println(httpResponseCode);
    }

    http.end();
    return false;
}

// play sound
void playSiren(){
  tone(BUZZER_PIN,1000);
  delay(300);
  tone(BUZZER_PIN,500);
  delay(300);
}

// ####################### get distance
int getDistance(){

  // Trigger the ultrasonic to send sound waves
  digitalWrite(TRIG_PIN, LOW);  // Set voltage to 0 to reset the sensor.
  delayMicroseconds(5);         // Wait for 5 microseconds to ensure a clean LOW state.

  // Turn the sensor on
  digitalWrite(TRIG_PIN, HIGH);  // Send a pulse.
  delayMicroseconds(20);         // Send a 20 microsecond pulse.

  // Reset the sensor
  digitalWrite(TRIG_PIN, LOW);

   // Get duration
  long duration = pulseIn(ECHO_PIN, HIGH, 60000);  // Wait for 60 milliseconds

  // Get distance
  long distance = duration * 0.034 / 2;

  if (distance < 2 || distance > 300) {  // Filter out values that are too high or too low
    Serial.println("Invalid distance reading");
    distance = 0;  // Set distance to 0 for invalid readings
  }

  // Check if distance is valid
  if (distance > 0 && distance < 300) {  
    // Add the new reading to the total and calculate the average
    total -= readings[readIndex];       // Subtract the oldest reading
    readings[readIndex] = distance;     // Store the new reading
    total += readings[readIndex];       // Add the new reading to the total
    readIndex = (readIndex + 1) % numReadings;  // Advance to the next index

    // Calculate the average distance
    averageDistance = total / numReadings;

    // Print the raw distance value
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm   ");

    // Print the average distance value
    Serial.print("Average Distance: ");
    Serial.print(averageDistance);
    Serial.println(" cm");

  }  

    // LED Control based on the average distance
  if (averageDistance > 0 && averageDistance < 4) {  // Check valid average distance
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED if average distance is within 10 cm
    playSiren(); // this turning on the buzzer
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,3);
    lcd.print("To close !");
    delay(3000);
  } else {
    digitalWrite(LED_BUILTIN, LOW);   // Turn off LED otherwise
    noTone(BUZZER_PIN); // the buzzer remains off
    lcd.setCursor(0, 3);  // Set cursor to the first line
    // lcd.print("                ");  // Overwrite the previous message with spaces
    // lcd.noBacklight();  // Turn off LCD backlight
  }

return averageDistance;
}

//check dht
void checkDHT() {
    float tempTest = dht.readTemperature();
    float humTest = dht.readHumidity();

    if (isnan(tempTest) || isnan(humTest)) {
        Serial.println("DHT sensor initialization failed. Please check wiring.");
    } else {
        Serial.println("DHT sensor initialized successfully.");
    }

    delay(2000);
}

// @@@@@@@@@@@@@@@@@@@@@@@@@@ humidity temp
void humidityTemp(float &temperature, float &humidity){
  // Get DHT Sensor readings
      for (int i = 0; i < 3; i++) { // Retry up to 3 times
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            break; // Exit loop if readings are valid
        }

        delay(500); // Short delay before retrying
    }


// Check if any reads failed and exit early
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    // Display DHT values on Serial Monitor
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("*C");

    // Display on LCD
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

// ⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️⏱️ check RTC module
void timeModuleCheck(){
  if(!rtc.begin()){
    Serial.println("Couldn't find RTC");
    lcd.setCursor(0,0);
    lcd.clear();
    lcd.print("RTC not found");
    while(1);
  }
    // // Manually adjust time to a specific value (for example, 2024-10-15 12:30:45)
    // rtc.adjust(DateTime(2024, 10, 22, 12, 39, 04));  // Year, Month, Day, Hour, Minute, Second

  if(rtc.lostPower()){
    // Serial.println("RTC lost power, setting the time!");
    // rtc.adjust(DateTime(F(__DATE__),F(__TIME__))); // set time when the code was compiled

  

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

// ⌚️⌚️⌚️⌚️⌚️⌚️⌚️⌚️⌚️⌚️⌚️⌚️ Display time on lcd
String displayTime() {
    // Get the current time from the RTC
    DateTime now = rtc.now();

    // Convert to 12-hour format
    int hour = now.hour();
    String meridian = "AM";
    if (hour >= 12) {
        meridian = "PM";
        if (hour > 12) hour -= 12;  // Convert to 12-hour format
    } else if (hour == 0) {
        hour = 12;  // Midnight is 12 AM
    }

    // Fetch month and year
    int month = now.month();
    int year = now.year(); // Full year (e.g., 2024)

    // Optionally, fetch the day of the month
    int day = now.day();

    // Display time on the LCD
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0); // First line
    lcd.print("Time: ");
    if (hour < 10) lcd.print('0'); // Add leading zero for single-digit hours
    lcd.print(hour);
    lcd.print(':');
    if (now.minute() < 10) lcd.print('0'); // Add leading zero for single-digit minutes
    lcd.print(now.minute());
    lcd.print(':');
    if (now.second() < 10) lcd.print('0'); // Add leading zero for single-digit seconds
    lcd.print(now.second());
    lcd.print(" ");
    lcd.print(meridian);

    // Display date on the second line of the LCD
    lcd.setCursor(0, 1); // Second line
    lcd.print("Date: ");
    if (month < 10) lcd.print('0'); // Add leading zero for single-digit months
    lcd.print(month);
    lcd.print('/');
    if (day < 10) lcd.print('0'); // Add leading zero for single-digit days
    lcd.print(day);
    lcd.print('/');
    lcd.print(year);

    // Print time and date to Serial Monitor
    Serial.print("Time: ");
    if (hour < 10) Serial.print('0');
    Serial.print(hour);
    Serial.print(':');
    if (now.minute() < 10) Serial.print('0');
    Serial.print(now.minute());
    Serial.print(':');
    if (now.second() < 10) Serial.print('0');
    Serial.print(now.second());
    Serial.print(" ");
    Serial.print(meridian);
    Serial.print(" | Date: ");
    if (month < 10) Serial.print('0');
    Serial.print(month);
    Serial.print('/');
    if (day < 10) Serial.print('0');
    Serial.print(day);
    Serial.print('/');
    Serial.println(year);

    // lcd.clear();

    // Return the time and date as a string (optional)
    char buffer[25];
    sprintf(buffer, "%02d:%02d:%02d %s | %02d/%02d/%04d", hour, now.minute(), now.second(), meridian.c_str(), month, day, year);
    return String(buffer);
}

// timestamp for sending to weatherdata table.
String getISO8601TimestampUTC() {
    DateTime utcTime = rtc.now();
    char buffer[20];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
            utcTime.year(), utcTime.month(), utcTime.day(),
            utcTime.hour(), utcTime.minute(), utcTime.second());
    return String(buffer);
}

//waterFlow sensor
float waterFlowSensor() {
  noInterrupts();
  int pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  if (pulses < 5) { // Ignore small noise pulses
      Serial.println("Noise detected on flow sensor!");
      return 0.0;
  }

  float waterFlow = (pulses / calibrationFactor);

  Serial.print("Water Flow: ");
  Serial.print(waterFlow);
  Serial.println(" L/min");

  return waterFlow;
}

// soil moisture sensor
int soilMoisture(){
  pinMode(soilMoisturePin, INPUT);
  int rawValue = analogRead(soilMoisturePin);

  // Ignore values that are too low (floating input issue)
  if (rawValue < 50) {
      Serial.println("Warning: Floating sensor input detected!");
      return -1;
  }

  Serial.print("Soil Raw Moisture: ");
  Serial.println(rawValue);

  delay(1000);
} 

//connect to wifi
void connectWifi(){
   //connect to Wi=Fi
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

// initialize the sensors
void startSensors(){
   // Welcome Message
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


// send sensor data
void sendDataToServer(float temperature, float humidity, int soilMoisture, String valveStatus, float waterFlow, String currentTime, int distance) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClientSecure client;
        client.setInsecure(); // Accept all certificates (for testing purposes). For production, you should verify the certificate.
       
        HTTPClient https;
        https.begin("https://midtermdocs-production.up.railway.app/add/sensor_data"); // Replace with your Node.js server IP or domain
        https.addHeader("Content-Type", "application/json");

        // Prepare JSON payload
        DateTime now = rtc.now();  // Get the current time from the RTC
        String timestamp = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + "T" +
                           String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "Z";

        String dataID = "D-" + String(now.unixtime()); // Generate unique data ID based on timestamp

        //soil moisture value


        // Construct JSON payload
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

        // Send the POST request
        int httpResponseCode = https.POST(jsonPayload);

        if (httpResponseCode > 0) {
            String response = https.getString();

            // Print on LCD
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Snr Response:");
            lcd.setCursor(0, 1);
            lcd.print(response);
            delay(4000);
            lcd.clear();

            // Print on Serial Monitor
            Serial.println("Server Response:");
            Serial.println(response);
        } else {
            // Print error on LCD
            lcd.clear();
            lcd.print("Error on sending POST");
            delay(4000);

            // Print error on Serial Monitor
            Serial.print("Error on sending POST: ");
            Serial.println(httpResponseCode);
        }

        https.end();
    } else {
        Serial.println("Wi-Fi Disconnected!");
    }
}

// read sensor data and send
void readSensorData(float temperature, float humidity, int soilMoistureValue, String valveStatus, float waterFlow, String currentTime,int distance){
   // Read sensor data
    // float temperature = dht.readTemperature();
    // float humidity = dht.readHumidity();
    // int soilMoistureValue = analogRead(soilMoisturePin);
    // String valveStatus = digitalRead(RELAY_PIN) ? "Open" : "Closed";

    // // Calculate water flow
    // noInterrupts();
    // int pulses = pulseCount;
    // pulseCount = 0;
    // interrupts();
    // float waterFlow = (pulses / calibrationFactor);

   // Print data to Serial Monitor and LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reading Data...");

      delay(1500);

      Serial.print("Temperature: ");
      Serial.println(temperature);
      lcd.setCursor(0, 0); // Adjust cursor for temperature
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print("C");

      delay(3000); // Allow time to read each value

      Serial.print("Humidity: ");
      Serial.println(humidity);
      lcd.setCursor(0, 1); // Adjust cursor for humidity
      lcd.print("Hum: ");
      lcd.print(humidity);
      lcd.print("%");

      delay(3000);

      Serial.print("Soil Moisture: ");
      Serial.println(soilMoistureValue);
      lcd.clear(); // Clear LCD for next data
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
      lcd.print("L/min");

      delay(3000);

      Serial.print("Tank distance: ");
      Serial.println(distance);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("T Distance: ");
      lcd.print(distance);
      lcd.print("cm");


    // Send data to server
    sendDataToServer(temperature, humidity, soilMoistureValue, valveStatus, waterFlow,currentTime,distance);

    delay(5000); // Wait before the next iteration
}

//get crop age
int calculateCropAge(DateTime plantingDate, DateTime currentDate) {
    TimeSpan age = currentDate - plantingDate;
    return age.days(); // or age in weeks/months as required
}

// Function to fetch weather data from API
void fetchWeatherData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    // Replace with your API key and desired location
    String apiKey = "81f360187f497f7d31d712d1c08bd0e1"; // OpenWeatherMap API key
    String lat = "-0.4500318";         // Replace with the latitude of the location
    String lon = "36.9163043";  
    // String city = "Nairobi";         
    String units = "metric";        // Use "imperial" for Fahrenheit
    String apiUrl = "https://api.openweathermap.org/data/2.5/weather?lat=" + lat + "&lon=" + lon + "&units=" + units + "&appid=" + apiKey;




    http.begin(apiUrl);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Weather API Response:");
      Serial.println(response);

      // Parse JSON response
      const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(4) + 350;
      DynamicJsonDocument doc(capacity);

      DeserializationError error = deserializeJson(doc, response);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      // Convert UNIX timestamp to ISO 8601 format
    long unixTime = doc["dt"].as<long>();
 
      // Extract data
      weatherTimestamp = getISO8601TimestampUTC();
      weatherTemperature = doc["main"]["temp"].as<float>();
      weatherHumidity = doc["main"]["humidity"].as<float>();
      windSpeed = doc["wind"]["speed"].as<float>();
      cloudCover = doc["clouds"]["all"].as<int>();
      forecast = doc["weather"][0]["description"].as<String>();
      weatherId = String(doc["weather"][0]["id"].as<int>());
      rainfall = doc["rain"]["1h"] | 0.0; // Get rainfall in the last 1 hour, if available
      sunshineDuration = "N/A"; // OpenWeatherMap API may not provide sunshine duration directly

      // Print extracted data
      Serial.println("Extracted Weather Data:");
      Serial.println("Timestamp: " + weatherTimestamp);
      Serial.println("Temperature: " + String(weatherTemperature));
      Serial.println("Humidity: " + String(weatherHumidity));
      Serial.println("Rainfall: " + String(rainfall));
      Serial.println("Wind Speed: " + String(windSpeed));
      Serial.println("Cloud Cover: " + String(cloudCover));
      Serial.println("Forecast: " + forecast);
      Serial.println("Weather ID: " + weatherId);

    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
      delay(3000);
    }

    http.end();
  } else {
    Serial.println("Wi-Fi Disconnected");
  }
}

// Function to send weather data to your server
void sendWeatherDataToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("https://midtermdocs-production.up.railway.app/add/weather_data"); // Replace with your server URL
    http.addHeader("Content-Type", "application/json");

    // Prepare JSON payload
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

    // Send the POST request
    int httpResponseCode = http.POST(jsonPayload);

      lcd.clear();
      lcd.backlight();
     

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server Response:");
      Serial.println(response);

      // Print on LCD
      lcd.setCursor(0, 0);
      lcd.print("Wth Response:");
      lcd.setCursor(0, 1);
      lcd.print(httpResponseCode);
      delay(4000);
      lcd.clear();

    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);

      //lcd
      lcd.setCursor(0, 0);
      lcd.print("HTTP Error:");
      lcd.setCursor(0, 1);
      lcd.print(httpResponseCode);
      delay(4000);
      lcd.clear();
    }

    http.end();
  } else {
    Serial.println("Wi-Fi Disconnected!");
  }
}

// 🌾🌾  🌾🌾  🌾🌾  🌾🌾  🌾🌾  🌾🌾  🌾🌾 algorithm to irrigate 🌾🌾  🌾🌾  🌾🌾  🌾🌾  🌾🌾 

// calculate ETo using Hargreaves Equatio 
float calculateETo(float T_mean, float T_max, float T_min) {
    // Adjusted Hargreaves equation without Ra
    float ETo = 0.0023 * (T_mean + 17.8) * sqrt(T_max - T_min);
    return ETo; // ETo in mm/day
}

// get data for the ETo formular 
void fetchLatestData(float &T_max, float &T_min, int &cloudCoverPercentage) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("https://midtermdocs-production.up.railway.app/api/latest_data");
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("Latest Data Response: " + response);

            // Parse JSON
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, response);

            if (!error) {
                T_max = doc["max_temperature"].as<float>();
                T_min = doc["min_temperature"].as<float>();
                cloudCoverPercentage = doc["cloudCover"].as<int>();
            } else {
                Serial.println("JSON parsing failed");
            }
        } else {
            Serial.print("HTTP GET failed, code: ");
            Serial.println(httpResponseCode);
        }

        http.end();
    } else {
        Serial.println("WiFi not connected");
    }
}

//calculate ETc
float calculateETc(float ETo, float Kc) {
    return ETo * Kc; // ETc in mm/day
}

//get crop id
String getCropId(String cropName) {
    if (cropName == "Maize") {
        return "C001";
    }
    // Add more mappings as needed
    else {
        return "";
    }
}

// calculate crop age to  Weeks
int calculateCropAgeWeeks(String datePlanted) {
    int year, month, day;
    sscanf(datePlanted.c_str(), "%d-%d-%d", &year, &month, &day);
    DateTime plantingDate(year, month, day);
    DateTime currentDate = rtc.now();
    TimeSpan age = currentDate - plantingDate;
    return age.days() / 7; // Convert days to weeks
}

float fetchKcValue(String cropId, int cropAgeWeeks) {
    float kcValue = 0.65; // Default Kc value

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String url = "https://midtermdocs-production.up.railway.app/api/get_kc?cropid=" + cropId + "&cropageweeks=" + String(cropAgeWeeks);
        http.begin(url);
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("Kc Value Response: " + response);

            // Parse JSON
            DynamicJsonDocument doc(512);
            DeserializationError error = deserializeJson(doc, response);

            if (!error) {
                kcValue = doc["kc"].as<float>();
            } else {
                Serial.println("JSON parsing failed");
            }
        } else {
            Serial.print("HTTP GET failed, code: ");
            Serial.println(httpResponseCode);
        }

        http.end();
    } else {
        Serial.println("WiFi not connected");
    }

    return kcValue;
}

// calculating soil moisture
float calculateSoilMoistureInMM(int soilMoisturePin, int minReading, int maxReading, float maxSoilDepthMM, int numSamples = 1) {
    int totalMoisture = 0;
    int validReadings = 0;

    // Take multiple readings for better accuracy
    for (int i = 0; i < numSamples; i++) {
        int currentReading = analogRead(soilMoisturePin);
        Serial.print("Reading ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(currentReading);

        // Only include non-zero readings
        if (currentReading > 0) {
            totalMoisture += currentReading;
            validReadings++;
        }

        delay(100); // Short delay between readings to stabilize values
    }

    // Check if there are valid readings
    if (validReadings == 0) {
        Serial.println("No valid readings available!");
        return 0; // Return 0 if all readings are zero
    }

    // Calculate the average moisture value
    int averageMoistureValue = totalMoisture / validReadings;

    // Debug: Print the average value
    Serial.print("Average Moisture Value (excluding zeros): ");
    Serial.println(averageMoistureValue);

    // Convert average value to a percentage
    float moisturePercentage = map(averageMoistureValue, minReading, maxReading, 0, 100);

    // Debug: Print the percentage
    Serial.print("Mapped Moisture Percentage: ");
    Serial.println(moisturePercentage);

    // Ensure the percentage is within valid bounds (0-100%)
    moisturePercentage = constrain(moisturePercentage, 0, 100);

    // Calculate equivalent soil moisture in mm
    float moistureInMM = (moisturePercentage / 100.0) * maxSoilDepthMM;

    // Debug: Print the final moisture in mm
    Serial.print("Calculated Soil Moisture (mm): ");
    Serial.println(moistureInMM);

    return moistureInMM;
}

// Function to handle key-specific actions
void handleKeyPress(char key) {
    switch (key) {
        case '1': // Display time
            lcd.backlight();
            // lcd.print("Key pressed 1");
            // Serial.print("Key pressed is 1");
            // delay(2000);
            lcd.clear();
            displayTime();
            delay(3500);  // Allow user to read the time
            break;

        case '2': // Display temperature and humidity
            lcd.backlight();
            humidityTemp(temperature, humidity);
            lcd.noBacklight();
            lcd.clear();
            break;

        case '3': // Display valve status
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

        case '4': // Display tank distance
        {
            int distance = getDistance();
            lcd.clear();
            lcd.backlight();
            lcd.setCursor(0, 0);
            lcd.print("Water Distance :");
            lcd.setCursor(0, 1);
            lcd.print(distance);
            lcd.print("cm");
            delay(3500);
            lcd.noBacklight();
            lcd.clear();
        }
        break;

        case '5': // Display water flow rate
        {
            float waterFlow = waterFlowSensor();
            lcd.clear();
            lcd.backlight();
            lcd.setCursor(0, 0);
            lcd.print("Water Flow  :");
            lcd.setCursor(0, 1);
            lcd.print(waterFlow);
            lcd.print("L/min");
            delay(3500);
            lcd.noBacklight();
            lcd.clear();
        }
        break;

        case '6': // Test solenoid valve
            lcd.backlight();
            lcd.clear();
            lcd.print("Key pressed: ");
            lcd.println(key);
            Serial.println("$$$$$$$$$$$$$$$$$$ Solenoid Activated");
            digitalWrite(RELAY_PIN, HIGH);
            delay(10000);
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("Solenoid deactivated");
            break;

        case '7': // Display soil moisture
        {
            lcd.backlight();
            lcd.clear();
            int soilMoistureVal = analogRead(soilMoisturePin);
            // int soilMoistureMM = calculateSoilMoistureInMM(soilMoisturePin,minReading,maxReading, maxSoilDepthMM);
            int rawSoil = soilMoisture();
            Serial.print("Soil moisture in MM: ");
            Serial.println(rawSoil);
            lcd.setCursor(0,0);
            lcd.print("Moisture in mm:");
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

        default: // For other keys, do nothing
            Serial.println("Invalid key pressed");
            break;
    }
}




// 🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁🏁
void setup() {
Serial.begin(115200);
  
  //seting up the 5v relay
  pinMode(RELAY_PIN,OUTPUT); // set this pin as an output
  digitalWrite(RELAY_PIN,LOW);  // the solenoid starts closed

  //initialize the Flow sensor
  pinMode(FLOW_SENSOR_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN),countPulse,RISING);

  Serial.println("flow Sensor and relay Setup Complete");

   // initialize the LCD
  lcd.init();
  lcd.backlight(); // turn on the backlight.

  //connecting to wifi
  connectWifi();

  // login
    if (!userLoggedIn) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Please Login");
        delay(4000);
        userLogin(); // Prompt for login
    }

  // initialize the sensors
  startSensors();

   //board status
    boardStatus();
  
  //initialize the RTC module
  timeModuleCheck();

 // turn off backlight of lcd
   lcd.noBacklight();
    pinMode(soilMoisturePin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT); // sends signal from ESP to sensor
    pinMode(ECHO_PIN, INPUT);  // gets data/signal from the sensor.
     
  // Initialize DHT Sensor
  dht.begin();
  checkDHT();

  // Initialize all readings to zero
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }


}
// 🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂
void loop() {

   // powering esp32
    if (millis() - lastFetchTime >= fetchInterval) {
        Serial.println("Refreshing controller state...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Refreshing...");
        
        fetchControllerState();
        lastFetchTime = millis();
        
        lcd.clear();
    }


  // regular keypad intervals
    if (millis() - previousKeypadTime >= keypadInterval) {
        previousKeypadTime = millis();
        char key = keypad.getKey();  // Check for key press
        if (key) {
            handleKeyPress(key);  // Handle key press actions
        }
    }

   // If logged in, handle the logout action
    if (!userLoggedIn) {
        // If not logged in, prompt for login
        userLogin();  
        return;       
    }

// Serial.println(String("<<<<<>>>>>>>>>>") + key);


// turn of the screen
 if (millis() - lastKeyPressTime > backlightTimeout) {
    lcd.noBacklight();  // Turn off the backlight to save power
  }


// measure distance after every 3 seconds
  if(millis() - lastDistanceTime >= SonarInterval){
    getDistance();
    lastDistanceTime = millis();
  }
 

  //send weather data
  if(millis() - lastpullTime >= sendWeatherInterval){
     fetchWeatherData();           // Fetch data from the weather API
     sendWeatherDataToServer();    // Send the fetched data to your server
    lastpullTime = millis();

  }

    // send sensor data
    if (millis() - lastSendTime >= sendInterval) {

       // get the humidity and temperature
        float temperature, humidity;
        humidityTemp(temperature, humidity);
      
      // get current time. 
        String currentTime = displayTime(); 

       // get the soil moisture value 
        int soilMoistureValue = analogRead(soilMoisturePin);

        // get the valve status
        String valveStatus = digitalRead(RELAY_PIN) ? "Open" : "Closed";

        // get distance of the tank
        int distance = getDistance();

      // get the flow rate
        float waterFlow = waterFlowSensor();

        // send data to readSensor to be displayed on the lcd before sending.
        readSensorData(temperature, humidity, soilMoistureValue, valveStatus, waterFlow, currentTime, distance);
        lastSendTime = millis();
    }

  // here we read the analog pin value.
     int soilMoistureValue = analogRead(soilMoisturePin);

      if (millis() - lastSoilCheckTime >= soilCheckInterval) {
              lastSoilCheckTime = millis(); // Update the last check time


              // Check if soil moisture is below the threshold
              if (soilMoistureValue < soilMoistureThreshold) {
                  Serial.println("Soil moisture below threshold, fetching data...");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("Soil moisture");
                  lcd.setCursor(0,1);
                  lcd.println("Low..");
                  delay(3000);
                  
                  // Start state irrigation proccess
                  irrigationState = IRRIGATION_RUNNING;

                  // Calculate crop age in weeks
                  int cropAgeWeeks = calculateCropAgeWeeks(datePlanted);

                  // Fetch the crop coefficient (Kc) based on crop age
                  float Kc = fetchKcValue(cropId, cropAgeWeeks);

                  // Fetch the latest weather data
                  float T_max, T_min;
                  int cloudCoverPercentage;
                  fetchLatestData(T_max, T_min, cloudCoverPercentage);

                  // Calculate T_mean
                  float T_mean = (T_max + T_min) / 2.0;

                  // Calculate ETo using the fetched weather data
                  float ETo = calculateETo(T_mean, T_max, T_min);
                  Serial.print("Calculated ETo: ");
                  Serial.println(ETo);

                  // Recalculate Kc if necessary (e.g., crop age updated)
                  cropAgeWeeks = calculateCropAgeWeeks(datePlanted);
                  Kc = fetchKcValue(cropId, cropAgeWeeks);
                  Serial.print("Kc Value: ");
                  Serial.println(Kc);

                  // Calculate ETc based on ETo and Kc
                  float ETc = ETo * Kc;
                  Serial.print("Calculated ETc: ");
                  Serial.println(ETc);

              } else {
                  Serial.println("Soil moisture is sufficient. No need to fetch data.");
              }
          }

  // irrigation state switch case.   

    switch(irrigationState) {
      case IRRIGATION_IDLE:
            if (soilMoistureValue < soilMoistureThreshold) {
                  // Start irrigation
                  digitalWrite(RELAY_PIN, HIGH); // Open valve
                  irrigationStartTime = millis();
                  irrigationState = IRRIGATION_RUNNING;
                  Serial.println("Irrigation started");

                  // Display on LCD
                  lcd.clear();
                  lcd.backlight();
                  lcd.print("Irrigation ON");
              }
              break;

        case IRRIGATION_RUNNING:
              if (millis() - irrigationStartTime >= irrigationDurationMillis) {
                  // Stop irrigation
                  digitalWrite(RELAY_PIN, LOW); // Close valve
                  irrigationState = IRRIGATION_WAITING;
                  Serial.println("Irrigation stopped");

                  // Display on LCD
                  lcd.clear();
                  lcd.print("Irrigation OFF");

                  // Start waiting period
                  irrigationStartTime = millis();
              }
              break;

        case IRRIGATION_WAITING:
              if (millis() - irrigationStartTime >= 5000) { // Wait 5 seconds
                  // Check soil moisture again
                  soilMoistureValue = analogRead(soilMoisturePin);
                  if (soilMoistureValue < soilMoistureThreshold) {
                      // Start irrigation again
                      digitalWrite(RELAY_PIN, HIGH); // Open valve
                      irrigationStartTime = millis();
                      irrigationState = IRRIGATION_RUNNING;
                      Serial.println("Irrigation restarted");

                      // Display on LCD
                      lcd.clear();
                      lcd.print("Irrigation ON");
                  } else {
                      // Soil moisture is sufficient
                      irrigationState = IRRIGATION_IDLE;
                      Serial.println("Soil moisture sufficient");

                      // Display on LCD
                      lcd.clear();
                      lcd.print("Soil Moist OK");
                      delay(2000);
                      lcd.noBacklight();
                  }
              }
              break;
      }


  delay(50);  // Delay for stability between readings
}
