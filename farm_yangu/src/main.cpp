#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DHT.h>
#include <RTClib.h> // for rtc module.
#include <Wire.h> // thsi is for I2C communication pins
#include <WiFi.h> // this is the wifi module
#include <HTTPClient.h> // for api's
#include <ArduinoJson.h> // for JSON parsing

// wifi connection details
const char* ssid = "Nexacore";
const char* password ="Nexacore.@123";


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

//time for sending data
unsigned long lastSendTime = 0;  // Tracks the last time data was sent
const unsigned long sendInterval = 600000;  // 10 min interval

//time for weather
unsigned long lastpullTime = 0;  // Tracks the last time data was pulled and sent
const unsigned long sendWeatherInterval = 300000;  // 5min interval

float temperature = 0.0; // Global variable for temperature
float humidity = 0.0;    // Global variable for humidity

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


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>fuctions
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
    DateTime now = rtc.now();  // Get the current time

    // Display time on the LCD
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0);  // Set cursor to the beginning to avoid clearing

    // Print time to LCD in format HH:MM:SS
    lcd.print("Time: ");
    if (now.hour() < 10) lcd.print('0'); // Add leading zero for single-digit hours
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if (now.minute() < 10) lcd.print('0'); // Add leading zero for single-digit minutes
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    if (now.second() < 10) lcd.print('0'); // Add leading zero for single-digit seconds
    lcd.print(now.second(), DEC);

    // Print time to Serial Monitor in the same format
    Serial.print("Time: ");
    if (now.hour() < 10) Serial.print('0');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    if (now.minute() < 10) Serial.print('0');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    if (now.second() < 10) Serial.print('0');
    Serial.println(now.second(), DEC);
    
    char buffer[20];
    sprintf(buffer, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    return String(buffer);

    delay(3000);  // Refresh every second
}


//waterFlow sensor
float waterFlowSensor(){
    noInterrupts();
        int pulses = pulseCount;
        pulseCount = 0;
        interrupts();

        float waterFlow = (pulses / calibrationFactor);

        return waterFlow;
}

// soil moisture sensor
void soilMoisture(){
//soil moisture sensor
int soilMoistureValue = analogRead(soilMoisturePin);
Serial.print("Soil Moisture: ");
Serial.println(soilMoistureValue);

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
    Serial.print(".");
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
  lcd.setCursor(4,0);
  lcd.print("Welcome ");
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
            lcd.print("Server Response:");
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


// Function to fetch weather data from API
void fetchWeatherData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    // Replace with your API key and desired location
    String apiKey = "81f360187f497f7d31d712d1c08bd0e1"; // OpenWeatherMap API key
    String city = "Nairobi";         // Replace with your city
    String units = "metric";        // Use "imperial" for Fahrenheit
    String apiUrl = "http://api.openweathermap.org/data/2.5/weather?q=" + city + "&units=" + units + "&appid=" + apiKey;

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

      // Extract data
      weatherTimestamp = String(doc["dt"].as<long>()); // Unix timestamp
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

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server Response:");
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("Wi-Fi Disconnected!");
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

  // initialize the sensors
  startSensors();

  //initialize the RTC module
  timeModuleCheck();


 lcd.noBacklight();
  pinMode(soilMoisturePin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); // sends signal from ESP to sensor
  pinMode(ECHO_PIN, INPUT);  // gets data/signal from the sensor.
  
  

  // Initialize DHT Sensor
  dht.begin();
  checkDHT();



  //print startup message
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Keypad,LCD Test");
  // lcd.setCursor(0,1);
  // lcd.print("Press a key...");

  // Initialize all readings to zero
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }


}
// 🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂🔂
void loop() {
  // check if a key is pressed
  char key = keypad.getKey();

  // //display time
  // lcd.backlight();
  // displayTime();
  // delay(3000);

  Serial.println("<<<<<>>>>>>>>>>"+key);


// turn of the screen
 if (millis() - lastKeyPressTime > backlightTimeout) {
    lcd.noBacklight();  // Turn off the backlight to save power
  }


// measure distance after every 3 seconds
  if(millis() - lastDistanceTime >= SonarInterval){
    getDistance();
    lastDistanceTime = millis();
  }
 
  // test Solenoid valve
  if (key == '6'){
    lcd.backlight();
    lcd.clear();
    lcd.print("Key pressed: ");
    lcd.println(key);
    Serial.println("$$$$$$$$$$$$$$$$$$ Solenoid Activated");
    digitalWrite(RELAY_PIN,HIGH);
    delay(10000);
    digitalWrite(RELAY_PIN,LOW);
    Serial.println("Solenoid deactivated");
  }

 // display the time
  if (key == '1'){
    displayTime();
    delay(3500);
    lcd.clear();
  }

//display temp and humidity
  if(key == '2'){
    lcd.backlight();
    humidityTemp(temperature,humidity);
    lcd.noBacklight();
    lcd.clear();
  }

// display the valve status
  if(key == '3'){
     String valveStatus = digitalRead(RELAY_PIN) ? "Open" : "Closed";
     lcd.clear();
     lcd.backlight();
     lcd.setCursor(0,0);
     lcd.print("Valve:");
     lcd.setCursor(0,1);
     lcd.print(valveStatus);
     delay(5000);
     lcd.noBacklight();
     lcd.clear();
  }

  // display tank distance
  if(key == '4'){
    int distance = getDistance();
     lcd.clear();
     lcd.backlight();
     lcd.setCursor(0,0);
     lcd.print("Water Distance :");
     lcd.setCursor(0,1);
     lcd.print(distance);
     lcd.print("cm");
     delay(3500);
     lcd.noBacklight();
     lcd.clear();
  }

  // display the flow rate

  if(key == '5'){
     float waterFlow = waterFlowSensor();
     lcd.clear();
     lcd.backlight();
     lcd.setCursor(0,0);
     lcd.print("Water Flow  :");
     lcd.setCursor(0,1);
     lcd.print(waterFlow);
     lcd.print("L/min");
     delay(3500);
     lcd.noBacklight();
     lcd.clear();

  }
  //send weather data
  if(millis() - lastpullTime >= sendWeatherInterval){

     fetchWeatherData();           // Fetch data from the weather API
     sendWeatherDataToServer();    // Send the fetched data to your server

    lastpullTime = millis();

  }


    // Update sensor data every `sendInterval`
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

  delay(50);  // Delay for stability between readings
}
