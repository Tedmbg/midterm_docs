#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DHT.h>
#include <RTClib.h> // for rtc module.
#include <Wire.h> // thsi is for I2C communication pins
#include <WiFi.h> // this is the wifi module
#include <HTTPClient.h> // for api's

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

// timing of the DHT
const unsigned long DHTInterval = 6000;  // Read DHT every 2 seconds
unsigned long lastDHTReadTime = 0;

// timing for the screen time out
unsigned long lastKeyPressTime = 0;  // Stores the time of the last key press
const unsigned long backlightTimeout = 7000;  // 4 seconds timeout

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
const unsigned long sendInterval = 60000;  // 60 seconds interval



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

    return averageDistance;
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

    delay(1000);  // Refresh every second
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
      lcd.print("L/min");


    // Send data to server
    sendDataToServer(temperature, humidity, soilMoistureValue, valveStatus, waterFlow,currentTime,distance);

    delay(5000); // Wait before the next iteration
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

// if key press it is displayed
// if(key){
//   lcd.backlight();
//   lastKeyPressTime = millis();  // Reset the timer
//   lcd.clear();
//   lcd.setCursor(0,0); // set the column and the row
//   lcd.print("Key Pressed: ");
//   lcd.print(key); // the key which has been pressed
//   lcd.clear();
// }
// turn of the screen
 if (millis() - lastKeyPressTime > backlightTimeout) {
    lcd.noBacklight();  // Turn off the backlight to save power
  }

// // Only read DHT every 4 seconds
//   if (millis() - lastDHTReadTime >= DHTInterval) {
//     float temperature, humidity;
//     humidityTemp(temperature,humidity);  // Read DHT sensor
//     lastDHTReadTime = millis();  // Update last read time
//   }

// measure distance after every 3 seconds
  if(millis() - lastDistanceTime >= SonarInterval){
    getDistance();
    lastDistanceTime = millis();
  }
 
  // test Solenoid valve
  if (key == '5'){
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

  if (key == '2'){
    displayTime();
    delay(3500);
  }

  //soilMoisture
  //soil moisture sensor
  int soilMoistureValue = analogRead(soilMoisturePin);
  Serial.print("Soil Moisture: ");
  Serial.println(soilMoistureValue);

  delay(1000);

  //valve status
  String valveStatus = digitalRead(RELAY_PIN) ? "Open" : "Closed";


  // test the flow sensor
  // Check if 1 second has passed since the last flow rate calculation
  // if (millis() - lastFlowReadTime >= flowInterval) {
  //   // Disable interrupts to avoid conflict while reading the pulse count
  //   noInterrupts();  
  //   int pulses = pulseCount;  // Make a local copy of the pulse count
  //   pulseCount = 0;  // Reset the pulse count after reading
  //   interrupts();  // Re-enable interrupts

  //   // Calculate flow rate in liters per second
  //   float waterFlow = (pulses / calibrationFactor);  // Pulses counted in 1 second

  //   // Print the flow rate to the Serial Monitor
  //   Serial.print("Flow Rate: ");
  //   Serial.print(waterFlow);
  //   Serial.println(" L/sec");

  //   // Update the last read time
  //   lastFlowReadTime = millis();
  // }

 // Get current time
  String currentTime = displayTime();

  // return tank distance
  int distance = getDistance();

  // get water flow value
   noInterrupts();  
    int pulses = pulseCount;  // Make a local copy of the pulse count
    pulseCount = 0;  // Reset the pulse count after reading
    interrupts();  // Re-enable interrupts

    // Calculate flow rate in liters per second
    float waterFlow = (pulses / calibrationFactor);  // Pulses counted in 1 second

  // get temp and humidity
  float temperature,humidity;
  humidityTemp(temperature,humidity);  // Read DHT sensor

    // Send data every 10 seconds
    if (millis() - lastSendTime >= sendInterval) {
        readSensorData(temperature, humidity, soilMoistureValue, valveStatus, waterFlow, currentTime, distance);
        lastSendTime = millis();  // Reset the timer
    }

  delay(50);  // Delay for stability between readings
}
