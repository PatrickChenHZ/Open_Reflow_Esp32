#include <Arduino.h>
// For ESP32 Bluetooth
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;

bool inreflow = false;
bool relaypwr = false;
bool ventpwr = false;
bool vent_interlock = false;

String bake_p = "";

bool tarmode = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/*
// Libraries for MAX31855 thermocouple interface
#include <SPI.h>
SPIClass SPI2(HSPI); // We are using HSPI pins on the ESP32
*/


#include "Adafruit_MAX31856.h" // https://github.com/adafruit/Adafruit-MAX31855-library

// Library for PID control
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

// Define pins
#define relay 15
#define vent_fan 26

#define oled_scl 22
#define oled_sda 21

#define LED 2 // This LED is used to indicate if the reflow process is underway
#define MAX_CLK 14 // MAX31855 clock pin
#define MAX_CS 27 // MAX31855 chip select pin
#define MAX_DO 12 // MAX31855 data pin (HSPI MISO on ESP32)
#define MAX_DI 13 // MAX31856 data pin (HSPI MOSI on ESP32)

// Initialize Bluetooth software serial

// Initialize thermocouple
// Software SPI used
Adafruit_MAX31856 Kthermocouple = Adafruit_MAX31856(MAX_CS, MAX_DI, MAX_DO, MAX_CLK);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Define reflow temperature profile parameters (in *C)
// If needed, define a subtraction constant to compensate for overshoot:
int T_const=2; // From testing, overshoot was about 5-6*C

// Standard lead-free solder paste (melting point around 215*C)
int T_preheat=150;
int T_soak=217;
int T_reflow_s = 249;
int T_reflow=T_reflow_s - T_const;

int T_vent_start = 125;

// "Low-temp" lead-free solder paste (melting point around 138*C)
//#define T_preheat 90
//#define T_soak 138
//#define T_reflow 165 - T_const

// Test values to make sure your Reflowduino is actually working
//#define T_preheat 50
//#define T_soak 80
//#define T_reflow 100 - T_const

#define T_cool 40 // Safe temperature at which the board is "ready" (dinner bell sounds!)
#define preheat_rate 2 // Increase of 1-3 *C/s
#define soak_rate 0.7 // Increase of 0.5-1 *C/s
#define reflow_rate 2 // Increase of 1-3 *C/s
#define cool_rate -4 // Decrease of < 6 *C/s max to prevent thermal shock. Negative sign indicates decrease

// Define PID parameters. The gains depend on your particular setup
// but these values should be good enough to get you started
#define PID_sampleTime 1000 // 1000ms = 1s
// Preheat phase
#define Kp_preheat 150
#define Ki_preheat 0
#define Kd_preheat 100
// Soak phase
#define Kp_soak 200
#define Ki_soak 0.05
#define Kd_soak 300
// Reflow phase
#define Kp_reflow 300
#define Ki_reflow 0.05
#define Kd_reflow 350

// Bluetooth app settings. Define which characters belong to which functions
#define dataChar "*" // App is receiving data from Reflowduino
#define stopChar "!" // App is receiving command to stop reflow process (process finished!)
#define startReflow "A" // Command from app to "activate" reflow process
#define stopReflow "S" // Command from app to "stop" reflow process at any time
#define configReflow "C" // Command from app to reconfigure reflow,and indicate T-Const
#define config_preheat "P" //indicate T-Preheat
#define config_soak "K" //indicate T-Soak
#define config_reflow "R" //indicate T-Reflow
#define startTarget "V" //Command from app to strat target mode
#define stopTarget "G" //Command from app to stop target mode

double temperature, chiptemperature, output, setPoint; // Input, output, set point
PID myPID(&temperature, &output, &setPoint, Kp_preheat, Ki_preheat, Kd_preheat, DIRECT);

// Logic flags
bool justStarted = true;
bool reflow = false; // Baking process is underway!
bool preheatComplete = false;
bool soakComplete = false;
bool reflowComplete = false;
bool coolComplete = false;

double T_start; // Starting temperature before reflow process
int windowSize = 2000;
unsigned long sendRate = 2000; // Send data to app every 2s
unsigned long t_start = 0; // For keeping time during reflow process
unsigned long previousMillis = 0;
unsigned long duration, t_final, windowStartTime, timer;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.print("<-- Received Value: ");

      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
      }
      Serial.println();

      // Do stuff based on the command received from the app
      if (rxValue.find(startReflow) != -1) { // Command from app to start reflow process
        justStarted = true;
        reflow = true; // Reflow started!
        t_start = millis(); // Record the start time
        timer = millis(); // Timer for logging data points
        Serial.println("<-- ***Reflow process started!"); // Left arrow means it received a command
      }
      else if (rxValue.find(stopReflow) != -1) { // Command to stop reflow process
        digitalWrite(relay, LOW); // Turn off appliance and set flag to stop PID control
        reflow = false;
        relaypwr = false;
        Serial.println("<-- ***Reflow process aborted!");
      }
      else if (rxValue.find(startTarget) != -1){
        //WIP
      }
      else if (rxValue.find(stopTarget) != -1){
        //WIP
      }
      else if (rxValue.find(configReflow) != -1) { // Command to stop reflow process
        //configure
        T_const = stoi(rxValue.substr(rxValue.find("C")+1,(rxValue.find("P")-rxValue.find("C"))-1));
        T_preheat = stoi(rxValue.substr(rxValue.find("P")+1,(rxValue.find("K")-rxValue.find("P"))-1));
        T_soak = stoi(rxValue.substr(rxValue.find("K")+1,(rxValue.find("R")-rxValue.find("K"))-1));
        T_reflow_s = stoi(rxValue.substr(rxValue.find("R")+1,(rxValue.length()-rxValue.find("R"))-2));
        T_reflow=T_reflow_s - T_const;
        Serial.print("<-- Config Received const:");
        Serial.print(T_const);
        Serial.print(" preheat: ");
        Serial.print(T_preheat);
        Serial.print(" soak: ");
        Serial.print(T_soak);
        Serial.print(" reflow: ");
        Serial.println(T_reflow);
      }
      // Add you own functions here and have fun with it!
    }
  }
};

void setup() {
  Serial.begin(115200);

//  while (!Serial) delay(1); // OPTIONAL: Wait for serial to connect
  Serial.println("*****ESP32 UP*****");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(25,20);
  display.println("Reflow");
  display.setCursor(45,45);
  display.setTextSize(1);
  display.println("v0.1");
  display.display();
  delay(2000);

  if (!Kthermocouple.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  Kthermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  pinMode(LED, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(vent_fan, OUTPUT);

  digitalWrite(LED, LOW);
  inreflow = false;
  digitalWrite(relay, LOW); // Set default relay state to OFF
  relaypwr = false;
  digitalWrite(vent_fan, LOW);
  ventpwr = false;

  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTime(PID_sampleTime);
  myPID.SetMode(AUTOMATIC); // Turn on PID control

  /***************************** BLUETOOTH SETUP *****************************/
  // Create the BLE Device
  BLEDevice::init("Reflowduino32"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a BLE client to connect...");
}

void update_disp(){
  display.clearDisplay();

  String oven_sta = "";
  display.setTextSize(2);
  display.setCursor(10,3);
  display.setTextColor(WHITE);
  if(!inreflow){
    display.println("IDLE");
  }
  else{
    display.println(bake_p);
  }
  display.drawRect(0, 0, 128, 20, WHITE);
  //Temperture + Relay + Fan
  if(relaypwr){
    oven_sta="HT: ON";
  }
  else{
    oven_sta="HT: OFF";
  }
  display.setTextSize(1);
  display.setCursor(3,25);
  display.setTextColor(WHITE);
  display.println((String)("T:") + (String)(temperature));
  display.setCursor(3,35);
  display.println((oven_sta));
  if(ventpwr){
    oven_sta="ON";
  }
  else{
    oven_sta="OFF";
  }
  display.setCursor(3,45);
  display.println((String)("Fan:") + (String)(oven_sta));
  display.drawRect(0, 20, 64, 40, WHITE);
  //thermal profile
  display.drawRect(64, 20, 64, 40, WHITE);
  display.setCursor(66,24);
  display.println((String)("Pre:") + (String)(T_preheat));
  display.setCursor(66,32);
  display.println((String)("Sk:") + (String)(T_soak));
  display.setCursor(66,40);
  display.println((String)("Rf:") + (String)(T_reflow));
  display.setCursor(66,48);
  display.println((String)("C:") + (String)(T_const));

  display.display();
}

void loop() { 
  /***********************************TARGET TEMP MODE****************************/
  if (tarmode){
    //WIP
  }
  else{
    //WIP
  }
  /***************************** REFLOW PROCESS CODE *****************************/
  if (reflow) {
    digitalWrite(LED, HIGH); // Blue LED indicates reflow is underway
    inreflow = true;

    // This only runs when you first start the reflow process
    if (justStarted) {
      justStarted = false;
      
      t_start = millis(); // Begin timers
      windowStartTime = millis();
      T_start = temperature;
      
      if (isnan(T_start)) {
       Serial.println("Invalid reading, check thermocouple!");
      }
      else {
       Serial.print("Starting temperature: ");
       Serial.print(T_start);
       Serial.println(" *C");
      }
    }

    // Determine the amount of time that elapsed in any particular phase (preheat, soak, etc)
    duration = millis() - t_start;

    // Determine the desired set point according to where are in the reflow process
    // Perform a linear extrapolation of what desired temperature we want to be at.
    /********************* PREHEAT *********************/
    if (!preheatComplete) {
      if (temperature >= T_preheat) { // Check if the current phase was just completed
        preheatComplete = true;
        t_start = millis(); // Reset timer for next phase
        Serial.println("Preheat phase complete!");
      }
      else {
        // Calculate the projected final time based on temperature points and temperature rates
        t_final = (T_preheat - T_start) / (preheat_rate / 1000.0) + t_start;
        // Calculate desired temperature at that instant in time using linear interpolation
        setPoint = duration * (T_preheat - T_start) / (t_final - t_start);
        bake_p="A:PREHEAT";
      }
    }
    /********************* SOAK *********************/
    else if (!soakComplete) {
      if (temperature >= T_soak) {
        soakComplete = true;
        t_start = millis();
        Serial.println("Soaking phase complete!");
      }
      else {
        t_final = (T_soak - T_start) / (soak_rate / 1000.0) + t_start;
        setPoint = duration * (T_soak - T_start) / (t_final - t_start);
        bake_p="A:SOAK";
      }
    }
    /********************* REFLOW *********************/
    else if (!reflowComplete) {
      if (temperature >= T_reflow) {
        reflowComplete = true;
        t_start = millis();
        Serial.println("Reflow phase complete!");
      }
      else {
        t_final = (T_reflow - T_start) / (reflow_rate / 1000.0) + t_start;
        setPoint = duration * (T_reflow - T_start) / (t_final - t_start);
        bake_p="A:REFLOW";
      }
    }
    /********************* COOLDOWN *********************/
    else if (!coolComplete) {
      if (temperature <= T_cool) {
        coolComplete = true;
        reflow = false;
        Serial.println("PCB reflow complete!");
        vent_interlock = false;
        
        // Tell the app that the entire process is finished!
        pCharacteristic->setValue(stopChar);
        pCharacteristic->notify(); // Send value to the app
      }
      else {
        t_final = (T_cool - T_start) / (cool_rate / 1000.0) + t_start;
        setPoint = duration * (T_cool - T_start) / (t_final - t_start);
        bake_p="A:COOL";
      }
    }
    //activate cooling fan
    //deactivate when temp is below 8+cooling target
    if(bake_p == "A:COOL" && !coolComplete && !vent_interlock){
      if(temperature <= T_vent_start){
        if(temperature >= T_cool + 8){
          digitalWrite(vent_fan, HIGH);
          ventpwr = true;
        }
        else{
          digitalWrite(vent_fan, LOW);
          ventpwr = false;
          //prevent repeated relay action
          vent_interlock = true;
        }
      }
    }

    // Use the appropriate PID parameters based on the current phase
    if (!soakComplete) myPID.SetTunings(Kp_soak, Ki_soak, Kd_soak);
    else if (!reflowComplete) myPID.SetTunings(Kp_reflow, Ki_reflow, Kd_reflow);
    
    // Compute PID output (from 0 to windowSize) and control relay accordingly
    myPID.Compute(); // This will only be evaluated at the PID sampling rate
    if (millis() - windowStartTime >= windowSize) windowStartTime += windowSize; // Shift the time window
    if (output > millis() - windowStartTime){
      digitalWrite(relay, HIGH); // If HIGH turns on the relay
      relaypwr = true;
      //Serial.println("RL ON");
    }
    //if (output < millis() - windowStartTime) digitalWrite(relay, HIGH); // If LOW turns on the relay
    else{
      digitalWrite(relay, LOW);
      relaypwr = false;
      //Serial.println("RL OFF");
    }
  }
  else {
    //REFLOW STOP
    digitalWrite(LED, LOW);
    digitalWrite(relay, LOW);
    relaypwr = false;
    inreflow = false;
    //Serial.println("RL OFF");
  }

  /***************************** BLUETOOTH CODE *****************************/
  // Send data to the app periodically
  if (millis() - previousMillis > sendRate) {
    previousMillis = millis();

    /***************************** MEASURE TEMPERATURE *****************************/
    temperature = Kthermocouple.readThermocoupleTemperature(); // Read temperature
    //temperature = thermocouple.readFarenheit(); // Alternatively, read in deg F but will need to modify code
    chiptemperature = Kthermocouple.readCJTemperature();
    
    // Check and print any faults
    uint8_t fault = Kthermocouple.readFault();
    if (fault) {
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
    }

    Serial.print("--> Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
    update_disp();

     // Only send temperature value if it's legit and if connected via BLE
    if (!isnan(temperature) && deviceConnected) {
      // First convert the value to a char array
      char tempBuff[16]; // make sure this is big enuffz
      char txString[16];
      dtostrf(temperature, 1, 2, tempBuff); // float_val, min_width, digits_after_decimal, char_buffer
      sprintf(txString, "%s%s", dataChar, tempBuff); // The 'dataChar' character indicates that we're sending data to the app
      
      pCharacteristic->setValue(txString);
      pCharacteristic->notify(); // Send the value to the app
    }
  }
  // Commands sent from the app are checked in the callback function when they are received
}