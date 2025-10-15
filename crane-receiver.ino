
/*
Author: Bello  shehu
Title: Crane remote control receiver
Description: Receiver circuit using NRF for remote control of mini-crane. 
Date: 30/07/2025
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <NewPing.h>


# define ACTIVE_LOW_ON LOW // Active low value for ON state
# define ACTIVE_LOW_OFF HIGH // Active low value for OFF state

# define ACTIVE_HIGH_ON HIGH // Active HIGH value for ON state
# define ACTIVE_HIGH_OFF LOW // Active HIGH value for OFF state


// --- DRIVER MODULE 1 (WHEEL) PINS---
#define STEERING_PIN1 16
#define STEERING_PIN2 17
#define WHEELS_PIN1 14
#define WHEELS_PIN2 15

// --- LED PINS
#define LED_SIGNAL 18 
#define LED_POWER 19

// --- ULTRASONIC SENSOR (BACK) PINS ---
#define UL_BACK_TRIG_PIN 1 // BACK TRIGGER PIN
#define UL_BACK_ECHO_PIN 0 // BACK ECHO PIN

// --- ULTRASONIC SENSOR (FRONT RIGHT) PINS ---
#define UL_FR_TRIG_PIN 2 // FRONT-RIGHT TRIGGER PIN
#define UL_FR_ECHO_PIN 3 // FRONT-RIGHT ECHO PIN

// --- ULTRASONIC SENSOR (FRONT LEFT) PINS ---
#define UL_FL_TRIG_PIN 4 // FRONT-LEFT TRIGGER PIN
#define UL_FL_ECHO_PIN 5 // FRONT-LEFT ECHO PIN

#define MAX_DISTANCE 100
#define OBSTACLE_MIN_DISTANCE 10

NewPing frontRightSonar(UL_FR_TRIG_PIN, UL_FR_ECHO_PIN, MAX_DISTANCE);
NewPing frontLeftSonar(UL_FL_TRIG_PIN, UL_FL_ECHO_PIN, MAX_DISTANCE);
NewPing backSonar(UL_BACK_TRIG_PIN, UL_BACK_ECHO_PIN, MAX_DISTANCE);


// --- SERIAL-PARALLEL CONVERTER (74HC595) PINS ---

#define DATA_PIN 6 // 7hc595 pin 14: Serial data in 
#define CLOCK_PIN 7 // 7hc595 pin 11
#define LATCH_PIN 8 // 7hc595 pin 12

// --- NRF MODULE ---
#define NRF_MISO_PIN 12
#define NRF_MOSI_PIN 11
#define NRF_SCK_PIN 13
#define NRF_CSN_PIN 10
#define NRF_CE_PIN 9

// --- LED PINS
#define LED_SIGNAL 18
#define LED_POWER 19

// --- VOLTAGE SEBNSOR
#define VOLT_SENSOR_PIN A7
// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
// Float for Reference Voltage
float ref_voltage = 5.0;
// Integer for ADC value
int adc_value = 0;

// data for various motor states
// boom up: 1
// boom down: 2
// rotation left: 4
// rotation right: 8
// hook up: 16
// hook down: 32

/*
  The following values should be assigned according to the type of the relay module to be used (active high or active low):

  - rotateLeft (4 for active high, 251 for active low)
  - rotateLeft (8 for active high, 119 for active low)
  - boomUp (1 for active high, 254 for active low)
  - boomDown (2 for active high, 253 for active low)
  - hookUp (16 for active high, 191 for active low)
  - hookDown (32 for active high, 127 for active low)
*/
const byte rotateLeft = 251;  // rotate left only 00000100. Active low val: 251 
const byte rotateRight = 119;  // rotate left only 0001000. Active low val: 119

const byte boomUp = 254;  // rotate left only 00000001. Active low val: 254
const byte boomDown = 253;  // rotate left only 00000010. Active low val: 253

const byte hookUp = 191;  // rotate left only 01000000. Active low val: 191
const byte hookDown = 223;  // rotate left only 10000000. Active low val: 127
const byte resetAll = 255; // 11111111 

unsigned long lastTimeVoltageReading, lastLEDBlinkDuration = millis();

byte ledState, blinkState = LOW;
byte blinkIndex = 0;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00011";

// struct Data_package {
//   char boom = 'u'; // boom can take the following: up, down, left (to ratete left), right (to ratate right)
//   char hook = 'd'; // hook take the following: up, down
//   char drive = 'f'; // drive take the following: forward, left, right, backward
// };
byte data = 0;

// Data_package data; // Create a variable with the above structure

void toggleSignalLED(byte state = LOW);

void setup() {
  // PIN CONFIG
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  // WHEEL PINS
  pinMode(STEERING_PIN1, OUTPUT);
  pinMode(STEERING_PIN2, OUTPUT);
  pinMode(WHEELS_PIN1, OUTPUT);
  pinMode(WHEELS_PIN2, OUTPUT);

  // LED PINS
  pinMode(LED_SIGNAL, OUTPUT);
  pinMode(LED_POWER, OUTPUT); 
  ledLoop(3, 100);
  controlDrivers(resetAll);
  steeringStop();
  driveStop();
  //Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // testing
  driveForward();
  delay(10000);
  driveBackward();
  delay(10000);
  driveStop();
  
  // reset all to 1
}

void loop() {
// check whether a data is receiver
// testing loop
  // while(true){
  //     driveForward();
  //     delay(10000);
  //     driveBackward();
  //     delay(10000);
  //     driveStop();
  // }

  while (radio.available()) {
    resetAllValues();
    radio.read(&data, sizeof(data)); // Read the whole data and store it into the 'data' structure
    ledLoop(2,100);

    // // check for hook control data
    int frontLeftDistance = frontLeftSonar.ping_cm();
    int frontRightDistance = frontRightSonar.ping_cm();
    int backDistance = backSonar.ping_cm();

    if(data == 7){
      controlDrivers(hookUp);
      toggleSignalLED(HIGH);
      ledLoop(5, 100);
    }
    else if(data == 8){
      controlDrivers(hookDown);
      toggleSignalLED(HIGH);
      ledLoop(6, 100); 
    }

    // check if it is drive control data
    else if (data == 1 && (frontRightDistance > OBSTACLE_MIN_DISTANCE || frontRightDistance == 0 )  && (frontLeftDistance > OBSTACLE_MIN_DISTANCE|| frontLeftDistance == 0)){
      driveForward();
      toggleSignalLED(HIGH);
      ledLoop(7, 100);
    }
    else if (data == 3 && backDistance > OBSTACLE_MIN_DISTANCE){
      driveBackward();
      toggleSignalLED(HIGH);
      ledLoop(8, 100);
    }
    else if (data == 4 && (frontRightDistance > OBSTACLE_MIN_DISTANCE || frontRightDistance == 0 )  && (frontLeftDistance > OBSTACLE_MIN_DISTANCE|| frontLeftDistance == 0)){
      driveLeft();
      toggleSignalLED(HIGH);
      ledLoop(9, 100);
    }
    else if (data == 2 && (frontRightDistance > OBSTACLE_MIN_DISTANCE || frontRightDistance == 0 )  && (frontLeftDistance > OBSTACLE_MIN_DISTANCE|| frontLeftDistance == 0)){
      driveRight();
      toggleSignalLED(HIGH);
      ledLoop(10, 100);
    }
    else if(data == 9){
      controlDrivers(boomUp);
      ledLoop(1, 100);
    }
    else if(data == 10){
      controlDrivers(boomDown);
      toggleSignalLED(HIGH);
      ledLoop(2, 100);
    }
    else if(data == 11){
      controlDrivers(rotateLeft);
      toggleSignalLED(HIGH);
      ledLoop(3, 100);
    }
    else if(data == 12){
      controlDrivers(rotateRight);
      toggleSignalLED(HIGH);
      ledLoop(4, 100);
    }else{
      controlDrivers(resetAll); // turn off all
      driveStop();
      toggleSignalLED(LOW);
      resetAllValues();
    }
  }

  resetAllValues();
  scanBatteryVoltage();
}

void resetAllValues(){
  data = 0;
}

void controlDrivers(byte numberToDisplay){
    digitalWrite(LATCH_PIN, LOW);
    // Shift out the bits
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, numberToDisplay);
    // ST_CP HIGH change LEDs
    digitalWrite(LATCH_PIN, HIGH);
}

void driveForward(){
  digitalWrite(WHEELS_PIN1, ACTIVE_LOW_ON);
  digitalWrite(WHEELS_PIN2, ACTIVE_LOW_OFF);
}

void driveBackward(){
  digitalWrite(WHEELS_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(WHEELS_PIN2, ACTIVE_LOW_ON);
}

void driveLeft(){
  digitalWrite(STEERING_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(STEERING_PIN2, ACTIVE_LOW_ON);
  driveForward();
}

void driveRight(){
  digitalWrite(STEERING_PIN1, ACTIVE_LOW_ON);
  digitalWrite(STEERING_PIN2, ACTIVE_LOW_OFF);
  driveForward();
}


void driveStop(){
  digitalWrite(WHEELS_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(WHEELS_PIN2, ACTIVE_LOW_OFF);
}

void steeringStop(){
  digitalWrite(STEERING_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(STEERING_PIN2, ACTIVE_LOW_OFF);
}

void toggleSignalLED(byte state=LOW){
  digitalWrite(LED_SIGNAL, state);
}

void togglePowerLED( byte state=LOW){
  digitalWrite(LED_POWER, state);
}

float getVoltage(){
  float adc_value = analogRead(VOLT_SENSOR_PIN);
  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;

  // Calculate voltage at divider input
  float voltage = adc_voltage*(R1+R2)/R2;
  return voltage;
}

void ledLoop(byte count, byte blinkDelay){
  // blink LED certain number of time
  do{
    if(millis() - lastLEDBlinkDuration > blinkDelay){
      // digitalWrite(LED_SIGNAL, blinkState);
      if(blinkState){
        digitalWrite(LED_SIGNAL, LOW);
      }else{
        digitalWrite(LED_SIGNAL, HIGH);
      }
      blinkState = !blinkState;
      lastLEDBlinkDuration = millis();
      blinkIndex++;
    };
  }while(blinkIndex <= count);
}

void scanBatteryVoltage(){
  if(millis() - lastTimeVoltageReading > 2000){
    float volt = getVoltage();
    if(volt < 11.3){
      // turn blink RED LED when less 10v
      if(ledState){
        togglePowerLED(LOW);
      }else{
        togglePowerLED(HIGH);
      }
      ledState = !ledState;
    }else {
      // turn LED 
      togglePowerLED(HIGH);
    }
    lastTimeVoltageReading = millis();
  }
}