
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
#define STEER_PIN1 16 // pin for moving the crane left and right directions
#define STEER_PIN2 17 // pin for moving the crane left and right directions
#define RIGHT_WHEEL_PIN1 14
#define RIGHT_WHEEL_PIN2 15

// hook pins
#define HOOK_PIN1 7
#define HOOK_PIN2 8

// BOOM PINS
#define BOOM_PIN1 5
#define BOOM_PIN2 6

// ROTATE PINS
#define ROTATE_PIN1 19
#define ROTATE_PIN2 4

// --- LED PINS
#define LED_SIGNAL 18 
// #define LED_POWER 19

// --- ULTRASONIC SENSOR (BACK) PINS ---
#define UL_BACK_TRIG_PIN 1 // BACK TRIGGER PIN
#define UL_BACK_ECHO_PIN 0 // BACK ECHO PIN

// --- ULTRASONIC SENSOR (FRONT RIGHT) PINS ---
#define UL_FR_TRIG_PIN 2 // FRONT-RIGHT TRIGGER PIN
#define UL_FR_ECHO_PIN 3 // FRONT-RIGHT ECHO PIN

#define MAX_DISTANCE 100
#define OBSTACLE_MIN_DISTANCE 20 // 20cm

NewPing frontRightSonar(UL_FR_TRIG_PIN, UL_FR_ECHO_PIN, MAX_DISTANCE);
// NewPing frontLeftSonar(UL_FL_TRIG_PIN, UL_FL_ECHO_PIN, MAX_DISTANCE);
NewPing backSonar(UL_BACK_TRIG_PIN, 
UL_BACK_ECHO_PIN, MAX_DISTANCE);

// --- NRF MODULE ---
#define NRF_MISO_PIN 12
#define NRF_MOSI_PIN 11
#define NRF_SCK_PIN 13
#define NRF_CSN_PIN 10
#define NRF_CE_PIN 9

// --- LED PINS
#define LED_SIGNAL 18
// #define LED_POWER 19

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

unsigned long lastTimeVoltageReading, lastLEDBlinkDuration = millis();

byte ledState, blinkState = LOW;
byte blinkIndex = 0;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00011";

byte data = 0;

void toggleSignalLED(byte state = LOW);

void setup() {

  // WHEEL PINS
  pinMode(STEER_PIN1, OUTPUT);
  pinMode(STEER_PIN2, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN1, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN2, OUTPUT);

  pinMode(BOOM_PIN1, OUTPUT);
  pinMode(BOOM_PIN2, OUTPUT);
  pinMode(HOOK_PIN1, OUTPUT);
  pinMode(HOOK_PIN2, OUTPUT);
  pinMode(ROTATE_PIN1, OUTPUT);
  pinMode(ROTATE_PIN2, OUTPUT);

  // LED PINS
  pinMode(LED_SIGNAL, OUTPUT);
  // pinMode(LED_POWER, OUTPUT); 
  ledLoop(3, 100);
  // controlDrivers(resetAll);
  steeringStop();
  driveStop();
  //Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

}

void loop() {
  while (radio.available()) {
    //resetAllValues();
    radio.read(&data, sizeof(data)); // Read the whole data and store it into the 'data' structure
    ledLoop(2,100);

    // // check for hook control data
    // int frontLeftDistance = frontLeftSonar.ping_cm();
    int frontRightDistance = frontRightSonar.ping_cm();
    int backDistance = backSonar.ping_cm();

    if(data == 7){
      // controlDrivers(hookUp);
      hookUp();
      toggleSignalLED(HIGH);
      ledLoop(5, 100);
    }
    else if(data == 8){
      // controlDrivers(hookDown);
      hookDown();
      toggleSignalLED(HIGH);
      ledLoop(6, 100); 
    }

    // check if it is drive control data
    else if (data == 1 && (frontRightDistance > OBSTACLE_MIN_DISTANCE || frontRightDistance == 0 )){
      driveForward();
      toggleSignalLED(HIGH);
      ledLoop(7, 100);
    }
    else if (data == 3 && ( backDistance == 0 || backDistance > OBSTACLE_MIN_DISTANCE)){
      driveBackward();
      toggleSignalLED(HIGH);
      ledLoop(8, 100);
    }
    else if (data == 4 && (frontRightDistance > OBSTACLE_MIN_DISTANCE || frontRightDistance == 0 )){
      steerLeft();
      toggleSignalLED(HIGH);
      ledLoop(9, 100);
    }
    else if (data == 2 && (frontRightDistance > OBSTACLE_MIN_DISTANCE || frontRightDistance == 0)){
      steerRight();
      toggleSignalLED(HIGH);
      ledLoop(10, 100);
    }
    else if(data == 9){
      // controlDrivers(boomUp);
      boomUp();
      ledLoop(1, 100);
    }
    else if(data == 10){
      // controlDrivers(boomDown);
      boomDown();
      toggleSignalLED(HIGH);
      ledLoop(2, 100);
    }
    else if(data == 11){
      // controlDrivers(rotateLeft);
      rotateLeft();
      toggleSignalLED(HIGH);
      ledLoop(3, 100);
    }
    else if(data == 12){
      // controlDrivers(rotateRight);
      rotateRight();
      toggleSignalLED(HIGH);
      ledLoop(4, 100);
    }else{
      // controlDrivers(resetAll); // turn off all
      rotateOff();
      boomOff();
      hookOff();
      driveStop();
      toggleSignalLED(LOW);
      //resetAllValues();
    }
  }

  // resetAllValues();
  scanBatteryVoltage();
}

void resetAllValues(){
  data = 255;
}

// functions to control hook
void hookUp(){
  digitalWrite(HOOK_PIN1, ACTIVE_LOW_ON);
  digitalWrite(HOOK_PIN2, ACTIVE_LOW_OFF);
}

void hookDown(){
  digitalWrite(HOOK_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(HOOK_PIN2, ACTIVE_LOW_ON);
}

void hookOff(){
  digitalWrite(HOOK_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(HOOK_PIN2, ACTIVE_LOW_OFF);
}

// functions to control boom
void boomUp(){
  digitalWrite(BOOM_PIN1, ACTIVE_LOW_ON);
  digitalWrite(BOOM_PIN2, ACTIVE_LOW_OFF);
}

void boomDown(){
  digitalWrite(BOOM_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(BOOM_PIN2, ACTIVE_LOW_ON);
}

void boomOff(){
  digitalWrite(BOOM_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(BOOM_PIN2, ACTIVE_LOW_OFF);
}


// functions to control rotation:
void rotateLeft(){
  digitalWrite(ROTATE_PIN1, ACTIVE_LOW_ON);
  digitalWrite(ROTATE_PIN2, ACTIVE_LOW_OFF);
}

void rotateRight(){
  digitalWrite(ROTATE_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(ROTATE_PIN2, ACTIVE_LOW_ON);
}

void rotateOff(){
  digitalWrite(ROTATE_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(ROTATE_PIN2, ACTIVE_LOW_OFF);
}



void driveForward(){
  digitalWrite(RIGHT_WHEEL_PIN1, ACTIVE_LOW_ON);
  digitalWrite(RIGHT_WHEEL_PIN2, ACTIVE_LOW_OFF);
}

void driveBackward(){
  digitalWrite(RIGHT_WHEEL_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(RIGHT_WHEEL_PIN2, ACTIVE_LOW_ON);
}

void steerLeft(){
  digitalWrite(STEER_PIN1, ACTIVE_LOW_ON);
  digitalWrite(STEER_PIN2, ACTIVE_LOW_OFF);
}

void steerRight(){
  digitalWrite(STEER_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(STEER_PIN2, ACTIVE_LOW_ON);
}


void driveStop(){
  digitalWrite(RIGHT_WHEEL_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(RIGHT_WHEEL_PIN2, ACTIVE_LOW_OFF);
  digitalWrite(STEER_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(STEER_PIN2, ACTIVE_LOW_OFF);
}

void steeringStop(){
  digitalWrite(STEER_PIN1, ACTIVE_LOW_OFF);
  digitalWrite(STEER_PIN2, ACTIVE_LOW_OFF);
}

void toggleSignalLED(byte state=LOW){
  digitalWrite(LED_SIGNAL, state);
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
       // togglePowerLED(LOW);
      }else{
       // togglePowerLED(HIGH);
      }
      ledState = !ledState;
    }else {
      // turn LED 
      // togglePowerLED(HIGH);
    }
    lastTimeVoltageReading = millis();
  }
}