/**
 * Author: Nitish Chennoju
 * Version 2b
 * Three RC Modes: FLY, STOP, HOLD. FLY mode enables full RC control.
 * STOP sets power 0 to motor and resets servo position. HOLD mode holds all current positions
 * 
 * UPDATE: Extended AIleron Movement Range + Channel 1, 2 Calibration (Works regardless on TX endpoints) +
 * Buzzer Warnings
 * 
 *
*/
#include<Wire.h>
#include <Servo.h>
#include <EnableInterrupt.h>


#define SERIAL_PORT_SPEED 115200
#define RC_NUM_CHANNELS  6

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3

#define RC_CH5_INPUT  4
#define RC_CH6_INPUT  5

//GYRO Variables
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t GyX, GyY, Tmp, AcX, AcY, AcZ;  //17000 max val
float posX;
//END

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];


//MY VARIABLES
Servo leftAileron;
Servo rightAileron;
Servo esc;

const int LAND_SERVO_OFFSET = 30;
int pos = 90;

const float PITCH_RATIO = 0.9;
const float ROLL_RATIO = 0.65;

float CH1_MIN = 1500;
float CH2_MIN = 1500;
float CH1_MAX = 1500;
float CH2_MAX = 1500;

float pitchAmount, rollAmount;
int posL, posR;

const int ESC_PIN = 9;
const int lA_PIN = 12;
const int rA_PIN = 11;
const int STATUS_LED_PIN = 13;
const int BUZZER_PIN = 13;
const int FLIGHT_LED = 7;

const int MAX_ROLL_AMOUNT = 50;


void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }
void calc_ch6() { calc_input(RC_CH6, RC_CH6_INPUT); }

void calib(int seconds) {
  Serial.println("STARTING...");
  rc_read_values();
  delay(1000);
  
  unsigned long tm = millis();
  
  while((unsigned long) seconds * 1000 > millis() - tm || CH1_MIN > 1400 || CH2_MIN > 1400 || CH1_MAX < 1600 || CH2_MAX < 1600){
    rc_read_values();
    if(rc_values[RC_CH1] < CH1_MIN && rc_values[RC_CH1] != 0){ CH1_MIN = rc_values[RC_CH1]; }
    if(rc_values[RC_CH2] < CH2_MIN && rc_values[RC_CH1] != 0){ CH2_MIN = rc_values[RC_CH2]; }
    if(rc_values[RC_CH1] > CH1_MAX){ CH1_MAX = rc_values[RC_CH1]; }
    if(rc_values[RC_CH2] > CH2_MAX){ CH2_MAX = rc_values[RC_CH2]; }

    if((millis()/50)%2 == 0){
      analogWrite(FLIGHT_LED, 255);
      //tone(BUZZER_PIN, 700, 300);
    }else{
      analogWrite(FLIGHT_LED, 0);
    }
    Serial.println("MODE: CALIBRATION\t" + String(millis()) + "\tMin: " + String(CH1_MIN) + "\t" + String(CH2_MIN) + "\tMAX: " + String(CH1_MAX) + "\t" + String(CH2_MAX));
  }
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(SERIAL_PORT_SPEED);
  
  leftAileron.attach(lA_PIN);
  rightAileron.attach(rA_PIN);
  esc.attach(ESC_PIN);

  esc.writeMicroseconds(1000);
  
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);

  //pinMode(STATUS_LED_PIN, OUTPUT);
  //pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLIGHT_LED, OUTPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);

  pitchAmount = 0;
  rollAmount = 0;

  calib(10);
}

void loop() {
  rc_read_values();

  Serial.print("MODE:\t");
  
  //When FLY mode is enabled
  if(rc_values[RC_CH5] > 1850){
    
    if(rc_values[RC_CH6] > 1400 && rc_values[RC_CH6] < 1600){
      //ROLL STABILIZATION
      Serial.print("ROLL STAB\t");

      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr,14,true);

      Tmp = Wire.read()<<8|Wire.read();
      GyX = (Wire.read()<<8|Wire.read());

      posX = -((GyX/17000.0) * 90.0);

      /*if(abs(posX) < 7){
        rollAmount = posX;
      }else if(posX <= -7){
        rollAmount = posX - 10;
      }else if(posX >= 7){
        rollAmount = posX + 10;
      }*/
      rollAmount = posX;
      
      pitchAmount = (float(rc_values[RC_CH2] - CH2_MIN) * (180/(CH2_MAX - CH2_MIN))) - 90.0;
        
      pitchAmount *= PITCH_RATIO;

      posR = (90 + pitchAmount) - rollAmount;
      posL = (90 - pitchAmount) - rollAmount;

      if(posL > 180) posL = 180;
      if(posL < 0) posL = 0;
      if(posR > 180) posR = 180;
      if(posR < 0) posR = 0;

      Serial.print("Left A: ");
      Serial.print(posL);
      Serial.print("\tRight A: ");
      Serial.print(posR);
      Serial.print("\tRollX : ");
      Serial.print(rollAmount);

      leftAileron.write(posL);
      rightAileron.write(posR);

      //THROTTLE
      Serial.print("\tMotor Power: ");
      if(rc_values[RC_CH3] < 1000){
        esc.writeMicroseconds(1000);
        Serial.print(1000);
      }else if(rc_values[RC_CH3] > 2000){
        esc.writeMicroseconds(2000);
        Serial.print(2000);
      }else{
        esc.writeMicroseconds(rc_values[RC_CH3]);
        Serial.print(rc_values[RC_CH3]);
      }

      Serial.println();

      
    }else if(rc_values[RC_CH6] > 1850) {
      //FLY (GYRO) EASY
      Serial.print("FLY (GYRO) EASY/t");

      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr,14,true);

      Tmp = Wire.read()<<8|Wire.read();
      GyX = (Wire.read()<<8|Wire.read());

      posX = ((GyX/17000.0) * 90.0);

      if(posX < -MAX_ROLL_AMOUNT){
        if(float(rc_values[RC_CH1] - CH1_MIN) > 600){
          rollAmount = (float(rc_values[RC_CH1] - CH1_MIN) * (180/(CH1_MAX - CH1_MIN))) - 90.0;
        }else{
          //rollAmount = - 3*(((GyX/17000.0) * 90.0) - 60.0);          
        }
      }else if(posX > MAX_ROLL_AMOUNT){
        if(float(rc_values[RC_CH1] - CH1_MIN) < 400){
          rollAmount = (float(rc_values[RC_CH1] - CH1_MIN) * (180/(CH1_MAX - CH1_MIN))) - 90.0;
        }else{
          //rollAmount = - 3*(((GyX/17000.0) * 90.0) + 60);          
        }
      }else{
        rollAmount = (float(rc_values[RC_CH1] - CH1_MIN) * (180/(CH1_MAX - CH1_MIN))) - 90.0;
      }

      pitchAmount = (float(rc_values[RC_CH2] - CH2_MIN) * (180/(CH2_MAX - CH2_MIN))) - 90.0;
        
      pitchAmount *= PITCH_RATIO;
      rollAmount *= ROLL_RATIO;
      
      if(float(rc_values[RC_CH1] - CH1_MIN) > 400 && float(rc_values[RC_CH1] - CH1_MIN) < 600){
        rollAmount = -((GyX/17000.0) * 90.0);
      }

      posR = (90 + pitchAmount) - rollAmount;
      posL = (90 - pitchAmount) - rollAmount;

      if(posL > 180) posL = 180;
      if(posL < 0) posL = 0;
      if(posR > 180) posR = 180;
      if(posR < 0) posR = 0;

      Serial.print("Left: ");
      Serial.print(posL);
      Serial.print("\tRight: ");
      Serial.print(posR);
      Serial.print("\tRoll: " + String(rollAmount));
      Serial.print("\tPitch: " + String(pitchAmount));
      Serial.print("\tPosX: " + String(posX));

      leftAileron.write(posL);
      rightAileron.write(posR);

    
      //THROTTLE
      Serial.print("\tMotor Power: ");
      if(rc_values[RC_CH3] < 1000){
        esc.writeMicroseconds(1000);
        Serial.print(1000);
      }else if(rc_values[RC_CH3] > 2000){
        esc.writeMicroseconds(2000);
        Serial.print(2000);
      }else{
        esc.writeMicroseconds(rc_values[RC_CH3]);
        Serial.print(rc_values[RC_CH3]);
      }
  
      Serial.println();
    
    }else{
        analogWrite(FLIGHT_LED, 255);
        Serial.print("FLY (GYRO) HARD\t");

        rollAmount = (float(rc_values[RC_CH1] - CH1_MIN) * (180/(CH1_MAX - CH1_MIN))) - 90.0;
        
        pitchAmount = (float(rc_values[RC_CH2] - CH2_MIN) * (180/(CH2_MAX - CH2_MIN))) - 90.0;
        
        pitchAmount *= PITCH_RATIO;
        rollAmount *= ROLL_RATIO;


        posR = (90 + pitchAmount) - rollAmount;
        posL = (90 - pitchAmount) - rollAmount;

        if(posL > 180) posL = 180;
        if(posL < 0) posL = 0;
        if(posR > 180) posR = 180;
        if(posR < 0) posR = 0;

        Serial.print("Left: ");
        Serial.print(posL);
        Serial.print("\tRight: ");
        Serial.print(posR);
        Serial.print("\tRoll: " + String(rollAmount));
        Serial.print("\tPitch: " + String(pitchAmount));

        leftAileron.write(posL);
        rightAileron.write(posR);

    
        //THROTTLE
        Serial.print("\tMotor Power: ");
        if(rc_values[RC_CH3] < 1000){
          esc.writeMicroseconds(1000);
          Serial.print(1000);
        }else if(rc_values[RC_CH3] > 2000){
          esc.writeMicroseconds(2000);
          Serial.print(2000);
        }else{
          esc.writeMicroseconds(rc_values[RC_CH3]);
          Serial.print(rc_values[RC_CH3]);
        }
  
        Serial.println();
      }
    }else{
      leftAileron.write(90);
      rightAileron.write(90);
      esc.writeMicroseconds(1000);
      Serial.println("STOP");
      
      //digitalWrite(STATUS_LED_PIN, HIGH);
      analogWrite(FLIGHT_LED, 255);
      delay(500);
      //digitalWrite(STATUS_LED_PIN, LOW);
      analogWrite(FLIGHT_LED, 0);
      delay(500);
    }
}
