/**
 * Author: Nitish Chennoju
 * Version 3
 * Three RC Modes: FLY, TAKEOFF/ALTHOLD, ____. FLY mode enables full RC control. STOP sets power 0 to motor and resets servo position.
*/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4


#include <Wire.h>
#include <Adafruit_BMP085.h>
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


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];


//MY VARIABLES
const int TAKEOFF_ALT = 1;
const float servoC = 90.0/1500.0;

const int MAX_THROTTLE = 2000;
const int RANGE = 0;

Adafruit_BMP085 bmp;
float iAltitude;
int initialAltitudeInt;
boolean once = true;
boolean reset = true;

//Components
const int ESC_PIN = 9;
const int lA_PIN = 12;
const int rA_PIN = 11;
const int STATUS_LED_PIN = 13;
const int REAR_LED_PIN = 8;

Servo leftAileron;
Servo rightAileron;
Servo esc;

int pos/* = 90*/;
/*int lPos;
int rPos;*/


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

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(REAR_LED_PIN, OUTPUT);

  if(!bmp.begin()){
    while(1){
      Serial.println("Could not find BMP sensor");
      
      for(int i = 0; i < 3; i++){
        digitalWrite(STATUS_LED_PIN, HIGH);
        digitalWrite(REAR_LED_PIN, LOW);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        digitalWrite(REAR_LED_PIN, LOW);
        delay(500);
      }
      delay(2000);
    }
  }


  iAltitude = bmp.readAltitude(101500);
  initialAltitudeInt = bmp.readAltitude(101500);
  
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

  
  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);
}

void loop() {
  rc_read_values();

  Serial.print("MODE:\t");
  
  //When FLY mode is enabled
  if(rc_values[RC_CH5] > 1500){
    
    if(rc_values[RC_CH6] > 1400 && rc_values[RC_CH6] < 1600){
      //HOLD (NEED TO CHANGE)
      //LAND MODE
      //GPS MODE
      Serial.print("\tThrottle: ");
      Serial.println(rc_values[RC_CH3]);
      esc.writeMicroseconds(rc_values[RC_CH3]);
      

      reset = true;
      
    }else if(rc_values[RC_CH6] > 1850) {
      //TAKEOFF MODE + ALT HOLD MODE
      int rollAmount;
      
        if(once){
          once = false;
          
          Serial.print("TAKEOFF");

          digitalWrite(STATUS_LED_PIN, HIGH);
          digitalWrite(REAR_LED_PIN, HIGH);
          delay(5000);

          //5 blinks indicates initialization sequence
          for(int i = 5; i >= 1; i--){
            digitalWrite(STATUS_LED_PIN, HIGH);
            digitalWrite(REAR_LED_PIN, HIGH);
            Serial.println(i);
            delay(500);
            digitalWrite(STATUS_LED_PIN, LOW);
            digitalWrite(REAR_LED_PIN, LOW);
            delay(500);
          }

          Serial.println("--- GO ----");
          
          initialAltitudeInt = bmp.readAltitude(101500);
          int alt = (int)(bmp.readAltitude(101500)-initialAltitudeInt);

          for(int i = 0; i <= 150; i++){
            Serial.print("Throttle Ramp = ");
            Serial.println(1000 + i);
            esc.writeMicroseconds(1000 + i);
            delay(10);
          }

          Serial.println("MOTOR TO SPEED");
          Serial.println("---------------");
          delay(500);
          
          while(alt < TAKEOFF_ALT){
            rc_read_values();
            
            Serial.print("ALT: ");
            Serial.println(alt);
            
            alt = (int)(bmp.readAltitude(101500)-initialAltitudeInt);

            //ROLL
            if((rc_values[RC_CH1] * servoC) >= 105){
              rollAmount = -15;
            }else if((rc_values[RC_CH1] * servoC) <= 75){
              rollAmount = 15;
            }else{
              rollAmount = 90 - (rc_values[RC_CH1] * servoC);
            }

            //check is it is + or - rollamount
            leftAileron.write((90 + ((TAKEOFF_ALT - alt) * 10)) + rollAmount);
            rightAileron.write((90 - ((TAKEOFF_ALT - alt) * 10)) + rollAmount);
            
            if(rc_values[RC_CH5] < 1500 || rc_values[RC_CH6] < 1850){
              break;
            }
          }
          
          
        }else{
          //ALT HOLD MODE

          digitalWrite(REAR_LED_PIN, HIGH);
          
          if(reset){
            iAltitude = bmp.readAltitude(101500);
            reset = false;
          }
          
          float alt = bmp.readAltitude(101500)-iAltitude;

          //ROLL
          if((rc_values[RC_CH1] * servoC) >= 105){
            rollAmount = -15;
          }else if((rc_values[RC_CH1] * servoC) <= 75){
            rollAmount = 15;
          }else{
            rollAmount = 90 - (rc_values[RC_CH1] * servoC);
          }


          if(abs(alt) <= 5){
            leftAileron.write((90 - (10*alt)) + rollAmount);
            rightAileron.write((90 + (10*alt)) + rollAmount);
            //esc.writeMicroseconds(1500 + (100 * alt));
          }else{
            if(alt > 0){
              leftAileron.write(40);
              rightAileron.write(140);
              //esc.writeMicroseconds(1000);
            }else{
              leftAileron.write(140);
              rightAileron.write(40);
              //esc.writeMicroseconds(2000);
            }
          }
          
          esc.writeMicroseconds(rc_values[RC_CH3]);
          analogWrite(REAR_LED_PIN, (rc_values[RC_CH3]-1000)/10 + 140);
          
          Serial.print("ALT HOLD\t");
    
          Serial.print("Alt: ");
          Serial.print(alt);
          Serial.print("\tTemperature: ");
          Serial.println(bmp.readTemperature());
          
        }
      
      }else{
        //FLY MODE
        digitalWrite(REAR_LED_PIN, HIGH);
        
        Serial.print("FLY\t");
        
        //ROLL
        if((rc_values[RC_CH1] * servoC) >= 125){
          pos = 55;
        }else if((rc_values[RC_CH1] * servoC) <= 55){
          pos = 125;
        }else{
          pos = 180 - (rc_values[RC_CH1] * servoC);
        }
      
        //PITCH
        if((rc_values[RC_CH2] * servoC) >= 125){
          leftAileron.write(pos-35);
          rightAileron.write(pos+35);
          Serial.print("Left: ");
          Serial.print(pos - 35);
          Serial.print("\tRight: ");
          Serial.print(pos + 35);
        }else if((rc_values[RC_CH2] * servoC) <= 55){
          leftAileron.write(pos+35);
          rightAileron.write(pos-35);
          Serial.print("Left: ");
          Serial.print(pos + 35);
          Serial.print("\tRight: ");
          Serial.print(pos - 35);
        }else{
          leftAileron.write(pos - (rc_values[RC_CH2] * servoC - 90));
          rightAileron.write(pos + (rc_values[RC_CH2] * servoC - 90));
          Serial.print("Left: ");
          Serial.print(pos - (rc_values[RC_CH2] * servoC - 90));
          Serial.print("\tRight: ");
          Serial.print(pos + (rc_values[RC_CH2] * servoC - 90));
        }
  
        /*lPos = leftAileron.read();
        rPos = rightAileron.read();*/
        
    
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
  
        reset = true;
        Serial.println();
      }

      
    }else{
      //STOP MODE (FLY MODE DISABALED)
      
      leftAileron.write(90);
      rightAileron.write(90);
      esc.writeMicroseconds(1000);
      Serial.println("STOP");

      initialAltitudeInt = bmp.readAltitude(101500);
      
      digitalWrite(STATUS_LED_PIN, HIGH);
      //digitalWrite(REAR_LED_PIN, HIGH);
      for(int i = 50; i <= 220; i++){
        analogWrite(REAR_LED_PIN, i);
        delay(5);
      }
      digitalWrite(STATUS_LED_PIN, LOW);
      //digitalWrite(REAR_LED_PIN, LOW);
      for(int i = 225; i >= 50; i--){
        analogWrite(REAR_LED_PIN, i);
        delay(5);
      }
      
      once = true;
      reset = true;
    }

}


