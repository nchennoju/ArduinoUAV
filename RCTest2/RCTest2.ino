/**
 * Author: Nitish Chennoju
 * Version 2
 * Three RC Modes: FLY, STOP, HOLD. FLY mode enables full RC control. STOP sets power 0 to motor and resets servo position. HOLD mode holds all current positions
*/

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

#define RC_CH5_INPUT  /*A4*/4
#define RC_CH6_INPUT  /*A5*/5


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

//MY VARIABLES
const float servoC = 90.0/1500.0;

Servo leftAileron;
Servo rightAileron;
Servo esc;

const int LAND_SERVO_OFFSET = 30;
int pos = 90;


boolean once = true;
int lPos;
int rPos;
float Delay;

const int ESC_PIN = 9;
const int lA_PIN = 12;
const int rA_PIN = 11;
const int STATUS_LED_PIN = 13;

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

  pinMode(STATUS_LED_PIN, OUTPUT);

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
  if(rc_values[RC_CH5] > 1850){
    
    if(rc_values[RC_CH6] > 1400 && rc_values[RC_CH6] < 1600){
      //HOLD VALUES
      Serial.print("HOLD\t");

      Serial.print("Left A: ");
      Serial.print(lPos);
      Serial.print("\tRight A: ");
      Serial.println(rPos);
      
      once = true;
      
    }else if(rc_values[RC_CH6] > 1850) {
      //LANDING MODE
      Serial.println("LAND");

        if(once){
          once = false;
          Delay = 3000.0 / abs(LAND_SERVO_OFFSET - 120.0);      //3000 ms

          leftAileron.write(90-LAND_SERVO_OFFSET);
          rightAileron.write(90+LAND_SERVO_OFFSET);

          delay(1500);

          esc.writeMicroseconds(1000);
  
          for(int i = 0; i <= 2*LAND_SERVO_OFFSET; i++){
            leftAileron.write((90-LAND_SERVO_OFFSET)+i);
            rightAileron.write((90+LAND_SERVO_OFFSET)-i);
            delay(Delay);
          }
          
        }
      
      }else{
        Serial.print("FLY\t");
        
        //ROLL
        if((rc_values[RC_CH1] * servoC) >= 100){
          pos = 80;
        }else if((rc_values[RC_CH1] * servoC) <= 80){
          pos = 100;
        }else{
          pos = 180 - (rc_values[RC_CH1] * servoC);
        }
      
        //PITCH
        if((rc_values[RC_CH2] * servoC) >= 105){
          leftAileron.write(pos-15);
          rightAileron.write(pos+15);
          Serial.print("Left: ");
          Serial.print(pos + 15);
          Serial.print("\tRight: ");
          Serial.print(pos - 15);
        }else if((rc_values[RC_CH2] * servoC) <= 75){
          leftAileron.write(pos+15);
          rightAileron.write(pos-15);
          Serial.print("Left: ");
          Serial.print(pos - 15);
          Serial.print("\tRight: ");
          Serial.print(pos + 15);
        }else{
          leftAileron.write(pos - (rc_values[RC_CH2] * servoC - 90));
          rightAileron.write(pos + (rc_values[RC_CH2] * servoC - 90));
          Serial.print("Left: ");
          Serial.print(pos - (rc_values[RC_CH2] * servoC - 90));
          Serial.print("\tRight: ");
          Serial.print(pos + (rc_values[RC_CH2] * servoC - 90));
        }
  
        lPos = leftAileron.read();
        rPos = rightAileron.read();
        
    
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
  
        once = true;
        Serial.println();
      }
    }else{
      leftAileron.write(90);
      rightAileron.write(90);
      esc.writeMicroseconds(1000);
      Serial.println("STOP");
      
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(100);
      once = true;
    }

}


