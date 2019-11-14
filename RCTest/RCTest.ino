#include <Servo.h>

#define THROTTLE_SIGNAL_IN 1 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt. CHANNEL IN PIN
#define THROTTLE_SIGNAL_IN_PIN 3 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead



#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car

volatile int nThrottleIn = NEUTRAL_THROTTLE;
volatile int nPitchIn = NEUTRAL_THROTTLE;
volatile unsigned long ulStartPeriod = 0; // set in the interrupt


volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
// we could use nThrottleIn = 0 in loop instead of a separate variable, but using bNewThrottleSignal to indicate we have a new signal 
// is clearer for this first example

int pos; //Throttle


void setup()
{
  // tell the Arduino we want the function calcInput to be called whenever INT0 (digital pin 2) changes from HIGH to LOW or LOW to HIGH
  // catching these changes will allow us to calculate how long the input pulse is
  attachInterrupt(digitalPinToInterrupt(THROTTLE_SIGNAL_IN_PIN), calcThrottle, CHANGE);
  //attachInterrupt(PITCH_SIGNAL_IN, calcPitch, CHANGE);

  
  
  Serial.begin(115200); 
}

void loop()
{
    Serial.print("typr:\t");
    Serial.println(nThrottleIn);
    pos = (nThrottleIn - 1000)/10 + 40;
 // if a new throttle signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
     if(bNewThrottleSignal)
     {
    
        Serial.print("typr:\t");
        Serial.println(nThrottleIn);
        pos = (nThrottleIn - 1000)/10 + 40;
    
    
    
    
        
       // set this back to false when we have finished
       // with nThrottleIn, while true, calcInput will not update
       // nThrottleIn
       bNewThrottleSignal = false;
     }

 // other processing ... 
}

void calcThrottle() {
  // if the pin is high, its the start of an interrupt
  if(digitalRead(THROTTLE_SIGNAL_IN_PIN) == HIGH)
  { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod && (bNewThrottleSignal == false))
    {
      nThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewThrottleSignal = true;
    }
  }
}

