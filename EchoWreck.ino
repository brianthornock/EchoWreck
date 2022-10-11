#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins of ATTiny841, make sure it runs at 8 MHz internal oscillator

//DigitalPin Assignments
const unsigned int encA = PIN_PA0;
const unsigned int encB = PIN_PA1;
const unsigned int pwm1 = PIN_PA4;
const unsigned int pwm2 = PIN_PA5;
const unsigned int pwm3 = PIN_PA3;
const unsigned int pwm4 = PIN_PA2;
const unsigned int tapSwitch = PIN_PB2;
const unsigned int divSwitch1 = PIN_PB0; //This will be an analog read
const unsigned int divSwitch2 = PIN_PB1; //This will be an analog read
const unsigned int ledOut = PIN_PA7;
const unsigned int testLED = PIN_PA6;


// Initiate encoder stuff
volatile int lastEnc = 0;

// Initiate the tap tempo button stuff
int8_t buttonState;
int8_t lastButtonState = LOW; // Previous state of tap tempo switch
int8_t tapStatus = LOW; // Current status of the tap tempo switch

unsigned long lastDebounceTime = 0; // initiate the debounce time counter
unsigned long debounceDelay = 30; // How long we enforce a debounce. Shouldn't be longer than minTime/2

int8_t prevTaps = 0; // How many taps we have in our history
unsigned int tapTimeout; // The amount of time between taps necessary to discard the tap buffer
int8_t maxTaps = 10; // Max number of taps to keep in history
int8_t prevTimes [10]; // Array for storing the tap periods
unsigned int tapTime; // Averaged value of times in tap history
unsigned int prevTapTime; // Previous averaged tap tempo time
int8_t useTap; // This is used to determine if we have enough taps to calculate the tap period
unsigned int prevTapDelay; // This is used for debouncing the tap tempo switch

//Set up bounds for how fast/slow we allow things to go before divisions
int8_t minTime = 46; // shortest allowable delay. Determined by where the PWM/delay time bottoms out
uint8_t maxTime = 171; // longest allowable delay for the first head
int8_t timeStep = 2; // The amount of time in ms between divisions over the range of [minTime:maxTime]
uint8_t encTimeStep = 8; //The amount of time in ms between encoder clicks so the user isn't spinning it forever

unsigned int delayTime; //delayTime is the time specified. This will be for head 4's delay time, so that we have the best resolution
unsigned int prevdelayTime; //set this to the default
unsigned int currentMillis; // Used for debouncing tap tempo switch
unsigned int prevMillis; // Used for keeping track of LED blinking

//Set up LED related items
int8_t currLEDState = LOW; // LED starts off off
unsigned int currLEDOffInterval; // How long the LED has been off
unsigned int currLEDOnInterval; // How long the LED has been on
int8_t updateLEDInterval = 1;

float multiplier = 2; // How much to divide the tapped tempo
int8_t updateMult = 1;
int8_t checkMultiplier = 0;
int8_t prevDivSwitch1 = LOW; // Keep track of previous division switch state so that we minimize delay through the loop
int8_t prevDivSwitch2 = LOW;
uint8_t currSW1 = LOW;
uint8_t currSW2 = LOW;

//Update PWM related items
uint8_t minPWM = 1; //Minimum PWM duty cycle
uint8_t minPWMDelta = 10; //Minimum separation between voltage levels
uint8_t updateDutyCycle = 0;
uint8_t prevDutyCycle;

// Table to convert delay times into 10-bit PWM duty cycles. 342 entries supports delay times up to 684ms in 2 ms steps.

// Stick it in PROGMEM so it lives in flash and doesn't use any SRAM 

// Commented out first table is for 1k current limiting pin 6 resistor
/*
const unsigned int delay_conv[] PROGMEM = {975,975,975,975,975,975,975,975,975,975,975,975,975,975,975,975,975,975,975,975,965,935,835,765,703,650,610,575,545,517,495,473,451,430,410,390,372,357,344,334,323,313,
304,296,286,278,270,261,252,243,237,232,228,223,217,212,207,202,198,194,190,185,181,177,173,168,165,159,156,154,151,148,145,143,141,139,137,135,133,131,129,126,124,121,119,118,117,115,114,112,111,110,108,107,
105,104,103,102,101,99,98,97,96,95,94,93,92,91,91,90,89,88,87,86,85,84,83,82,81,80,80,79,78,77,77,76,76,75,74,74,73,72,72,71,71,70,70,69,69,68,68,67,67,66,66,65,65,64,64,63,63,63,62,62,61,61,61,60,60,59,59,59,
58,58,58,57,57,57,56,56,56,55,55,55,54,54,54,53,53,53,52,52,52,51,51,51,51,50,50,50,50,49,49,49,49,48,48,48,48,47,47,47,47,46,46,46,46,45,45,45,45,44,44,44,44,43,43,43,42,42,42,42,41,41,41,41,40,40,40,40,40,40,
39,39,39,39,39,38,38,38,38,38,37,37,37,37,37,37,37,36,36,36,36,36,36,36,35,35,35,35,35,35,34,34,34,34,34,34,34,34,33,33,33,33,33,33,33,33,32,32,32,32,32,32,32,32,31,31,31,31,31,31,31,31,31,30,30,30,30,30,30,30,
30,30,29,29,29,29,29,29,29,29,29,28,28,28,28,28,28,28,28,28,28,27,27};
*/

// Second table is for 1.5k current limiting pin 6 resistor
const unsigned int delay_conv[] PROGMEM = {900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,900,825,760,710,666,626,595,570,543,523,506,488,468,451,433,415,397,384,374,365,
354,343,333,323,314,305,296,289,282,275,268,261,254,249,254,239,234,229,224,219,214,210,207,203,199,195,192,189,186,183,180,177,174,171,168,165,162,159,157,155,153,151,149,147,145,143,141,139,137,135,133,132,130,
129,127,125,124,123,121,120,119,117,116,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,98,97,97,96,95,94,93,93,92,91,90,90,89,88,87,87,86,85,84,83,83,82,81,81,80,80,79,79,78,78,78,77,77,76,76,75,75,74,
74,73,73,72,72,71,71,70,70,69,69,68,68,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,61,61,61,60,60,60,60,59,59,59,58,58,58,57,57,57,56,56,56,55,55,55,55,54,54,54,54,53,53,53,53,52,52,52,52,51,51,51,51,50,50,50,50,
49,49,49,49,49,48,48,48,48,47,47,47,47,47,46,46,46,46,46,45,45,45,45,45,44,44,44,44,44,43,43,43,43,43,43,42,42,42,42,42,42,41,41,41,41,41,41,40,40,40,40,40,40,40,39,39,39,39,39,39,38,38,38,38,38,38,38,38,37,37,37,37,
37,37,37,36,36,36,36,36,36,36,36,35,35,35,35,35,35,35,35,34,34,34,34,34,34,33,33,33,33,33,33,33,33,33,32,32,32,32,32,32};


void setup() {
  //Define what each pin is
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(divSwitch1, INPUT);
  pinMode(divSwitch2, INPUT);
  pinMode(tapSwitch, INPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(ledOut, OUTPUT);
  pinMode(testLED, OUTPUT);

  //Set up the initial state of the pins
  digitalWrite(encA, HIGH);
  digitalWrite(encB, HIGH);
  digitalWrite(divSwitch1, LOW);
  digitalWrite(divSwitch2, LOW);
  digitalWrite(tapSwitch, LOW);
  digitalWrite(ledOut, LOW);
  digitalWrite(testLED, LOW);

  uint8_t div1 = digitalRead(divSwitch1);
  uint8_t div2 = digitalRead(divSwitch2);

  if (div1 == HIGH) {
    multiplier = 2;
    prevDivSwitch1 = HIGH;
    currSW1 = HIGH;
    checkMultiplier = 1;
  }
  else if (div2 == HIGH) {
    multiplier = 4;
    prevDivSwitch2 = HIGH;
    currSW2 = HIGH;
    checkMultiplier = 1;
  }

  delayTime = 300; //set this to the default
  prevdelayTime = delayTime; //set this to the default
  tapTime = delayTime; //Initialize with a default value
  prevTapTime = delayTime; //Initialize with default value
  prevTapDelay = 0;

  currLEDOnInterval = round(delayTime / 2);
  currLEDOffInterval = round(delayTime - currLEDOnInterval);

  prevMillis = millis();

  tapTimeout = 12 * maxTime; // How long to keep waiting for taps. Give 3 lengths of the longest delay time in the set

  //Set up the timer outputs to the correct pins. Don't touch timer 0 so that millis() works correctly
  //Pin 8 -> PWM2 -> TOCC4 (make timer OC2B)
  //Pin 9 -> PWM1 -> TOCC3 (make timer OC2A)
  //Pin 10 -> PWM3 -> TOCC2 (make timer OC1B)
  //Pin 11 -> PWM4 -> TOCC1 (make timer OC1A)
  TOCPMSA0 = (1<<TOCC1S0) | (1<<TOCC2S0) | (1<<TOCC3S1); //This sets TOCC1 throught TOCC3
  TOCPMSA1 = (1<<TOCC4S1); //This sets TOCC4 to be OC2B
  TOCPMCOE = (1<<TOCC1OE) | (1<<TOCC2OE) | (1<<TOCC3OE) | (1<<TOCC4OE); //Enable the time/output compare on TOCC1-4
  
  //Disable interrupts on timers
  TIMSK1 = 0;
  TIMSK2 = 0;
  
  //Set up 16 bit timers so that we can use PWM
  //PWM is 10-bit fast, 0x03FF TOP, no prescaler
  //Set to Fast, 10-bit PWM, max value of 1024
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (1<<WGM10); //TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = (1<<WGM12) | (1<<CS10); //No prescaler,  | _BV(WGM12)
  TCCR1C = 0b00000000; //Set to zero when in PWM mode
  TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); //Set to Fast, 10-bit PWM, max value of 1024
  TCCR2B = (1<<WGM22) | (1<<CS20); //No prescaler,  | _BV(WGM22)
  TCCR2C = 0b00000000; //Set to zero when in PWM mode

  // Initialize the PWM outputs
 
  int delayTime1 = timeStep * round(delayTime/(4*timeStep)/2);
  int delayTime2 = 2 * delayTime1;
  int delayTime3 = 3 * delayTime1;
  int delayTime4 = 4 * delayTime1;
  
  //Get PWM duty cycle for desired delay time. Divide by two to get the right index since delay_conv is in 2 ms increments
  int dutyCycle1 = pgm_read_word_near(delay_conv + delayTime1); 
  int dutyCycle2 = pgm_read_word_near(delay_conv + delayTime2);
  int dutyCycle3 = pgm_read_word_near(delay_conv + delayTime3); 
  int dutyCycle4 = pgm_read_word_near(delay_conv + delayTime4);

  
  writeDutyCycle(3,dutyCycle1); //PWM1 is on TOCC3
  writeDutyCycle(4,dutyCycle2); //PWM2 is on TOCC4
  writeDutyCycle(2,dutyCycle3); //PWM3 is on TOCC2
  writeDutyCycle(1,dutyCycle4); //PWM4 is on TOCC1
  
  //Set up an ISR for the encoder switch pin so that it reacts instantly
  //and to reduce loop execution time with determining multiplier
  GIMSK = 0b00110000; //enable pin change interrupts for all pins
  PCMSK0 = 0b00000011; //enable PCINT0/Pin13 and PCINT1/Pin12 as pin change interruptible for encoder pins
  PCMSK1 = 0b00000011; //enable PCINT8/Pin3 and PCINT9/Pin2 as pin change interruptible for division switch
  sei(); //start interrupt service
}



void loop() {
  
  if (checkMultiplier) {
    //Update the multiplier by polling the switch
    updateMultiplier();
  }
  
  //Check to see if tap tempo is used
  checkTapTempo();

  //Update the sample time based on above
  updatedelayTime();

  currentMillis = millis();

  //Update the LED blink rate
  updateLED();

  //Update the PWM generation
  updatePWM();
  
}


//Interrupt handling for PCINT0/PCMSK0
ISR (PCINT1_vect) {

  //If things changed due to division switch
  currSW1 = digitalRead(divSwitch1);
  currSW2 = digitalRead(divSwitch2);
  
  if ((currSW1 != prevDivSwitch1) | (currSW2 != prevDivSwitch2)) {
    checkMultiplier = 1;
    //testBlink();
  }
  
}




//Interrupt handling for PCINT1/PCMSK1
ISR (PCINT0_vect) {
  
  int8_t  readA1 = digitalRead(encA);
  int8_t  readB1 = digitalRead(encB);

  delayMicroseconds(2000); //Brute force debouncing
  int8_t  readB2 = digitalRead(encB);
  int8_t  readA2 = digitalRead(encA);

  if (readA1 == readA2 && readB1 == readB2) {

    int8_t MSB = readA1;
    int8_t LSB = readB1;

    int encoded = (MSB << 1) | LSB; // shift MSB by one bit and append LSB
    int sum = (lastEnc << 2) | encoded; // shift last ENC value by two bits and append the latest two

    //Clockwise turn
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
      tapTime = tapTime + encTimeStep;
      //testBlink();
    }
    //Counterclockwise turn
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
      tapTime = tapTime - encTimeStep;
      //testBlink();
    }
    lastEnc = encoded;
  }

}



void updateMultiplier() {
  // Poll the switch, but let the encoders use the interrupts

  float prevMult = multiplier;

  // If the first read is different from the previous read, execute
  if ((currSW1 != prevDivSwitch1) | (currSW2 != prevDivSwitch2)){
    // Brute force debounce the switch
    delay(100);
    
    int currSW1_2 = digitalRead(divSwitch1);
    int currSW2_2 = digitalRead(divSwitch2);

    if ((currSW1 == currSW1_2) | (currSW2 == currSW2_2)){
      if (currSW1 == HIGH) {
        multiplier = 1.5;
      }
      else if (currSW2 == HIGH) {
        multiplier = 2.5;
      }
      else {
        multiplier = 2;
      }
      prevDivSwitch1 = currSW1;
      prevDivSwitch2 = currSW2;
    }
  }

  if (multiplier != prevMult) {
    updateMult = 1;
    testBlink();
  }
  else {
    updateMult = 0;
  }
  checkMultiplier = 0;
}


void updatedelayTime() {

  //If we are more than timeStep ms off, update the delayTime and apply the  multiplier according to the switch
  if ((abs(tapTime - prevTapTime) >= timeStep)) {

    int tempDelayTime = round(tapTime);
    prevTapTime = tapTime;
    //testBlink();

    prevMillis = millis(); //Set new start time for LED blinking to now
    updateLEDInterval = 1; //Tell LED to update
    updateDutyCycle = 1; //Tell duty cycle to update

    // Don't let delay times get shorter or longer than our limits. Note the factor of 4, as that is our tap time reference and we have 4 heads.
    if (tempDelayTime < 4* minTime) {
      tempDelayTime = 4 * minTime;
    }
    else if (tempDelayTime > 4 * maxTime) {
      tempDelayTime = 4 * maxTime;
    }

    delayTime = round(tempDelayTime);
  }
}



//Code for debouncing tap tempo switch
void switchDebounce() {
  int8_t reading = digitalRead(tapSwitch);
  int8_t reading2 = 2; // Set to a value that can't be reached with digitalRead on purpose

  if (reading != lastButtonState) {
    delay(debounceDelay);
    reading2 = digitalRead(tapSwitch);
    
  }
  
  if (reading == reading2) {
    buttonState = reading;

    if (buttonState == HIGH) {
        tapStatus = HIGH;
    }
  }
  
  lastButtonState = reading;
}


void checkTapTempo() {

  //Check to see if the tap tempo switch has been pressed
  switchDebounce();

  if (tapStatus == HIGH) {
    tapStatus = LOW;
    //Check to see if we already have a tap tempo history. If so, add this to
    //the history. If not, start a new count.
    if (prevTaps > 0) {

      int currTime = millis();
      int currDelay = currTime - prevTapDelay;
      // Check to make sure we didn't time out
      if (currDelay < tapTimeout) {
        //Set the flag for using tap tempo
        useTap = 1;

        // Create the temp array for storing times in
        unsigned int newPrevTimes [maxTaps];

        if (prevTaps < maxTaps) {

          //Fill up the new array with all the old values first
          for (int k = 0; k < prevTaps - 1; k++) {
            newPrevTimes[k] = prevTimes[k];
          }

          //Then add in the new value at the end
          newPrevTimes[prevTaps - 1] = currDelay;
          prevTaps++;

        } // End if prevTaps < maxTaps
        // If we have filled the tap buffer

        for (int nTime = 0; nTime < maxTaps; nTime++) {
          prevTimes[nTime] = newPrevTimes[nTime];
        }

      } // End if currDelay < tapTimeout
      else {
        //If we timeout, reset the counter and zero out the tempo array
        prevTaps = 1;

        for (int i = 0; i < maxTaps; i++) {
          prevTimes[i] = 0;
        }

        useTap = 0;
      } // End if tap has timed out
    } // End if prevTaps > 0
    // If we do not have any previous taps (first tap after timeout)
    else {
      prevTaps = 1;

      for (uint8_t i = 0; i < maxTaps; i++) {
        prevTimes[i] = 0;
      }

      useTap = 0;
    }

    if (useTap == 1 && prevTaps > 2) {
      
      //Calculate the average polling time, including the multiplier and the random switch
      int sum, loop, numVals;
      float avg;

      sum = avg = 0;
      numVals = 0;

      for (loop = 0; loop < prevTaps - 1; loop++) {
        if (prevTimes[loop] != 0) {
          sum += prevTimes[loop];
          numVals++;
        }
      }
      avg = (float)sum / numVals;
      tapTime = round(avg);

      // Allow for longer tap tempo intervals, then divide down by 2 until it gets to a manageable value
      while (tapTime > 4 * maxTime) {
        tapTime = timeStep * round(tapTime/(2 * timeStep));
        //tapTime = round(tapTime/(2));
      }
      
    }
    else {
      //If we don't have the information to produce a tap tempo, stick with what we have
    }
    prevTapDelay = millis();
  }

}


//Code for LED flashing update
void updateLED() {


  if (updateLEDInterval) {
    updateLEDInterval = 0;
    currLEDOnInterval = round(delayTime / 2);
    currLEDOffInterval = round(delayTime - currLEDOnInterval);
  }
 
  //Check to see if we have completed the LED on or off interval and change if we have
  if (currLEDState == LOW) {
    if (currentMillis - prevMillis >= currLEDOffInterval) {
      currLEDState = HIGH;
      prevMillis += currLEDOffInterval;
      digitalWrite(ledOut, HIGH);
    }
  }

  if (currLEDState == HIGH) {
    if (currentMillis - prevMillis >= currLEDOnInterval) {
      currLEDState = LOW;
      prevMillis += currLEDOnInterval;
      digitalWrite(ledOut, LOW);
    }
  }
}




void updatePWM() {
  if (updateDutyCycle) {

    uint8_t shortTime = timeStep*round(delayTime/(4*timeStep)); // Get the shortest time to be a multiple of our time step
    
    //Convert the current delay time to the appropriate PWM using the mapping table. Divide by 2 to index into delay_conv later
    int delayTime1 = round(shortTime/2); //This is the first head's time, 1/4 tapped time
    int delayTime2 = round(multiplier * shortTime/2); //This is the second heads's
    int delayTime3 = round(3 * shortTime/2); //Third head's time, get the picture?
    int delayTime4 = round(2 * multiplier * shortTime/2); //You guessed it, fourth heads' time
    
    //Get duty cycle for 10-bit PWM. Divide by 2 to get the nearest 2 ms delay time from the delay_conv table.
    int dutyCycle1 = pgm_read_word_near(delay_conv + delayTime1); 
    int dutyCycle2 = pgm_read_word_near(delay_conv + delayTime2);
    int dutyCycle3 = pgm_read_word_near(delay_conv + delayTime3);
    int dutyCycle4 = pgm_read_word_near(delay_conv + delayTime4);
    
    //Update the PWM pins' duty cycle accordingly
    writeDutyCycle(3,dutyCycle1); //PWM1 is on TOCC3
    writeDutyCycle(4,dutyCycle2); //PWM2 is on TOCC4
    writeDutyCycle(2,dutyCycle3); //PWM3 is on TOCC2
    writeDutyCycle(1,dutyCycle4); //PWM4 is on TOCC1
  }
}




void writeDutyCycle(uint8_t tocc, uint16_t duty) {

  // Set duty cycle based on the TOCC pin being specified
  if(tocc == 1) {
    OCR1A = duty;
    TOCPMCOE |= 1<<TOCC1OE;
  } 
  else if (tocc == 2) {
    OCR1B = duty;
    TOCPMCOE |= 1<<TOCC2OE;
  }
  else if (tocc == 3) {
    OCR2A = duty;
    TOCPMCOE |= 1<<TOCC3OE;
  }
  else if (tocc == 4) {
    OCR2B = duty;
    TOCPMCOE |= 1<<TOCC4OE;
  }
  
  
}


void testBlink() {
  digitalWrite(testLED,HIGH);
  delay(100);
  digitalWrite(testLED,LOW);
}
