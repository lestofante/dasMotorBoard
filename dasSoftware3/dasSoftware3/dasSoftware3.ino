/*
  attiny85 specific code, 8mhz only
  
  interrupts:
  input into INT0
  interrupt from that pin, expecting pulse of 1000-2000 us
  when signal is high 8us timer start
  timer in 8us ticks, overflows at 255, 255*8 = 2040. 
  if that timer rolls over return a 0
  pulse goes low time is stored/acted on, timer runs in 1ms mode 
  1ms compare to 30, if expries return 0

  pinout:
  Pin7 (PB2)  Radio IN
  Pin2 (PB3)  Q1 (fwd LED)
  Pin3 (PB4)  Q2 (rev LED)
  Pin5 (PB0)  Q3 (PWM REV)
  Pin6 (PB1)  Q4 (PWM FWD and BRK)
  Pin1 (PB5)  brake LED/user input
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

//eeprom version
const uint8_t no_secret = 5;

//ms are in ms
const uint8_t ms_loopTime = 15; //duty cycle falling edge to falling edge
const uint8_t ms_timeOut = 30; //timeout between pulses
const uint16_t ms_doubleTap = 1000; //time to rev in ms
const uint16_t ms_revTime = 2000; //time reverse is active
const uint16_t ms_trigTime = 450; //time to double tap to start rev timer

//counter constants
const uint8_t no_timeOuts = 10; //number of timeouts to safe the controller
const uint8_t no_taps = 2; //how many taps to get into rev
const uint8_t no_maxOffset = 240; //assert full throttle (255) after this num
const uint8_t no_pulseAvg = 50;

//us times in uS are us/8
const uint8_t us_weenieRev = 165; //partial throttle for reverse cause I'm a weenie
const uint8_t us_neutOffset = 3; //dead zone for neutral

//these settings are eeprom reset defaults
//or are loaded from calibration memory
volatile uint8_t us_maxFwd = 250; //2000
volatile uint8_t us_neutral = 187; //1500
volatile uint8_t us_maxRev = 125; //1000
volatile uint8_t us_radOff = 187; //1493

volatile uint8_t us_minFwd = us_neutral + us_neutOffset;
volatile uint8_t us_minRev = us_neutral - us_neutOffset;

volatile bool flag_rev = false;
volatile bool flag_calib = false;

volatile int ms_revTimer = 0; //needs to go negative
volatile uint8_t us_meanTime = 0;

volatile uint8_t no_pulseLeft = 0;
volatile uint8_t no_revTaps = 0;
volatile uint8_t no_timeOutCount = 0;

//interrupt vars
volatile uint16_t ms_timeOuts = 0;
volatile unsigned int no_overFlows = 0;  

//pins
const int pin_Q1 = 3;
const int pin_Q2 = 4;
const int pin_Q3 = 0;
const int pin_Q4 = 1;

const int pin_radio = 2;
const int pin_brakeLED = 5;


//=================Setup=================
void setup(){
  //pins
  pinMode(pin_Q1,OUTPUT);
  pinMode(pin_Q2,OUTPUT);
  pinMode(pin_Q3,OUTPUT);
  pinMode(pin_Q4,OUTPUT);
  
  pinMode(pin_brakeLED, INPUT);
  pinMode(pin_radio, INPUT);

  //calib mode
  if(digitalRead(pin_brakeLED)){
    calibMode();
  }
  
  //reset pin, continue
  pinMode(pin_brakeLED, OUTPUT);

  //check and load eeprom
  varInit();

  //flash LED
  flashies(2);
  
  //timers
  initPWM();
  initINT();
}
//=================LOOP=================
void loop(){
 //https://youtu.be/3gWTTQWrB3I?t=16
}

//=================Time/interrupt Setup=================
void initPWM(){
  //might want to set wgm as well.
  TCCR0B |= _BV(CS00);
}

void initINT(){
  //int0 is hard wired to pin 7
  MCUCR |= _BV(ISC00); //isc00 for any pin change
  GIMSK = _BV(INT0); //enable interrupt
  TIMSK &= ~_BV(OCIE1A); //stop compare A interrupt
  TIMSK |= _BV(TOIE1);   //start overflow interrupt
}

//=================Timer0 modes=================
void initFST(){
  TCCR1 = 0; //stop the timer
  TCNT1 = 0; //zero timer count
  TCCR1 = _BV(CS12) | _BV(CS11) | _BV(CS10); //count goes up 8us/tick
}

void initSLO(){
  TCCR1 = 0; //stop the timer
  ms_timeOuts = 0; //reset timeout counter
  evalPulse(TCNT1); //shove pulse width somewhere
  TCNT1 = 0; //zero timer count
  TCCR1 = _BV(CS12) | _BV(CS11); //overflow every 1.02ms
}

//=================ISRs=================
ISR(INT0_vect){
//  PINB |= _BV(PINB4);
  if(digitalRead(pin_radio)){
    initFST();
  }else{
    initSLO();
  }
}

//triggers > 2024us when in fast mode
//triggers every 1.024ms when in slow mode
ISR(TIMER1_OVF_vect){
  if(digitalRead(pin_radio)){
    evalPulse(0);
  }else if(ms_timeOuts < ms_timeOut){
    ms_timeOuts++;
  }else if(ms_timeOuts >= ms_timeOut){
    ms_timeOuts = 0;
    evalPulse(0);
  }
}

//compA is just keeping if statements out of my ISRs
//if accuracy was important all 3 could use this register
ISR(TIMER1_COMPA_vect){
  ms_timeOuts++;
}

//=================PWM SETTER=================
void evalPulse(uint8_t us_pulseTime){
  if(flag_calib){
    evalCalib(us_pulseTime);
  }else{
    evalSpeed(us_pulseTime);
  }
}

void evalCalib(uint8_t us_pulseTime){
  digitalWrite(pin_brakeLED, HIGH);
  //if it's 0 it gets evaluated, tough titties
  if(us_meanTime == 0){ //store pulse1
    us_meanTime = us_pulseTime; 
    no_pulseLeft--;
  }else{
    us_meanTime = (0.9*us_meanTime) + (0.1*us_pulseTime);
    no_pulseLeft--;
  }
  digitalWrite(pin_brakeLED, LOW);
}

void evalSpeed(uint8_t us_pulseTime){
  //neutral if read bad or transmitter off
  if(us_pulseTime == 0 || ( us_pulseTime <= us_radOff + 1 && us_pulseTime >= us_radOff - 1)){
    
    if(no_timeOutCount == no_timeOuts){
      neutral();
    }else{no_timeOutCount++;}
    
  }else{
    no_timeOutCount = 0;
  }

  //pulse is actually neutral
  if((us_pulseTime <= us_minFwd) && (us_pulseTime >= us_minRev)){
    if(ms_revTimer == 0){no_revTaps = 0;}

    //last state was reverse
    if(flag_rev){
      no_revTaps++;
      if(no_revTaps == no_taps+1){
        //no_revTaps = 0; 
        //ms_revTimer = 0; 
      }else if(no_revTaps > 1){
        ms_revTimer = ms_revTime;
      }else if(no_revTaps == 1){
        ms_revTimer = ms_trigTime;
      }
    }
    
    flag_rev = false;
    neutral(); 

  //pulse is reverse
  }else if((us_pulseTime <= us_minRev)){
    if(no_revTaps >= no_taps){
      reverse(us_pulseTime);
    }else if(no_revTaps < no_taps){
      brake(us_pulseTime);
    }
    
    flag_rev = true;

  //pulse is forward
  }else if(us_pulseTime >= us_minFwd){
    flag_rev = false;
    ms_revTimer = 0;
    no_revTaps = 0;
    forward(us_pulseTime);
  }
  
  //deduct timer each down tick
  ms_revTimer = constrain(ms_revTimer - ms_loopTime, 0, 0x7FFF);
}


//=================eeprom stuff=================
//load values from progmem
void varInit(){
  uint8_t no_promV = 0;
  EEPROM.get(8, no_promV);

  //nums match, load the saved settings
  //this will default to the header settings
  if(no_promV == no_secret){
    EEPROM.get(0, us_maxFwd);
    EEPROM.get(2, us_neutral);
    EEPROM.get(4, us_maxRev);
    EEPROM.get(6, us_radOff);
  }
  
  us_minFwd = us_neutral + us_neutOffset;
  us_minRev = us_neutral - us_neutOffset;
}

void writeCalib(){
  //just write all of them at once
  EEPROM.put(0, us_maxFwd);
  EEPROM.put(2, us_neutral);
  EEPROM.put(4, us_maxRev);
  EEPROM.put(6, us_radOff); 

  EEPROM.put(8, no_secret);

  //red says wrote
  flashies(2);
}

//=================Display Functions=================
//alternate delay for LED flashing
//this is not to be used in the main code
void delay1(uint16_t ms_delayTime){
  TCCR1 = 0; //stop the timer
  TCNT1 = 0; //zero timer count
  ms_timeOuts = 0; //reset timeout counter
  OCR1C = 250; // (250) 1a goes off before overflow 1.000ms!
  OCR1A = 250;
  TIMSK &= ~_BV(TOIE1); //stop the overflow interrupt
  TIMSK |= _BV(OCIE1A); //set compare A interrupt
  TCCR1 = _BV(CTC1) | _BV(CS12) | _BV(CS11); //reset tcnt on comp and div by 32
  
  while(ms_timeOuts < ms_delayTime){
    //https://youtu.be/3gWTTQWrB3I?t=16
  }
  
  TCCR1 = 0; //stop the timer
}

//flashes brake LED, each flash is 400ms
void flashies(uint8_t i){
  while(i){
    digitalWrite(pin_brakeLED, HIGH);
    delay1(200);
    digitalWrite(pin_brakeLED, LOW);
    delay1(200);
    i--;
  }
}

//=================speed functions=================
void neutral(){
//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 0);
//  digitalWrite(pin_Q3, 0);
//  digitalWrite(pin_Q4, 1);

  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 0);
  digitalWrite(pin_Q4, 0);

  digitalWrite(pin_brakeLED, LOW);
}

void forward(uint8_t rate){
//  digitalWrite(pin_Q1, 1);
//  digitalWrite(pin_Q2, 0);
//  digitalWrite(pin_Q3, 0);
//  digitalWrite(pin_Q4, 0);

  uint8_t mappedSpeed = 0;
  
  mappedSpeed = constrain(rate, us_minFwd , us_maxFwd);
  mappedSpeed = map(mappedSpeed, us_minFwd, us_maxFwd, 0, 255);

  //full fwd
  if(mappedSpeed >= no_maxOffset){
    mappedSpeed = 255;
  }

  digitalWrite(pin_Q1, 1);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 0);
  analogWrite(pin_Q4, mappedSpeed);

  digitalWrite(pin_brakeLED, LOW);
}

void brake(uint8_t rate){
//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 1);
//  digitalWrite(pin_Q3, 0);
//  digitalWrite(pin_Q4, 0);

  uint8_t mappedSpeed = 0;
  
  mappedSpeed = constrain(rate, us_maxRev, us_minRev);
  mappedSpeed = 255 - map(mappedSpeed, us_maxRev, us_minRev, 0, 255);

  //full fwd
  if(mappedSpeed >= no_maxOffset){
    mappedSpeed = 255;
  }

  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 1);
  analogWrite(pin_Q4, mappedSpeed);

  digitalWrite(pin_brakeLED, HIGH);
}

void reverse(uint8_t rate){
//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 0);
//  digitalWrite(pin_Q3, 1);
//  digitalWrite(pin_Q4, 0);

  uint8_t mappedSpeed = 0;
    
  //weenieRev goes here
  //mappedSpeed = constrain(rate, us_maxRev , us_minRev);
  mappedSpeed = constrain(rate, us_weenieRev , us_minRev);
  mappedSpeed = 255 - map(mappedSpeed, us_maxRev, us_minRev, 0, 255);
  
  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 1);
  analogWrite(pin_Q3, mappedSpeed);
  digitalWrite(pin_Q4, 0);

  digitalWrite(pin_brakeLED, LOW);
}

//=================calibration=================
void calibMode(){
  //wait for user release
  while(digitalRead(pin_brakeLED)){}
 
  neutralCalib();
  writeCalib();

  fwdCalib();
  writeCalib();

  revCalib();
  writeCalib();

  offCalib();
  writeCalib();

  //happy done dance
  uint8_t i = 3;
  while(i){
    digitalWrite(pin_Q1, HIGH);
    delay1(50);
    digitalWrite(pin_Q1, LOW);
    delay1(50);
    digitalWrite(pin_Q2, HIGH);
    delay1(50);
    digitalWrite(pin_Q2, LOW);
    delay1(50);
    digitalWrite(pin_brakeLED, HIGH);
    delay1(50);
    digitalWrite(pin_brakeLED, LOW);
    delay1(50);
    i--;
  }
  
  flashies(3); //done
}

void neutralCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate neutral test, brake and fwd flash
  while(!digitalRead(pin_brakeLED)){ 
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_Q2, HIGH);
    delay1(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_Q2, LOW);
    delay1(100);
  }

  //user released, acknowledge
  uint8_t i = 8; // for flashies
  while(i){
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_Q2, HIGH);
    delay1(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_Q2, LOW);
    delay1(100);
    i--;
  }

  //set back output
  pinMode(pin_brakeLED, OUTPUT);
  
  us_neutral = pulseAvgr(no_pulseAvg);
}

void fwdCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate fwd test
  while(!digitalRead(pin_brakeLED)){ 
    digitalWrite(pin_Q1, HIGH);
    delay1(100);
    digitalWrite(pin_Q1, LOW);
    delay1(100);
  }

  //user released, acknowledge
  uint8_t i = 8;   
  while(i){
    digitalWrite(pin_Q1, HIGH);
    delay1(100);
    digitalWrite(pin_Q1, LOW);
    delay1(100);
    i--;
  }

  //set back output
  pinMode(pin_brakeLED, OUTPUT);

  us_maxFwd = pulseAvgr(no_pulseAvg);  
}

void revCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate rev test
  while(!digitalRead(pin_brakeLED)){ 
    digitalWrite(pin_Q2, HIGH);
    delay1(100);
    digitalWrite(pin_Q2, LOW);
    delay1(100);
  }

  //user released, acknowledge
  uint8_t i = 8;
  while(i){
    digitalWrite(pin_Q2, HIGH);
    delay1(100);
    digitalWrite(pin_Q2, LOW);
    delay1(100);
    i--;
  }

  //set back input
  pinMode(pin_brakeLED, OUTPUT);

  us_maxRev = pulseAvgr(no_pulseAvg);  
}

void offCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate radio off test
  uint8_t i = 8;
  while(!digitalRead(pin_brakeLED)){
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_Q2, LOW);
    delay1(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_Q2, HIGH);
    delay1(100);
    i--;
  }
  digitalWrite(pin_Q2, LOW);
  
  //set back early
  pinMode(pin_brakeLED, OUTPUT);
  
  //acknowledge input, with red to indicate OFF?
  i = 8;
  while(i){
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_brakeLED, LOW);
    delay1(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_brakeLED, HIGH);
    delay1(100);
    i--;
  }

  //reset LEDs after wig wag
  digitalWrite(pin_brakeLED, LOW);
  
  us_radOff = pulseAvgr(no_pulseAvg);
}

uint16_t pulseAvgr(uint8_t no_pulses){
  //zero out mean time
  us_meanTime = 0;
  
  //stop timer
  TCCR1 = 0; //stop the timer
  TCNT1 = 0; //zero timer count
  
  no_pulseLeft = no_pulses; //start counting
  flag_calib = true;
   
  initINT(); //start the interrupter
  
  while(no_pulseLeft){
    //https://youtu.be/3gWTTQWrB3I?t=16
  }

  //stop interrupt
  //GIMSK = 0;
  GIMSK &= ~_BV(INT0);
  
  flag_calib = false;
    
  //stop timer
  TCCR1 = 0; //stop the timer
  TCNT1 = 0; //zero timer count
  
  return us_meanTime;
}
