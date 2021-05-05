#include <EEPROM.h>
#include <ResponsiveAnalogRead.h>

/*MOTER CANTRALLER
  oh_bother 2021
  punch me if you use this code, fucker
  
  controls a DC brushed motor (ultradash)
  using an attiny85 with an H bridge and inverting 
  transistors on the P channels so +5v = mosfet fun time.
  Input signa:
  20ms start to start
  2ms full forward
  1ms full rev
  1.5 neutral
  
  receivers/transmitters/pulses/controllers all vary
  so there's also a calibration mode if pin 1 of the 
  tiny85 is set positive at boot.

  LEDs indicate which function is calibrated and writes
  to eeprom each time. neutral, fwd, rev, then radio off
  user taps red LED (pin 1) high to get to next test

  results stay in eeprom till forever.
  
  MOSFET order:
  Q1 Q2
  Q3 Q4

  uses https://github.com/damellis/attiny cores.
  
  pinout:
  Pin7 (PB2)  Radio IN
  Pin2 (PB3)  Q1 (fwd LED)
  Pin3 (PB4)  Q2 (rev LED)
  Pin5 (PB0)  Q3
  Pin6 (PB1)  Q4
  Pin1 (PB5)  brake LED/user input
*/

// ====== variables =========
//version for eeprom
const uint8_t no_secret = 0;

//pulse times in uS
const int us_weenieRev = 1320; //I am weenie
const int ms_doubleTap = 1000; //time to rev in ms
const int calibTimeOut = 1000; //time to set pin 1 low
const int us_dutyCycle = 30000; //timeout between pulses
const int us_offset = 20;

const uint16_t ms_revTime = 2000;
const uint16_t ms_trigTime = 350;
const uint8_t no_taps = 2;
const uint8_t ms_loopTime = 20;
const uint8_t no_timeOuts = 10; //number of timeouts to neut

//pins
const int pin_Q1 = 3;
const int pin_Q2 = 4;
const int pin_Q3 = 0;
const int pin_Q4 = 1;

const int pin_radio = 2;
const int pin_brakeLED = 5;

//settings/vars
uint16_t us_maxFwd = 2000;
uint16_t us_neutral = 1500;
uint16_t us_maxRev = 1000;
uint16_t us_radOff = 1493;

uint16_t us_minFwd = us_neutral + us_offset;
uint16_t us_minRev = us_neutral - us_offset;

//vars
int ms_revTimer = 0;
uint8_t no_revTaps = 0;
bool flag_rev = false;
uint8_t no_timeOutCount = 0;

//library stuff
ResponsiveAnalogRead analog(0, true);

// ====== setup =========
void setup() {
  //pins
  pinMode(pin_Q1,OUTPUT);
  pinMode(pin_Q2,OUTPUT);
  pinMode(pin_Q3,OUTPUT);
  pinMode(pin_Q4,OUTPUT);
  
  pinMode(pin_brakeLED, INPUT);
  pinMode(pin_radio, INPUT);

  //initial pin states
  digitalWrite(pin_Q1, LOW);
  digitalWrite(pin_Q2, LOW);
  digitalWrite(pin_Q3, LOW);
  digitalWrite(pin_Q4, LOW);

  //hold pin low from start
  if(digitalRead(pin_brakeLED)){
    calibMode();
  }

  //reset pin, continue
  pinMode(pin_brakeLED, OUTPUT);

  //check and load eeprom
  varInit();

  //done
  flashies(2);
}

// ====== loop =========
void loop() {
  //get pulse time
  uint16_t us_pulseTime = pulseIn(pin_radio, HIGH, us_dutyCycle);

  //neutral if read bad or transmitter off
  if(us_pulseTime == 0 || us_pulseTime == us_radOff + 2 || us_pulseTime == us_radOff - 2){
    if(no_timeOutCount == no_timeOuts){
      neutral();
    }else{no_timeOutCount++;}
  }else{
    no_timeOutCount = 0;
  }

  //responisve analog smoothing
  us_pulseTime -= 1000;
  analog.update(us_pulseTime);
  us_pulseTime = analog.getValue();
  us_pulseTime += 1000;

  //pulse is neutral
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

  //deduct timer
  ms_revTimer = constrain(ms_revTimer - ms_loopTime, 0, 0x7FFF);
  delay(4);
}

// ====== functions =========
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
  
  us_minFwd = us_neutral + us_offset;
  us_minRev = us_neutral - us_offset;
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

//flashes brake LED, each flash is 400ms
void flashies(uint8_t i){
  while(i){
    digitalWrite(pin_brakeLED, HIGH);
    delay(200);
    digitalWrite(pin_brakeLED, LOW);
    delay(200);
    i--;
  }
}

//speed functions
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

void forward(uint16_t rate){
//  digitalWrite(pin_Q1, 1);
//  digitalWrite(pin_Q2, 0);
//  digitalWrite(pin_Q3, 0);
//  digitalWrite(pin_Q4, 0);

  uint16_t mappedSpeed = 0;
  
  mappedSpeed = constrain(rate, us_minFwd , us_maxFwd);
  mappedSpeed = map(mappedSpeed, us_minFwd, us_maxFwd, 0, 255);

  digitalWrite(pin_Q1, 1);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 0);
  analogWrite(pin_Q4, mappedSpeed);

  digitalWrite(pin_brakeLED, LOW);
}

void brake(uint16_t rate){
//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 1);
//  digitalWrite(pin_Q3, 0);
//  digitalWrite(pin_Q4, 0);

  uint16_t mappedSpeed = 0;
  
  mappedSpeed = constrain(rate, us_maxRev, us_minRev);
  mappedSpeed = 255 - map(mappedSpeed, us_maxRev, us_minRev, 0, 255);

  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 1);
  analogWrite(pin_Q4, mappedSpeed);

  digitalWrite(pin_brakeLED, HIGH);
}

void reverse(uint16_t rate){
//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 0);
//  digitalWrite(pin_Q3, 1);
//  digitalWrite(pin_Q4, 0);

  uint16_t mappedSpeed = 0;
    
  //weenieRev goes here
  mappedSpeed = constrain(rate, us_maxRev , us_minRev);
  mappedSpeed = 255 - map(mappedSpeed, us_maxRev, us_minRev, 0, 255);
  
  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 1);
  analogWrite(pin_Q3, mappedSpeed);
  digitalWrite(pin_Q4, 0);

  digitalWrite(pin_brakeLED, LOW);
}

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
    delay(50);
    digitalWrite(pin_Q1, LOW);
    delay(50);
    digitalWrite(pin_Q2, HIGH);
    delay(50);
    digitalWrite(pin_Q2, LOW);
    delay(50);
    digitalWrite(pin_Q3, HIGH);
    delay(50);
    digitalWrite(pin_Q3, LOW);
    delay(50);
    i--;
  }
  
  flashies(5); //done
}

void neutralCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate neutral test, brake and fwd flash
  while(!digitalRead(pin_brakeLED)){ 
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_Q2, HIGH);
    delay(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_Q2, LOW);
    delay(100);
  }

  //user released, acknowledge
  uint8_t i = 8; // for flashies
  while(i){
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_Q2, HIGH);
    delay(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_Q2, LOW);
    delay(100);
    i--;
  }

  //set back output
  pinMode(pin_brakeLED, OUTPUT);
  
  us_neutral = pulseAvgr();
}

void fwdCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate fwd test
  while(!digitalRead(pin_brakeLED)){ 
    digitalWrite(pin_Q1, HIGH);
    delay(100);
    digitalWrite(pin_Q1, LOW);
    delay(100);
  }

  //user released, acknowledge
  uint8_t i = 8;   
  while(i){
    digitalWrite(pin_Q1, HIGH);
    delay(100);
    digitalWrite(pin_Q1, LOW);
    delay(100);
    i--;
  }

  //set back output
  pinMode(pin_brakeLED, OUTPUT);
  
  us_maxFwd = pulseAvgr();  
}

void revCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate rev test
  while(!digitalRead(pin_brakeLED)){ 
    digitalWrite(pin_Q2, HIGH);
    delay(100);
    digitalWrite(pin_Q2, LOW);
    delay(100);
  }

  //user released, acknowledge
  uint8_t i = 8;
  while(i){
    digitalWrite(pin_Q2, HIGH);
    delay(100);
    digitalWrite(pin_Q2, LOW);
    delay(100);
    i--;
  }

  //set back input
  pinMode(pin_brakeLED, OUTPUT);
  
  us_maxRev = pulseAvgr();  
}

void offCalib(){
  //ready input
  pinMode(pin_brakeLED, INPUT);
  
  //indicate radio off test
  uint8_t i=8;
  while(!digitalRead(pin_brakeLED)){
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_Q2, LOW);
    delay(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_Q2, HIGH);
    delay(100);
    i--;
  }
  digitalWrite(pin_Q2, LOW);
  
  //set back early
  pinMode(pin_brakeLED, OUTPUT);
  
  //acknowledge input, with red to indicate OFF?
  i=8;
  while(i){
    digitalWrite(pin_Q1, HIGH);
    digitalWrite(pin_brakeLED, LOW);
    delay(100);
    digitalWrite(pin_Q1, LOW);
    digitalWrite(pin_brakeLED, HIGH);
    delay(100);
    i--;
  }

  //reset LEDs after wig wag
  digitalWrite(pin_brakeLED, LOW);

  
  us_radOff = pulseAvgr();
}

uint16_t pulseAvgr(){
  int i = 50;
  uint16_t us_meanTime = pulseIn(pin_radio, HIGH, us_dutyCycle);
  
  while(i){
    uint16_t us_pulseTime = pulseIn(pin_radio, HIGH, us_dutyCycle);
    us_meanTime = (0.9*us_meanTime) + (0.1*us_pulseTime);
    i--;
  }
  
  return us_meanTime;
}
