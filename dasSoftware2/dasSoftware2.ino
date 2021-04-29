/*MOTER CANTRALLER
  oh_bother 2021
  punch me if you use this code, fucker
  
  
  controls a DC brushed motor (ultradashshhhshshs)
  using an attiny85 with an H bridge and inverting 
  transistors on the P channels so +5v = mosfet fun time.
  Input signa:
  15ms start to start
  1.5ms pulse Neutral
  1.93ms pulse Full Throttle
  1.03ms pulse Full Reverse
  1.32ms special weenie reverse mode

  actual serve:
  20ms start to start
  2ms full forward
  1ms full rev
  1.5 neutral
  
  MOSFET order:
  Q1 Q2
  Q3 Q4

  uses https://github.com/damellis/attiny cores.
  
  pinout:
  Pin7 (PB2)  Radio IN
  Pin2 (PB3)  Q1
  Pin3 (PB4)  Q2
  Pin5 (PB0)  Q3
  Pin6 (PB1)  Q4
  Pin1 (PB5)  brake LED
 
  program will receive a pulse every 15ms
  that pulse will set states of mosfets
  default state is neutral (no fets)
  pwm to fets scaled from nothing to full on
  brake activates Q3 and Q4 and brake LED
  rev to neutral to rev(within 1 second)  = rev

  rewrite of this shit to make it not horrible

  pulsein test:
  frsky
  1488-1490 neut
  1936-1938 fwd
  1024-1026 rev
  1566 off
  0 disconnected

  das8ch
  1494-1496 neut
  1942-1944 fwd
  1031-1032 rev
  1492-1494 off (1508-1509 2nd test)
  0 disconnected
*/

// ====== variables =========
//pulse times in uS
const int us_maxFwd = 2000;
const int us_neutral = 1500;
const int us_maxRev = 1000;
const int us_dutyCycle = 21000; //timeout between pulses
const int us_weenieRev = 1320; //I am weenie

const int us_offset = 50;
const int us_minFwd = us_neutral + us_offset;
const int us_minRev = us_neutral - us_offset;
const int us_partRev = us_maxRev - 20;

//other timing
const int ms_doubleTap = 1000; //time to rev in ms
const int calibTimeOut = 1000; //time to set pin 1 low

//other settings
const int analogRes = 255;

//pins
const int pin_Q1 = 3;
const int pin_Q2 = 4;
const int pin_Q3 = 0;
const int pin_Q4 = 1;

const int pin_radio = 2;
const int pin_brakeLED = 5;

//vars
const uint16_t revTime = 1000;
const uint8_t revTapsNo = 2;
int revTimer = 0;
uint8_t revTaps = 0;
uint8_t revPrev = 0;
bool reverseOK = 0;

uint8_t mappedSpeed = 0;
unsigned long pulseTime = 0;

// ====== setup =========
void setup() {
  //pins
  pinMode(pin_Q1,OUTPUT);
  pinMode(pin_Q2,OUTPUT);
  pinMode(pin_Q3,OUTPUT);
  pinMode(pin_Q4,OUTPUT);
  pinMode(pin_brakeLED, INPUT_PULLUP);

  pinMode(pin_radio, INPUT);

  //initial pin states
  digitalWrite(pin_Q1, LOW);
  digitalWrite(pin_Q2, LOW);
  digitalWrite(pin_Q3, LOW);
  digitalWrite(pin_Q4, LOW);

  //hold pin low from start
  if(digitalRead(pin_brakeLED) == LOW){
    calibMode();
  }

  //reset pin, continue
  pinMode(pin_brakeLED, OUTPUT);

  //load progmem
  varInit();
  flashies(2);
}

// ====== loop =========
void loop() {
  //get pulse time
  pulseTime = pulseIn(pin_radio, HIGH);
    
  if(pulseTime == 0){
    neutral();
    revCheck(0);
  }else if(pulseTime <= us_minFwd && pulseTime >= us_minRev){
    neutral(); 
    revCheck(1);
  }else if(pulseTime <= us_minRev && reverseOK){
    mappedSpeed = constrain(pulseTime, us_weenieRev , us_maxRev);
    mappedSpeed = map(mappedSpeed, us_minRev, us_maxRev, 255, 0);
    reverse(mappedSpeed);
    revCheck(1);
  }else if(pulseTime <= us_minRev){
    mappedSpeed = constrain(pulseTime, us_maxRev , 0);
    mappedSpeed = map(mappedSpeed, us_maxRev, us_minRev, 255, 0);
    brake(mappedSpeed);
    if(pulseTime <= us_partRev){revCheck(2);}else{revCheck(3);}
  }else if(pulseTime >= us_minFwd){
    mappedSpeed = constrain(pulseTime, 0 , us_maxFwd);
    mappedSpeed = map(mappedSpeed, us_minFwd, us_maxFwd, 0, 255);
    forward(mappedSpeed);
    revCheck(0);
  }

  delay(4);
}

//uses passed in state to work the reverse flag
void revCheck(int checkNo){

//0 is reset
//1 is neutral
//2 is brake
//3 is trigger'd

//0 if more than 1s has passed triggered
//3 to 1 to 3 to 1, within timer, rev ok
const uint16_t revTime = 1000;
const uint8_t revTapsNo = 2;
uint8_t revTaps = 0;
uint8_t revPrev = 0;
uint16_t revTimer = 0;

  revTimer = revTimer - (us_DutyCycle/1000); //approx cycle time
  revTimer = constrain(revTimer, 0, revTime); 
  
  //did checkNo Change?
  if(revPrev == checkNo){return;}
  if(revPrev == 3 && checkNo == 1){revTaps++;}
  if(revPrev == 1 && checkNo == 3){revTimer = revTime;}
  if(revPrev == 0){revTaps = 0;}
  
  revPrev = checkNo;
}

// ====== functions =========
//load values from progmem
void varInit(){
  //global_variable = sets[arrayloc]
}

void writeCalib(){
  //const PROGMEM uint16_t sets[] = {global_variable, global_variable1, etc}
  delay(1000);  
}

//flashes brake LED, each flash is 400ms
void flashies(int i){
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
  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 0);
  digitalWrite(pin_Q4, 1);

//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 0);
//  digitalWrite(pin_Q3, 0);
//  digitalWrite(pin_Q4, 0);

  digitalWrite(pin_brakeLED, LOW);
}

void forward(uint8_t rate){
  digitalWrite(pin_Q1, 1);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 0);
  digitalWrite(pin_Q4, 0);
  
//  digitalWrite(pin_Q1, 0);
//  analogWrite(pin_Q2, rate);
//  digitalWrite(pin_Q3, 0);
//  analogWrite(pin_Q4, rate);

  digitalWrite(pin_brakeLED, LOW);
}

void brake(uint8_t rate){
  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 1);
  digitalWrite(pin_Q3, 0);
  digitalWrite(pin_Q4, 0);

//  digitalWrite(pin_Q1, 0);
//  digitalWrite(pin_Q2, 0);
//  analogWrite(pin_Q3, rate);
//  analogWrite(pin_Q4, rate);

//  digitalWrite(pin_brakeLED, HIGH);
  digitalWrite(pin_brakeLED, LOW);
}

void reverse(uint8_t rate){
  digitalWrite(pin_Q1, 0);
  digitalWrite(pin_Q2, 0);
  digitalWrite(pin_Q3, 1);
  digitalWrite(pin_Q4, 0);
  
//  analogWrite(pin_Q1, rate);
//  digitalWrite(pin_Q2, 0);
//  analogWrite(pin_Q3, rate);
//  digitalWrite(pin_Q4, 0);

  digitalWrite(pin_brakeLED, LOW);
}

void calibMode(){
  pinMode(pin_brakeLED, OUTPUT);
 
  flashies(12); //time to release gnd
  delay(1000); //done flipping out
  flashies(1); //1st calib

  neutralCalib();
  flashies(2);

  fwdCalib();
  flashies(3);

  revCalib();
  flashies(4);

  revCalib();
  writeCalib();
  flashies(5); //done
}

void neutralCalib(){
  delay(1000);
}

void fwdCalib(){
  delay(1000);  
}

void revCalib(){
  delay(1000);  
}

void offCalib(){
  delay(1000);  
}

unsigned long pulseAvgr(){
  //get pulse time
  unsigned long pulseTime = pulseIn(pin_radio, HIGH);
  
  return pulseTime;
}
