/*

  
*/

int DelayStep = 1; //number of steps in each loop - this affects the speed of all servos - higher is faster

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwmMain = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwmBase = Adafruit_PWMServoDriver(0x40);

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFPlayerMini_Fast.h>
#include <OneButton.h>
#include "Buttons.h"

SoftwareSerial dfSoftwareSerial(9, 10); // DFPlayer RX, TX
SoftwareSerial suSoftwareSerial(16, 17); // Speakup RX, TX
DFPlayerMini_Fast myDFPlayer;
OneButton switchButton1(8, true);
OneButton volButtonDown(2, true);
OneButton volButtonUp(3, true);

int incomingByte = 9999;

uint16_t RunMode = 0;

long timer = 0;
#define lTiltHome 300
uint16_t lTiltPos = lTiltHome; 
uint16_t lTiltPrevPos = lTiltHome; 

#define rTiltHome 300
uint16_t rTiltPos = rTiltHome; 
uint16_t rTiltPrevPos = rTiltHome; 

#define turnHome 300
uint16_t turnPos = turnHome; 
uint16_t turnPrevPos = turnHome; 

#define earHome 300
uint16_t earPos = earHome; 
uint16_t earPrevPos = earHome;

#define holoHome 300
uint16_t holoPos = holoHome;
uint16_t holoPrevPos = holoHome;

#define aperHome 300
uint16_t aperPos = aperHome;
uint16_t aperPrevPos = aperHome;

#define bodyHome 300
uint16_t bodyPos = bodyHome; 
uint16_t bodyPrevPos = bodyHome; 

#define neckHome 350
uint16_t neckPos = neckHome;
uint16_t neckPrevPos = neckHome;

#define legsHome 270
uint16_t legsPos = legsHome;
uint16_t legsPrevPos = legsHome;

uint16_t earDelay;
uint16_t earDelayCount;
uint16_t holoDelay;
uint16_t holoDelayCount;
uint16_t aperDelay;
uint16_t aperDelayCount;
uint16_t bodyDelay;
uint16_t bodyDelayCount;
uint16_t neckDelay;
uint16_t neckDelayCount;
uint16_t turnDelay;
uint16_t turnDelayCount;
uint16_t rTiltDelay;
uint16_t rTiltDelayCount;
uint16_t lTiltDelay;
uint16_t lTiltDelayCount;
uint16_t legsDelay;
uint16_t legsDelayCount;

unsigned long volTimer = 0;
int volControl;
int volPrevControl = 15;



bool bEars = false;
bool bHolo = false;
bool bAper = false;
bool bBody = false;
bool bNeck = false;
bool bTurn = false;
bool bRTilt = false;
bool bLTilt = false;
bool bLegs = false;

bool bProjector = false;
bool bDance = false;


uint8_t servonum = 0;

//volume settings for charlieplex display
int volCounter = 0;
int volLevel = 6;

void setup() {
 
pwmMain.begin();
pwmBase.begin();

pwmMain.setOscillatorFrequency(26500000);
pwmMain.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
pwmBase.setPWMFreq(50);
  delay(10);

 
  
  //start serial connection
   dfSoftwareSerial.begin(9600);
   
  Serial.begin(9600);
  randomSeed(analogRead(3));

  


myDFPlayer.begin(dfSoftwareSerial);
myDFPlayer.wakeUp();
myDFPlayer.volume(30);

  suSoftwareSerial.begin(115200);

    //Charlieplex pins **********
    #define Charlie1 5
    #define Charlie2 6
    #define Charlie3 7
     
     
      
     //Begin pwmMain **********
     #define pwmMainAddr 1
     #define servoEar1 0
     #define servoEar2 1
     #define ledAperature 6
     #define ledHolo 7
     #define servoAperature 9
     #define servoLTilt 10
     #define servoRTilt 11
     #define servoNeck 12
     #define servoHeadTurn 13
     #define servoHolo 14
     #define servoBody 15
     //End pwmMain **********

     //Begin pwmBase **********
     #define pwmBaseAddr 0
     #define servoLegs 0
     #define ledRed 5
     #define ledGreen 6
     #define ledBlue 7

     //End pwmBase **********

     
     
     

     // servo extents
     #define earMin 200 
     #define earMax 400
     
     #define holoMin 239
     #define holoMax 366
     
     #define aperMin 225
     #define aperMax 375

     #define bodyMin 160
     #define bodyMax 460

     #define neckMin 150
     #define neckMax 460

     #define turnMin 150
     #define turnMax 550

     #define rTiltMin 200
     #define rTiltMax 420

     #define lTiltMin 160 
     #define lTiltMax 370

     #define legsMin 270
     #define legsMax 446

     //chance for each servo to move
     #define earChance 1500
     #define holoChance 5000
     #define aperChance 1000
     #define bodyChance 1000
     #define neckChance 2000
     #define turnChance 1000
     #define rTiltChance 1500
     #define lTiltChance 1500
     #define legsChance 4000 //This servo has to work very hard.  Don't over do it.
     

     //delay paramaters for servo speed - less delay and more steps is faster
     #define DelayMin 1 //minumum delay between servo steps - lower is faster
     #define DelayMed 4 //medium delay between servo steps - servos moving heavier parts should have a lower top speed
     #define DelayMax 10 //maximum delay between servo steps - higher is slower
     int DelayStep = 1; //number of steps in each loop - this affects the speed of all servos - higher is faster
     
     //CENTRE ALL SERVOS **************

    pwmMain.setPWM(servoLTilt, 0, lTiltHome);
    pwmMain.setPWM(servoRTilt, 0, rTiltHome);
    pwmMain.setPWM(servoHeadTurn, 0, turnHome);
    pwmMain.setPWM(servoEar1, 0, earHome);
    pwmMain.setPWM(servoEar2, 0, earHome);
    pwmMain.setPWM(servoHolo , 0, holoHome);
    pwmMain.setPWM(servoAperature  , 0, aperHome);
    pwmMain.setPWM(servoBody, 0, bodyHome);
    pwmMain.setPWM(servoNeck, 0, neckHome);
    pwmMain.setPWM(ledAperature, 0, 800);
    pwmMain.setPWM(ledHolo, 0, 800);
    
    //pwmBase.setPWM(servoLegs, 0, legsHome);
    pwmBase.setPWM(servoLegs, 0, legsMin);
    
    //END CENTRE ALL SERVOS **************

    // Switch
  switchButton1.setClickTicks(300);
  switchButton1.setPressTicks(600);
  switchButton1.attachClick(switch1_Click);
  switchButton1.attachDoubleClick(switch1_Doubleclick);
  switchButton1.attachLongPressStart(switch1_LongPressStart);

  volButtonDown.setClickTicks(300);
  volButtonDown.setPressTicks(600);
  volButtonDown.attachClick(switch2_Click);
  volButtonDown.attachDoubleClick(switch2_Doubleclick);
  volButtonDown.attachLongPressStart(switch2_LongPressStart);

  volButtonUp.setClickTicks(300);
  volButtonUp.setPressTicks(600);
  volButtonUp.attachClick(switch3_Click);
  volButtonUp.attachDoubleClick(switch3_Doubleclick);
  volButtonUp.attachLongPressStart(switch3_LongPressStart);

  
  // end Switch



//#define DEBUGMODE
  
delay(1000);
     #ifdef DEBUGMODE
     Serial.println("debugmode");
  pwmBase.setPWM(servoLegs, 0, legsMin);
      delay(2000);
      pwmBase.setPWM(servoLegs, 0, legsMax);
      delay(2000);
       pwmBase.setPWM(servoLegs, 0, legsHome);
      delay(2000);
      pwmBase.setPWM(servoLegs, 0, legsMax);
      delay(2000);
      pwmBase.setPWM(servoLegs, 0, legsMin);
     
      
     #endif
    

     // pwmMain.setPWM(servoNeck, 0, neckMax);
     // delay(20000);

      
      
  #define pirPin  14
 // pinMode(pirPin, INPUT);   
  
   myDFPlayer.volume(volLevel * 5);
  myDFPlayer.play(5);
  delay(1500);
  
}


// BEGIN LOOP ***************
void loop() {


  
  switchButton1.tick();
  volButtonDown.tick();
  volButtonUp.tick();

//volume charlieplexed LEDs

       
        CharliePlex(volLevel);
        

       if (random(1,20) == 1) {
          CharliePlex(random(1,7));
        }
  

  switch(RunMode) {
      case RunAuto:

        


        // randomly flicker the charlieplex strip
        
        
          
        //chance to sleep
        
        if (random(20000) == 1){
          
      //    pwmMain.setPWM(ledHolo, 0, 0);
      //    delay(random(2000,15000));
          
        }
        
        DelayStep = 1;
        // begin ears **************
          MoveServo(pwmMainAddr, servoEar2, bEars, earPrevPos, earPos, earMin, earMax, earDelay, earDelayCount, DelayMin, DelayMax, bodyChance, 0, 0);
        // end ears **************
        
        // begin holoprojector **************
          MoveServo(pwmMainAddr, servoHolo, bHolo, holoPrevPos, holoPos, holoMin, holoMax, holoDelay, holoDelayCount, DelayMin, DelayMax, holoChance, 0, 0);
        // end holoprojector **************
        
        // begin aperature **************
          MoveServo(pwmMainAddr, servoAperature, bAper, aperPrevPos, aperPos, aperMin, aperMax, aperDelay, aperDelayCount, DelayMin, DelayMax, aperChance, 0, 0);
        // end aperature **************
        
        // begin body **************
          MoveServo(pwmMainAddr, servoBody, bBody, bodyPrevPos, bodyPos, bodyMin, bodyMax, bodyDelay, bodyDelayCount, DelayMin, DelayMax, bodyChance, bodyHome +50 , 0);
        // end body **************
        
        // begin neck **************
          MoveServo(pwmMainAddr, servoNeck, bNeck, neckPrevPos, neckPos, neckMin, neckMax, neckDelay, neckDelayCount, DelayMed, DelayMax, neckChance, 0, 0);
        // end neck **************
        
        // begin turn **************
          MoveServo(pwmMainAddr, servoHeadTurn, bTurn, turnPrevPos, turnPos, turnMin, turnMax, turnDelay, turnDelayCount, DelayMin, DelayMax, turnChance, 0, 0);
        // end turn **************
        
        
        // begin rTilt **************
          MoveServo(pwmMainAddr, servoRTilt, bRTilt, rTiltPrevPos, rTiltPos, rTiltMin, rTiltMax, rTiltDelay, rTiltDelayCount, DelayMin, DelayMax, rTiltChance, 0, 0);
        // end rTilt **************
        
        
        // begin lTilt **************
          MoveServo(pwmMainAddr, servoLTilt, bLTilt, lTiltPrevPos, lTiltPos, lTiltMin, lTiltMax, lTiltDelay, lTiltDelayCount, DelayMin, DelayMax, lTiltChance, 0, 0);
        // end lTilt **************
        
        
        
        // begin legs **************
          MoveServo(pwmBaseAddr, servoLegs, bLegs, legsPrevPos, legsPos, legsMin, legsMax, legsDelay, legsDelayCount, DelayMed, DelayMax, legsChance, legsMin, 0);
        // end legs **************
        
        break;
      case RunMotion:
         if (digitalRead(pirPin)  == HIGH) {            // if motion detected
           LEDRGB(0xFFFFFF);
           RunMode = RunAuto;
         } 
         else {
           LEDRGB(0xFF4400);
         }
         //Serial.println(analogRead(0));
         
        
        break;
      case RunVoice:
        HomeAll();
        if (!bEars &&  !bHolo &&   !bAper &&  !bBody && !bNeck &&  !bTurn &&  !bLTilt &&  !bRTilt &&  !bLegs){
          if (suSoftwareSerial.available() > 0) {
            // read the incoming byte:
            incomingByte = suSoftwareSerial.read();
            Serial.println(incomingByte);
          }
          else {
            incomingByte = 9999;
          }
          
          switch(incomingByte) {
            case 1 ... 2:
              Dance();
              break;
            case 3 ... 8:
              Projector();
              break;
          }
        }
  
      
      
      break;
    }

    

  
  


if (random(20000)==1){
 // Projector();
}



  

if (random (4000)==1) {

 //myDFPlayer.play(random(1,69));
 
}


  } 
  
  // END LOOP ***********************



  


/*
 * servoName - Name of servo to move  eg. servoLegs
 * bEnableServo - Boolean for the servo eg. bLegs
 * prevPos - Servo's previous position eg. legsPrevPos
 * currPos- Servo's current position eg. legsPos
 * servoMin - Servo's minimum extent eg. legsMin
 * servoMax - Servo's maximum extent eg. legsMax
 * servoDelay - Servo's delay random number between delMin and delMax eg. legsDelay
 * delayCount - Counter to keep track of where we are eg. legsDelayCount
 * delMin - Minimum delay (fastest speed) for servoDelay eg. DelayMed or DelayMin
 * delMax - Maximum delay (slowest speed) for servoDelay eg. DelayMin or DelayMed
 * returnPos - If not zero, then return to this position after movement eg. legsHome or legsMin
 */
void MoveServo(uint16_t pwmAddress, uint16_t servoName, bool& bEnableServo, uint16_t& prevPos, uint16_t& currPos, uint16_t servoMin, uint16_t servoMax, uint16_t& servoDelay, uint16_t& delayCount, uint16_t delMin, uint16_t delMax, uint16_t chanceToMove, uint16_t returnPos, uint16_t destPos ) {
if (bEnableServo) {
  if (delayCount >= servoDelay) {
    if (prevPos <= currPos) {
      prevPos = prevPos + DelayStep;
      if (prevPos > currPos) {
        prevPos = currPos;
      }
    }
  
    if (prevPos > currPos) {
      prevPos = prevPos - DelayStep;
      if (prevPos < currPos) {
        prevPos = currPos;
      }
    }
    //check to see which PWM board the device is on.
    if (pwmAddress == pwmMainAddr){
      pwmMain.setPWM(servoName, 0, prevPos);
    }
    else{
      pwmBase.setPWM(servoName, 0, prevPos);
    }
    
    delayCount = 0;
    if (prevPos == currPos) {
      bEnableServo = false;
     }

     //some servos have an extra function, like LEDS etc.
    switch(servoName) {
      case servoAperature:
        pwmMain.setPWM(ledAperature, 0, map(aperPrevPos,aperMin,aperMax,0,4095)); // brighten the aperature LED as the aperature opens
        break;
      case servoHolo:
        if (bEnableServo){
          // a little flicker before setting to a value relative to the holoprojector position
          pwmMain.setPWM(ledHolo, 0, 4095); 
          pwmMain.setPWM(ledHolo, 0, holoPos * 17);
        }
        else {
          pwmMain.setPWM(ledHolo, 0, 400);
        }
        break;
      case servoEar2:
        pwmMain.setPWM(servoEar1, 0,map(earPrevPos,earMin,earMax,earMax,earMin));
      break;
    }
  }
  delayCount++;
}
else { 
   if (destPos != 0 && currPos != destPos) { //move to specific position
    servoDelay = 1;
    prevPos = currPos;
    currPos = destPos;
    bEnableServo = true;  
    }
    
    if (random (chanceToMove)==1 && destPos == 0) { //random chance of moving to a random position
      servoDelay = random(delMin, delMax);
      prevPos = currPos;
      currPos = (random (servoMin, servoMax));
      bEnableServo = true;
      if (servoName == servoHolo) {
        myDFPlayer.play(random(1,69));
      }
    }
    
  if (returnPos !=0 && bEnableServo == false && currPos != returnPos){ //after moving, return
    servoDelay = DelayMax;
    currPos = returnPos;
    bEnableServo = true;    
  }
}
}

 void LEDRGB(long hexValue){
    byte r=(byte)(hexValue>>16);
    byte g=(byte)(hexValue>>8);
    byte b=(byte)(hexValue);
    pwmBase.setPWM(ledRed, 0, map(r, 0,255,0,4095));
    pwmBase.setPWM(ledGreen, 0, map(g, 0,255,0,4095));
    pwmBase.setPWM(ledBlue, 0, map(b, 0,255,0,4095));
 }

 void LEDRGBFade(long hexValue){
    byte r=(byte)(hexValue>>16);
    byte g=(byte)(hexValue>>8);
    byte b=(byte)(hexValue);
    pwmBase.setPWM(ledRed, 0, map(r, 0,255,0,4095));
    pwmBase.setPWM(ledGreen, 0, map(g, 0,255,0,4095));
    pwmBase.setPWM(ledBlue, 0, map(b, 0,255,0,4095));
 }

 void ResetServos(){
  bEars = false;
  bHolo = false;
  bAper = false;
  bBody = false;
  bNeck = false;
  bTurn = false;
  bLTilt = false;
  bRTilt = false;
  bLegs = false;

 }
 void HomeAll() {
  /*  For Debugging
          Serial.println("----");
          Serial.println(bEars);
          Serial.println(bHolo); 
          Serial.println(bAper); 
          Serial.println(bBody); 
          Serial.println(bNeck); 
          Serial.println(bTurn); 
          Serial.println(bLTilt);
          Serial.println(bRTilt);
          Serial.println(bLegs);
  */
  DelayStep = 1;
  // begin ears **************
   MoveServo(pwmMainAddr, servoEar2, bEars, earPrevPos, earPos, earMin, earMax, earDelay, earDelayCount, DelayMin, DelayMax, 1, 0, earHome);
  // end ears **************
  
  // begin holoprojector **************
  MoveServo(pwmMainAddr, servoHolo, bHolo, holoPrevPos, holoPos, holoMin, holoMax, holoDelay, holoDelayCount, DelayMin, DelayMax, 1, 0, holoHome);
  // end holoprojector **************
  
  // begin aperature **************
  MoveServo(pwmMainAddr, servoAperature, bAper, aperPrevPos, aperPos, aperMin, aperMax, aperDelay, aperDelayCount, DelayMin, DelayMax, 1, 0, aperHome);
  // end aperature **************

  // begin body **************
  MoveServo(pwmMainAddr, servoBody, bBody, bodyPrevPos, bodyPos, bodyMin, bodyMax, bodyDelay, bodyDelayCount, DelayMed, DelayMax, 1, 0, bodyHome);
  // end body **************

  // begin neck **************
    MoveServo(pwmMainAddr, servoNeck, bNeck, neckPrevPos, neckPos, neckMin, neckMax, neckDelay, neckDelayCount, DelayMed, DelayMax, 1, 0, neckHome);
  // end neck **************
  
  // begin turn **************
    MoveServo(pwmMainAddr, servoHeadTurn, bTurn, turnPrevPos, turnPos, turnMin, turnMax, turnDelay, turnDelayCount, DelayMin, DelayMax, 1, 0, turnHome);
  // end turn **************
  
  
  // begin rTilt **************
    MoveServo(pwmMainAddr, servoRTilt, bRTilt, rTiltPrevPos, rTiltPos, rTiltMin, rTiltMax, rTiltDelay, rTiltDelayCount, DelayMin, DelayMax, 1, 0, rTiltHome);
  // end rTilt **************
  
  
  // begin lTilt **************
    MoveServo(pwmMainAddr, servoLTilt, bLTilt, lTiltPrevPos, lTiltPos, lTiltMin, lTiltMax, lTiltDelay, lTiltDelayCount, DelayMin, DelayMax, 1, 0, lTiltHome);
  // end lTilt **************
  
  
  
  // begin legs **************
    MoveServo(pwmBaseAddr, servoLegs, bLegs, legsPrevPos, legsPos, legsMin, legsMax, legsDelay, legsDelayCount, DelayMed, DelayMax, legsChance, 0, legsHome);
  // end legs **************
 }

 void CharliePlex(int ledNum){

  switch (ledNum) {
    case 1: 
      pinMode(Charlie1, OUTPUT);     
      digitalWrite(Charlie1, HIGH);
      pinMode(Charlie2, OUTPUT);     
      digitalWrite(Charlie2, LOW);   
      pinMode(Charlie3, INPUT);      
      digitalWrite(Charlie3, LOW);
      break;     
      
      
    case 2:
      pinMode(Charlie1, OUTPUT);     
      digitalWrite(Charlie1, LOW);
      pinMode(Charlie2, OUTPUT);     
      digitalWrite(Charlie2, HIGH);  
      pinMode(Charlie3, INPUT);      
      digitalWrite(Charlie3, LOW);
      
      break;
      
    case 3:
      pinMode(Charlie1, INPUT);     
      digitalWrite(Charlie1, LOW);
      pinMode(Charlie2, OUTPUT);    
      digitalWrite(Charlie2, HIGH);  
      pinMode(Charlie3, OUTPUT);    
      digitalWrite(Charlie3, LOW);
      break;
      
    case 4:  
      pinMode(Charlie1, INPUT);      
      digitalWrite(Charlie1, LOW);
      pinMode(Charlie2, OUTPUT);     
      digitalWrite(Charlie2, LOW);  
      pinMode(Charlie3, OUTPUT);     
      digitalWrite(Charlie3, HIGH);
      break;   
            
    case 5:
      pinMode(Charlie1, OUTPUT);    
      digitalWrite(Charlie1, LOW);
      pinMode(Charlie2, INPUT);     
      digitalWrite(Charlie2, LOW);
      pinMode(Charlie3, OUTPUT);    
      digitalWrite(Charlie3, HIGH);
      break;
      
      
    case 6:
      pinMode(Charlie1, OUTPUT);
      digitalWrite(Charlie1, HIGH);
      pinMode(Charlie2, INPUT);
      digitalWrite(Charlie2, LOW);
      pinMode(Charlie3, OUTPUT);
      digitalWrite(Charlie3, LOW);
      break;
     
  
  }
 }
  

  void goToSleep(){
    for (int  i = 450; i >= 250 ; i--){
    
    
    
    pwmMain.setPWM(servoLTilt, 0,i);
    pwmMain.setPWM(servoRTilt, 0, map(i,rTiltMin,rTiltMax,rTiltMax,rTiltMin));
     
    pwmMain.setPWM(servoNeck, 0, i);
     
   
    pwmMain.setPWM(servoEar1, 0, map(i,earMin,earMax,earMax,earMin));
    pwmMain.setPWM(servoEar2, 0, i);

    pwmMain.setPWM(ledAperature, 0, i);
    pwmMain.setPWM(ledHolo, 0, i);
     

    
    //delay(1);
     }

      pwmMain.setPWM(ledAperature, 0, 0);
    pwmMain.setPWM(ledHolo, 0, 0);
      pwmMain.setPWM(servoAperature, 0, aperMin);

    pwmMain.setPWM(servoHolo, 0, holoMin);
   
    //pwmMain.setPWM(servoBody, 0, bodyMin); 
    
    //pwmMain.setPWM(servoHeadTurn, 0, 445);

  }
  void Projector() {
    
    timer = millis() + 38000;
    myDFPlayer.play(75);
    pwmMain.setPWM(servoAperature, 0, aperMax);
    pwmMain.setPWM(servoHolo, 0, holoHome);
    pwmMain.setPWM(servoEar1, 0,map(250,earMin,earMax,earMax,earMin));
    pwmMain.setPWM(servoEar2, 0,250);
    pwmMain.setPWM(servoLTilt, 0,lTiltMax);
    pwmMain.setPWM(servoRTilt, 0, rTiltMin);
    bProjector = true;
    
    do{
      pwmMain.setPWM(ledHolo, 0, random(0,4000));
    delay(5);
 
  }
  while (millis() < timer );
  

  delay(100);

}

void Dance() {
  HomeAll();
  delay(200);
  pwmMain.setPWM(servoNeck, 0,neckMin);
  neckPrevPos = neckMin;
  
  bool bMove = false;
  myDFPlayer.play(76);
  // ****** move ears
  for (int i = 0; i < 8; i++){
    if (bMove) {

      pwmMain.setPWM(servoEar1, 0,earMin);
      pwmMain.setPWM(servoEar2, 0,earMin);
    }
    else {

      pwmMain.setPWM(servoEar1, 0,earMax);
      pwmMain.setPWM(servoEar2, 0,earMax);
    }

    bMove = !bMove;
    delay(465);
  }

  bMove = false;
  // ****** move head and ears
  for (int i = 0; i < 8; i++){
    if (bMove) {
      pwmMain.setPWM(servoLTilt, 0,lTiltHome-50);
      pwmMain.setPWM(servoRTilt, 0,rTiltHome+50);
      pwmMain.setPWM(servoEar1, 0,earMin);
      pwmMain.setPWM(servoEar2, 0,earMax);
     
      
    }
    else {
      pwmMain.setPWM(servoLTilt, 0,lTiltHome+50);
      pwmMain.setPWM(servoRTilt, 0,lTiltHome-50);
      pwmMain.setPWM(servoEar1, 0,earMax);
      pwmMain.setPWM(servoEar2, 0,earMin);
      
    }

    bMove = !bMove;
    delay(465);
    
  }
  
  
  bMove = false;
  // move head and ears
  for (int i = 0; i < 8; i++){
    if (bMove) {
      pwmMain.setPWM(servoLTilt, 0,lTiltHome+75);
      pwmMain.setPWM(servoRTilt, 0,rTiltHome+75);
      pwmMain.setPWM(servoEar1, 0,earMin);
      pwmMain.setPWM(servoEar2, 0,earMin);
      
    }
    else {
      pwmMain.setPWM(servoLTilt, 0,rTiltHome-75);
      pwmMain.setPWM(servoRTilt, 0,lTiltHome-75);
      pwmMain.setPWM(servoEar1, 0,earMax);
      pwmMain.setPWM(servoEar2, 0,earMax);
      
    }

    bMove = !bMove;
    delay(465);
    bProjector = true;
  }

  bMove = false;
  bool bMoveLegs = false;
  // move head, ears and legs
  for (int i = 0; i < 8; i++){
    if (bMove) {
      pwmMain.setPWM(servoLTilt, 0,lTiltHome+50);
      pwmMain.setPWM(servoRTilt, 0,lTiltHome-50);
      pwmMain.setPWM(servoEar1, 0,earMax);
      pwmMain.setPWM(servoEar2, 0,earMin);
      
    }
    else {// move legs every other motion
      if (bMoveLegs) {
        pwmBase.setPWM(servoLegs, 0, 390);
        legsPrevPos = 390;
      }
      else{
        pwmBase.setPWM(servoLegs, 0, 440);
      }
      bMoveLegs = !bMoveLegs;
      pwmMain.setPWM(servoLTilt, 0,lTiltHome-50);
      lTiltPrevPos = lTiltHome-50;
      
      pwmMain.setPWM(servoRTilt, 0,rTiltHome+50);
      rTiltPrevPos = rTiltHome+50;
      
      pwmMain.setPWM(servoEar1, 0,earMin);
      pwmMain.setPWM(servoEar2, 0,earMax);
      earPrevPos = earMax;
     
    }

    bMove = !bMove;
    delay(465);
    
  }

  myDFPlayer.pause();

  bNeck = true;
  bLTilt = true;
  bRTilt = true;
  bEars = true;
  bLegs = true;
  
  bDance = true;
   
   
}

void PlaySound(int FileNum){
  myDFPlayer.play(FileNum);
}


void VolDown(){
  if (volLevel >0){
    volLevel--;
    myDFPlayer.volume(volLevel * 5);
    PlaySound(1);
  }
}

void VolUp() {
  if (volLevel <7){
    volLevel++;
    myDFPlayer.volume(volLevel * 5);
    PlaySound(2);
  }
}

void Laugh() {
  myDFPlayer.play(29);

  pwmMain.setPWM(servoHeadTurn, 0, turnHome);
    for (int  i = 0; i < 5 ; i++) {
      
      pwmMain.setPWM(servoEar1, 0, earMin);
      pwmMain.setPWM(servoEar2, 0, earMax);
      pwmMain.setPWM(servoLTilt, 0,lTiltMin + 100);
      pwmMain.setPWM(servoRTilt, 0, rTiltMin + 100);
      delay(200);
      
      
      pwmMain.setPWM(servoEar1, 0, earMax);
      pwmMain.setPWM(servoEar2, 0, earMin);
      pwmMain.setPWM(servoLTilt, 0,lTiltMax - 100);
      pwmMain.setPWM(servoRTilt, 0, rTiltMax - 100);
      delay(200);
    }
    delay(10000);
}
