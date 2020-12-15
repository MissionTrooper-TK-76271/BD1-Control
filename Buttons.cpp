
/*
   buttons.cpp

   Created: March 8, 2020
   Author: Andrew Donatelli andrew@donatelli.net


*/


#include "Buttons.h"
#include <DFPlayerMini_Fast.h>


extern uint16_t RunMode;
extern bool bDance;
extern void ResetServos();
extern void HomeAll();
extern void LEDRGB(long hexValue);
extern int volLevel;
extern void VolDown();
extern void VolUp();



//********** SWITCH 1
void switch1_Click() {
  LEDRGB(0x0000FF);
  RunMode = RunAuto;
}

void switch1_Doubleclick() {
  LEDRGB(0x00FF00);
  delay(1000);
  LEDRGB(0x000000);
  delay(1000);
  LEDRGB(0x00FF00);
  delay(1000);
  LEDRGB(0x000000);
  delay(1000);
  LEDRGB(0x00FF00);
  delay(1000);
  ResetServos();
  RunMode = RunMotion;
}

void switch1_LongPressStart() {
  LEDRGB(0xFF0000);
  bDance = false;
  RunMode = RunVoice;
 

}

//********** SWITCH 2
void switch2_Click() {
  VolDown();
  
}

void switch2_Doubleclick() {
 
}

void switch2_LongPressStart() {
 
 

}

//********** SWITCH 3
void switch3_Click() {
  VolUp();

}

void switch3_Doubleclick() {
  
}

void switch3_LongPressStart() {
 
 

}
