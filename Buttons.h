/*
   buttons.h

   Created: March 8, 2020
   Author: Andrew Donatelli andrew@donatelli.net
   

*/


#include <Arduino.h>
#ifndef Buttons_h
#define Buttons_h

#define RunAuto 1
#define RunMotion 2
#define RunVoice 3


// ***** button callback functions


void switch1_Click();
void switch1_Doubleclick();
void switch1_LongPressStart();

void switch2_Click();
void switch2_Doubleclick();
void switch2_LongPressStart();

void switch3_Click();
void switch3_Doubleclick();
void switch3_LongPressStart();



#endif // Buttons_h
