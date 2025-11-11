/***************************************************
DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/product-1121.html>
 
 ***************************************************
 This example shows the basic function of library for DFPlayer.
 
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

/*
 * Sounds are as follows, must be loaded on the card in this order. 
1 battey charged - start
2 I amDO - default
3 - 5 greetings
6 - 9 Negative
1- - 14 Positive Random
15-20 squeeky Wheel
*/

#include "Arduino.h"
//#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//SoftwareSerial mySoftwareSerial (7, 8); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

//Variables for reading the RC switches
int readsw1;
int readsw2;
int readsw3;
int readsw4;

int pin1 = 11;
int pin2 = 10;
int pin3 = 12;
int pin4 = 9;

int currentsound = 2;
int soundstate =1;
int prevsoundstate =1;


void setup()
{
 
  Serial.begin(9600);
  
  if (!myDFPlayer.begin(Serial)) {  //Use softwareSerial to communicate with mp3.
    //Serial.println(F("Unable to begin:"));
    //Serial.println(F("1.Please recheck the connection!"));
    //Serial.println(F("2.Please insert the SD card!"));
//    while(true){
//      delay(0); // Code to compatible with ESP8266 watch dog.
//    }
  }
  //Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  delay (1000);
  myDFPlayer.play(1);  //Play the first mp3
}

void loop()
{
  static unsigned long timer = millis();

  readsw1 = pulseIn (pin1, HIGH);
  readsw2 = pulseIn (pin2, HIGH);
  readsw3 = pulseIn (pin3,HIGH);
  readsw4 = pulseIn (pin4,HIGH);

  if (readsw1< 1500){
    soundstate=1;
  }

   if (readsw1> 1500){
    soundstate=2;
  }

  if (soundstate != prevsoundstate){
    prevsoundstate = soundstate;
    myDFPlayer.play(currentsound);  //Play the first mp3
    
  }

  if (readsw2 > 1500){
    currentsound = random (3,6);
  }

  if (readsw2 < 1500){
    currentsound = 2;
  }

  if (readsw3 > 1300){
    currentsound = random (6,10);
  }
  if (readsw3 > 1800){
    currentsound = random (10,15);
  }

  if (readsw4 > 1500){
    currentsound = random (15,21);
    ;
  }

  

  


//  Serial.print (readsw1);
//  Serial.print (" ");
//   Serial.print (readsw2);
//  Serial.print (" ");
//   Serial.print (readsw3);
//  Serial.print (" ");
//   Serial.print (readsw4);
//  Serial.println (" ");
//  
  
  
  
  
  
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      //Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      //Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      //Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      //Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      //Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      //Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      //Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      //Serial.print(F("Number:"));
      //Serial.print(value);
      //Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      //Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          //Serial.println(F("Card not found"));
          break;
        case Sleeping:
         // Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
         //Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          //Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          //Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
         // Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          //Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}
