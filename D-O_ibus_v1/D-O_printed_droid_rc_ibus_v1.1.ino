///////////////////////////////////////////////////////////////////////////////////////////////////////
// DO v2 iBus Sketch v1.1
// Sketch written by Reinhard Stockinger 2020/11
// Sketch is for the Printed Droid D-O Control PCB developed by Nitewing
// Latest skecth can always be found on www.printed-droid.com
///////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <IBusBM.h>
#include <Servo.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define IBUS_ENABLED
//#define MAINBAR_CORRECTION
#define DFPLAYER_ENABLED
#define SERVOS_ENABLED

//Cytron Motor Controller
int dir1pin =13; //Motor Direction pin (goes to DIR1)
int spe1pin =12; //Motor Speed pin (goes to PWM1)
int dir2pin =11; //Motor Direction pin (goes to DIR2)
int spe2pin =10; //Motor Speed pin (goes to PWM2)
int mspeed = 10; 
int turnspeed=50; 

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=20;
float pid_i=20;
float pid_d=0;
////////////////////////PID CONSTANST/////////////////////
float kp=25;
float ki=0;
float kd=0.8;
float desired_angle = -0.3;//////////////TARGET ANGLE///////////// measure the default angle and change this value

int pin1 = 3;  // This is input for RC (tank mixed) drive 1
int pin2 = 4;  // This is input for RC (tank mixed) drive 2

int duration1 = 1500; // Duration of the pulse from the RC
int duration2 = 1500; // Duration of the pulse from the RC

int motorspeed1 = 0;
int motordirection1 = HIGH;
int motorspeed2 = 0 ;
int motordirection2 = HIGH;

//error correction for mainbar!
int centreangle = -10; //offset, to get this figure, unrem the serial print angle (0)
int actualangle = 0; //actual angle for mapping main bar
int angleerror = 0; //this is the mainbar correction
int anglestrength =40; // this changes the severity of the adjustment (lower the number, the more the head corrects)


#ifdef IBUS_ENABLED
  IBusBM IBus; // IBus object
#endif

#ifdef SERVOS_ENABLED
  //Pins for the servos
  int pin_mainbarServo = 0;
  int pin_head1Servo = 1;
  int pin_head2Servo = 5;
  int pin_head3Servo = 6;
  
  Servo mainbarServo;
  Servo head1Servo;
  Servo head2Servo;
  Servo head3Servo;
#endif

//DF-Player defines
#ifdef DFPLAYER_ENABLED
  SoftwareSerial mySoftwareSerial (7, 8); // RX, TX
  DFRobotDFPlayerMini myDFPlayer;
  
  void printDetail(uint8_t type, int value);
  void handleDFPlayer();

  //Variables for reading the RC switches
  int readsw1;
  int readsw2;
  int readsw3;
  int readsw4;
  
  int dfpin1 = 14;
  int dfpin2 = 15;
  int dfpin3 = 16;
  int dfpin4 = 17;
  
  int currentsound = 2;
  int soundstate =1;
  int prevsoundstate =1;
#endif

void setup() 
{
  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  ////////////////PIN MODE DEFINATIONS//////////////////////
  pinMode(dir1pin,OUTPUT);
  pinMode(spe1pin,OUTPUT);
  pinMode(dir2pin,OUTPUT);
  pinMode(spe2pin,OUTPUT);
  
  //Serial.begin(9600); // ** uncomment to allow debugging**

#ifdef SERVOS_ENABLED
  //Enable Servos
  mainbarServo.attach(pin_mainbarServo);
  mainbarServo.writeMicroseconds(1500); // set Mainbar to mid-point
  head1Servo.attach(pin_head1Servo);
  head1Servo.writeMicroseconds(1500); // center head 1 servo
  head2Servo.attach(pin_head2Servo);
  head2Servo.writeMicroseconds(1500); // center head 2 servo
  head3Servo.attach(pin_head3Servo);
  head3Servo.writeMicroseconds(1500); // center head 3 servo
#endif

#ifdef IBUS_ENABLED
  Serial1.begin(9600); //RX1 pin 19
  IBus.begin(Serial1, IBUSBM_NOTIMER);
#endif  

#ifdef DFPLAYER_ENABLED
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    //Serial.println(F("Unable to begin:"));
    //Serial.println(F("1.Please recheck the connection!"));
    //Serial.println(F("2.Please insert the SD card!"));
    //while(true){
    //  delay(0); // Code to compatible with ESP8266 watch dog.
    //}
  }
  //Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  delay (1000);
  myDFPlayer.play(1);  //Play the first mp3
#endif

  time = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////

#ifdef IBUS_ENABLED
  IBus.loop();
  duration1 = IBus.readChannel(0); // Measure the input from rc drive 1
  duration2 = IBus.readChannel(1); // Measure the input from rc drive 1
#else
  duration1 = pulseIn(pin1, HIGH); //Measures the input from rc drive 1
  duration2 = pulseIn(pin2, HIGH); //Measures the input from rc drive 2
#endif

  //this loops until it get a signal from RC, nothing will run
  while ( duration1 >2200||duration1<800) {
#ifdef IBUS_ENABLED
    IBus.loop();
    duration1 = IBus.readChannel(0);
    delay (300);
#else    
    duration1 = pulseIn(pin1, HIGH); //Measures the input from rc drive 1  
#endif
  }
  //this loops until it get a signal from RC, nothing will run
  while ( duration2 >2200||duration2<800) {                               
#ifdef IBUS_ENABLED
    IBus.loop();
    duration2 = IBus.readChannel(1);
    delay (300);
#else
    duration2 = pulseIn(pin2, HIGH); //Measures the input from rc drive 1  
#endif
  }
}

void loop() 
{

#ifdef IBUS_ENABLED
  IBus.loop();
  duration1 = IBus.readChannel(0); // Measure the input from rc drive 1
  duration2 = IBus.readChannel(1); // Measure the input from rc drive 1
#else  
  duration1 = pulseIn(pin1, HIGH); //Measures the input from rc drive 1
  duration2 = pulseIn(pin2, HIGH); //Measures the input from rc drive 2
#endif  
  //Serial.print (duration1);
  //Serial.print (" ");
  //Serial.print (duration2);
  //Serial.print (" \n");

  motorspeed1 = map (duration1,1000,2000,-255,255); //Maps the duration to the motorspeed from the stick
  motorspeed2 = map (duration2,1000,2000,-255,255); //Maps the duration to the motorspeed from the stick
  //Serial.print (motorspeed1);
  //Serial.print (" ");
  //Serial.print (motorspeed2);
  //Serial.print (" ");

#ifdef SERVOS_ENABLED
#ifdef MAINBAR_CORRECTION
  mainbarServo.writeMicroseconds((IBus.readChannel(2)+angleerror));
#else
  mainbarServo.writeMicroseconds(IBus.readChannel(2));
#endif  
  head1Servo.writeMicroseconds(IBus.readChannel(3));
  head2Servo.writeMicroseconds(IBus.readChannel(4));
  head3Servo.writeMicroseconds(IBus.readChannel(5));
#endif

#ifdef DFPLAYER_ENABLED
  handleDFPlayer();
#endif
  
  /*////////////////////////WARNING//////////////////////
   * DO NOT USE ANY DELAYS INSIDE THE LOOP OTHERWISE THE BOT WON'T BE 
   * ABLE TO CORRECT THE BALANCE FAST ENOUGH
   * ALSO, DONT USE ANY SERIAL PRINTS. BASICALLY DONT SLOW DOWN THE LOOP SPEED.
  */
  timePrev = time;  
  time = millis();  
  elapsedTime = (time - timePrev) / 1000; 
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  ////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU///////////////// 
  Acc_rawX=Wire.read()<<8|Wire.read(); 
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read(); 
  /////////////////////CONVERTING RAW DATA TO ANGLES/////////////////////
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  Wire.beginTransmission(0x68);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); 
  //////////////////PULLING RAW GYRO DATA FROM IMU/////////////////////////
  Gyr_rawX=Wire.read()<<8|Wire.read(); 
  Gyr_rawY=Wire.read()<<8|Wire.read(); 
  //Serial.print("rawx ");
  //Serial.print(Gyr_rawX);
  //Serial.print("rawy ");
  //Serial.print(Gyr_rawY);
  //Serial.print("\n");
  ////////////////////CONVERTING RAW DATA TO ANGLES///////////////////////
  Gyro_angle[0] = Gyr_rawX/131.0; 
  Gyro_angle[1] = Gyr_rawY/131.0;
  //////////////////////////////COMBINING BOTH ANGLES USING COMPLIMENTARY FILTER////////////////////////
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
  ////TOTAL_ANGLE[0] IS THE PITCH ANGLE WHICH WE NEED////////////
  error = Total_angle[0] - desired_angle; /////////////////ERROR CALCULATION////////////////////
  ///////////////////////PROPORTIONAL ERROR//////////////
  pid_p = kp*error;
  ///////////////////////INTERGRAL ERROR/////////////////
  pid_i = pid_i+(ki*error);  
  ///////////////////////DIFFERENTIAL ERROR//////////////
  pid_d = kd*((error - previous_error)/elapsedTime);
  ///////////////////////TOTAL PID VALUE/////////////////
  PID = pid_p + pid_d;
  ///////////////////////UPDATING THE ERROR VALUE////////
  previous_error = error;
  //Serial.println(PID);                     //////////UNCOMMENT FOR DDEBUGGING//////////////
  //delay(60);                               //////////UNCOMMENT FOR DDEBUGGING//////////////
  //Serial.println(Total_angle[0]);          //////////UNCOMMENT FOR DDEBUGGING//////////////
  /////////////////CONVERTING PID VALUES TO ABSOLUTE VALUES//////////////////////////////////
  mspeed = abs(PID);

  actualangle=Total_angle[0] - centreangle;
  angleerror=map(actualangle,anglestrength,-anglestrength,1000,2000)-1500;
  //Serial.println(angleerror);

  mspeed=map(mspeed,0,2100,0,700); // ** The last number  mspeed=map(mspeed,0,2100,0,700) - 700, can be changed to increase or decrease the "harshness" of the speed compensation when balancing. 
    
  //Serial.println(mspeed);                  //////////UNCOMMENT FOR DDEBUGGING//////////////
  ///////////////SELF EXPLANATORY///////////////
   
  if(Total_angle[0]<0+desired_angle)
  {
    mspeed=-mspeed;
  }
  if(Total_angle[0]>0+desired_angle)
  {
    mspeed = mspeed;
  }
    
  motorspeed1=motorspeed1+mspeed; //This add the RC drive to the correction drive, motorspeed is the rc, mspeed from the IMU
  motorspeed2=motorspeed2-mspeed; //This add the RC drive to the correction drive, motorspeed is the rc, mspeed from the IMU
  //Serial.println (mspeed);
  //Serial.print (" ");
  //Serial.print (mspeed);
  //Serial.print (" ");

  if (motorspeed1<0) {
    motordirection1 = LOW;
    motorspeed1=-motorspeed1;
  }
  else if (motorspeed1>0) {
    motordirection1 = HIGH;  
  }

  if (motorspeed2<0) {
    motordirection2 = LOW;
    motorspeed2=-motorspeed2;
  }
  else if (motorspeed2>0) {
    motordirection2 = HIGH;  
  }

  if (motorspeed1 >254){
    motorspeed1=255;
  }

  if (motorspeed2 >254){
    motorspeed2=255;
  }

  //Serial.print (motorspeed1);
  //Serial.print (" ");
  //Serial.print (motorspeed2);
  //Serial.println (" ");

  digitalWrite(dir1pin,motordirection1);
  analogWrite(spe1pin,motorspeed1); //increase the speed of the motor from 0 to 255
  digitalWrite(dir2pin,motordirection2);
  analogWrite(spe2pin,motorspeed2); //increase the speed of the motor from 0 to 255
}

/************************************************************************/

#ifdef DFPLAYER_ENABLED
void handleDFPlayer()
{
#ifdef IBUS_ENABLED
  IBus.loop();
  readsw1 = IBus.readChannel(6);
  readsw2 = IBus.readChannel(7);
  readsw3 = IBus.readChannel(8);
  readsw4 = IBus.readChannel(9);
#else
  readsw1 = pulseIn (dfpin1, HIGH);
  readsw2 = pulseIn (dfpin2, HIGH);
  readsw3 = pulseIn (dfpin3,HIGH);
  readsw4 = pulseIn (dfpin4,HIGH);
#endif

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
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

}

/************************************************************************/

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

#endif


       
