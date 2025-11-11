#include <Wire.h>

////////////////VARIABLE DEFINATION///////////////
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
  time = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////

duration1 = pulseIn(pin1, HIGH); //Measures the input from rc drive 1
duration2 = pulseIn(pin2, HIGH); //Measures the input from rc drive 2

while ( duration1 >2200||duration1<800) {
duration1 = pulseIn(pin1, HIGH); //Measures the input from rc drive 1  //this loops until it get a signal from RC, nothing will run
}
while ( duration2 >2200||duration2<800) {                               //this loops until it get a signal from RC, nothing will run
duration2 = pulseIn(pin2, HIGH); //Measures the input from rc drive 1  
}

  
}
void loop() 
{

duration1 = pulseIn(pin1, HIGH); //Measures the input from rc drive 1
duration2 = pulseIn(pin2, HIGH); //Measures the input from rc drive 2


//Serial.print (duration1);
//Serial.print (" ");
//Serial.print (duration2);
//Serial.print (" ");


motorspeed1 = map (duration1,1000,2000,-255,255); //Maps the duration to the motorspeed from the stick
motorspeed2 = map (duration2,1000,2000,-255,255); //Maps the duration to the motorspeed from the stick

//Serial.print (motorspeed1);
//Serial.print (" ");
//Serial.print (motorspeed2);
//Serial.print (" ");

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



       
