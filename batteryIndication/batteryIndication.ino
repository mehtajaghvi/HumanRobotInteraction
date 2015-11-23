/*
 Fade
 
 This example shows how to fade an LED on pin 9
 using the analogWrite() function.
 
 This example code is in the public domain.
 */
import processing.serial.*;
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;
String command;

//crank variables


int analogPinCrank=0;//read the crank
float brightness=0;
int ledBrightness=255;
int noBrightness=0;
//int threshold=170;//threshold for crank input

//publish over ROS
float effort=0;//effort put in cranking
int timeEffort=0;

//LED1
int green1 = 53;// the pin that the LED is attached to
int red1=52;

//LED2
int green2 = 51;// the pin that the LED is attached to
int red2=50;

//LED3
int green3 = 49;// the pin that the LED is attached to
int red3=48;

//LED4
int green4 = 47;// the pin that the LED is attached to
int red4=46;

//congruent Variables
boolean startC=1;
boolean userStarted=0;
boolean noipFlag=1;
boolean userDone=0;
boolean testOver=1;
float timeElapsed;
float deadTime=0;
float userStartTime;
float userStopTime;
// the setup routine runs once when you press reset:
void setup()  { 

  Serial.begin(9600);
  
  pinMode(red1, OUTPUT);
  pinMode(green1, OUTPUT);  // declare pin 9 to be an output:
    
  pinMode(red2, OUTPUT);
  pinMode(green2, OUTPUT);  // declare pin 9 to be an output:
    
  pinMode(red3, OUTPUT);
  pinMode(green3, OUTPUT);  // declare pin 9 to be an output:
    
  pinMode(red4, OUTPUT);
  pinMode(green4, OUTPUT);  // declare pin 9 to be an output:
 
} 

// the loop routine runs over and over again forever:
void loop()  { 
  
  //read the crank value and scale it to ledBrightness
  brightness = map(analogRead(analogPinCrank), 0, 1023, 0, 255);
  effort=(effort+brightness);

  //if user started, then start the experiment
  if (userDone==0){
    congruentCase();
    Serial.print(effort);Serial.print("\t"); Serial.println(brightness);
  
  }
  //if user is done cranking, record the data
  if (userDone==1 && testOver==1){
    float userTime=userStopTime-userStartTime; 
    Serial.print("Test is over:  "); 
    Serial.print(effort);Serial.print("\t"); 
    Serial.println(userTime);
    testOver=0;
  }
                           
}


void  congruentCase(){
  if(startC==1){
    userStartTime=millis();
    fullBrightness(); //start with all led's red 
    startC=0;
  }
  
  //check if user started to crank
  if(effort>10){
  userStarted=1;
  
  }
  
  if(userStarted==1 && brightness==0 && noipFlag==1){
    timeElapsed=millis();
    //Serial.println("timer started............");
    noipFlag=0;
    //Serial.println(timeElapsed);
    }
    
  deadTime=millis()-timeElapsed;
  if(userStarted==1 && brightness==0 && deadTime>5000){
    userDone=1;
    userStopTime=millis();
  }
  
  if(effort<=20000){
    
    analogWrite(red1,noBrightness);
    analogWrite(green1,ledBrightness);
    delay(50);
    analogWrite(green1,noBrightness);
   // delay(50);
   
    
  }
  else if(effort<40000){
    analogWrite(green1,ledBrightness);
    analogWrite(green2,ledBrightness);
    analogWrite(green2,noBrightness);
    analogWrite(green2,ledBrightness);
    analogWrite(red2,noBrightness);
    
  }
  else if(effort<80000){
   
    analogWrite(red3,noBrightness);
    analogWrite(green3,ledBrightness);
  }
  else if(effort<100000){
  
    analogWrite(red4,noBrightness);
    analogWrite(green4,ledBrightness);
  }
}


  
  



void incongruentCase(){
   analogWrite(red1,ledBrightness);
 // analogWrite(red2,brightness);
 // analogWrite(red3,brightness);
 // analogWrite(red4,brightness);

}

//all LED's red
void fullBrightness(){
   analogWrite(green1,ledBrightness);
   analogWrite(red2,ledBrightness);
   analogWrite(red3,ledBrightness);
   analogWrite(red4,ledBrightness);

}


