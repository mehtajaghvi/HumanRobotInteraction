/*
 Fade
 
 This example shows how to fade an LED on pin 9
 using the analogWrite() function.
 
 This example code is in the public domain.
 */


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
boolean testOver=0;
float timeElapsed;
float deadTime=0;
float userStartTime;
float userStopTime;
int userCount=0;
int th=2; //threshold
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
  if(brightness>th)
    effort=(effort+brightness);
   //fullBrightness()
  //if user started, then start the experiment
  if (userDone==0){
     congruentCase();
    //incongruentCase();
    Serial.print(effort);Serial.print("\t"); Serial.println(brightness);
  
  }
  //if user is done cranking, record the data
  if (userDone==1 && testOver==1){
    float userTime=userStopTime-userStartTime; 
    Serial.print("Test is over:  "); 
    Serial.print("USER ID  ");Serial.print(userCount);Serial.print("\t"); 
    Serial.print(effort);Serial.print("\t"); 
    Serial.println(userTime);
    testOver=0;
    delay(5000);
    userDone=0;
    startC=1;
    effort=0;
    noipFlag=1;
    userStarted=0;
  }
                           
}


void  congruentCase(){
  if(startC==1){
    Serial.println("User is starting the test");
    userStartTime=millis();
    fullBrightness(); //start with all led's red 
    startC=0;
  }
  
  //check if user started to crank
  if(effort>5){
  userStarted=1;
  
  }
  
  
  if(userStarted==1 && noipFlag==1 && brightness<=th){
      //Serial.println("starting timer ");
      timeElapsed=millis();//start timer if they are not cranking
      noipFlag=0;
      
   }
   

  if(userStarted==1 && noipFlag==0 && brightness>th){
      Serial.println("restart");
      timeElapsed=millis();//start timer if they are not cranking
      noipFlag=1;
      
  }
  
  deadTime=millis()-timeElapsed;  
  
  if(userStarted==1 && brightness<=th && deadTime>5000){
    Serial.println("I am done");
    userDone=1;//done with the test
    testOver=1;//test is done, now log the data in the loop
    userStopTime=millis();
    userCount=userCount+1;//user number
  }
  

  if(effort<=2000){
    
    analogWrite(red1,noBrightness);
    analogWrite(green1,ledBrightness);
    
    
  }
  else if(effort<30000){
       
    if(brightness >th){
      
      if (millis() >> 8 & 1){
      analogWrite(green2,ledBrightness);
      delay(50);
      }
      else{
      analogWrite(green2,noBrightness);
      delay(50);
      }
      analogWrite(red2,noBrightness);  
    }
    //if user stops go back to red and stop blinking
    else if(brightness<=th){
      analogWrite(green2,noBrightness);
      analogWrite(red2,ledBrightness);
    }
    
  }
  else if(effort<100000){
    //LED 3 starts
    analogWrite(green2,ledBrightness);
    analogWrite(red2,noBrightness);
    
    if(brightness >th){
      if (millis() >> 8 & 1){
      analogWrite(green3,ledBrightness);
      delay(50);
      }
      else{
      analogWrite(green3,noBrightness);
      delay(50);
      }
      analogWrite(red3,noBrightness);
    }
    
    else if(brightness<=th){
      analogWrite(green3,noBrightness);
      analogWrite(red3,ledBrightness);
    }
   
  }
  
  else if(effort<150000){
    //LED 4 starts 
    analogWrite(green3,ledBrightness);
    analogWrite(red3,noBrightness);
        
    if(brightness >th){
      if (millis() >> 8 & 1){
      analogWrite(green4,ledBrightness);
      delay(50);
      }
      else{
      analogWrite(green4,noBrightness);
      delay(50);
      }
      analogWrite(red4,noBrightness);
      }
    else if(brightness<=th){
      analogWrite(green4,noBrightness);
      analogWrite(red4,ledBrightness);
    }
    
  }
}

//////Incongruent Case//////
  
void incongruentCase(){
    if(startC==1){
    Serial.println("User is starting the test");
    userStartTime=millis();
    fullICBrightness(); //start with all led's red 
    startC=0;
  }
  
  //check if user started to crank
  if(effort>5){
  userStarted=1;
  }
    
  if(userStarted==1 && noipFlag==1 && brightness<=th){
      //Serial.println("starting timer ");
      timeElapsed=millis();//start timer if they are not cranking
      noipFlag=0;
      
   }
  
  if(userStarted==1 && noipFlag==0 && brightness>th){
      Serial.println("restart");
      timeElapsed=millis();//start timer if they are not cranking
      noipFlag=1;
      
  }
  
  deadTime=millis()-timeElapsed;  
  
  if(userStarted==1 && brightness<=th && deadTime>5000){
    Serial.println("I am done");
    userDone=1;//done with the test
    testOver=1;//test is done, now log the data in the loop
    userStopTime=millis();
    userCount=userCount+1;//user number
  }

 if(effort>1000 ){
       
    if(brightness >th){
      
      if (millis() >> 8 & 1){
      analogWrite(green4,ledBrightness);
      delay(50);
      }
      else{
      analogWrite(green4,noBrightness);
      delay(50);
      }
      analogWrite(red4,noBrightness);  
    }
    //if user stops go back to red and stop blinking
    else if(brightness<=th){
      analogWrite(green4,noBrightness);
      analogWrite(red4,ledBrightness);
    }
    
  }  
   
}

//all LED's red
void fullBrightness(){
   //green red red red
   analogWrite(green1,ledBrightness);
   analogWrite(red2,ledBrightness);
   analogWrite(red3,ledBrightness);
   analogWrite(red4,ledBrightness);
   
   analogWrite(green2,noBrightness);
   analogWrite(green3,noBrightness);
   analogWrite(green4,noBrightness);
  

}


void fullICBrightness(){
   //green red red red
   analogWrite(green1,ledBrightness);
   analogWrite(green2,ledBrightness);
   analogWrite(green3,ledBrightness);
   analogWrite(red4,ledBrightness);
   
   analogWrite(red1,noBrightness); 
   analogWrite(red2,noBrightness);
   analogWrite(red3,noBrightness);
  
  

}


