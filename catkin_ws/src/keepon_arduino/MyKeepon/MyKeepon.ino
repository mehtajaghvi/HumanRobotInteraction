/* My Keepon Arduino controller
   Copyright © 2012 BeatBots LLC

   This program is :qfree software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

   For a copy of the GNU General Public License, see
   http://www.gnu.org/licenses/gpl.html

   For support, please post issues on the Github project page at
   http://github.com/BeatBots/MyKeepon

   Learn more about My Keepon at
   http://mykeepon.beatbots.net

   Keepon® is a trademark of BeatBots LLC.
   
   Modified by Roger Nasci for HRI Fall 2015
   Version 1: added ros interface for serial communication
              removed query for Keepon feedback
   Version 2: incorrect version uploaded to GIT
              
*/
#include <string.h>
#include <Wire.h>

#define cbi(sfr, bit) _SFR_BYTE(sfr) &= ~_BV(bit)
#define sbi(sfr, bit) _SFR_BYTE(sfr) |= _BV(bit)

#define MK_FREQ 49600L // Set clock to 50kHz (actualy 49.6kHz seems to work better)
#define SOUND (byte)0x52  // Sound controller (device ID 82).  Write to 0xA4, read from 0xA5.
#define BUTTON (byte)0x50 // Button controller (device ID 80). Write to 0xA0, read from 0xA1.
#define MOTOR (byte)0x55  // Motor controller (device ID 85).  Write to 0xAA, read from 0xAB.


/*****************************************************************************************************
ROS Interface
*****************************************************************************************************/
//ROS interface for serial
//#define SERIAL_BUFFER_SIZE 256
#include <ros.h>
#include <std_msgs/String.h>
ros::NodeHandle nh;
String command;

//Message callback 
void messageCB( const std_msgs::String& msg_)
{
    command = msg_.data;
}

ros::Subscriber<std_msgs::String>sub_("/keepon/command", messageCB);

std_msgs::String str_msg;
ros::Publisher msgPub_("/keepon/status", &str_msg);

//function for returning a message to ROS
void statusOut(const char* output)
{
  str_msg.data = output;
  msgPub_.publish( &str_msg );
  nh.spinOnce();
}

/*****************************************************************************************************
Setup
*****************************************************************************************************/
void setup()
{
  pinMode(SDA, OUTPUT); // Data wire on My Keepon
  pinMode(SCL, OUTPUT); // Clock wire on My Keepon
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  
  //setup ros nodes
  nh.initNode();
  nh.advertise(msgPub_);
  nh.subscribe(sub_);
  command = "NULL";
  nh.spinOnce();
  
  //set up wire i2c stuff
  delay(500);
  bootup();
}

/*****************************************************************************************************
Bootup
*****************************************************************************************************/
void bootup()
{
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  
  //comment this if not connected to keepon...
  statusOut("Waiting for My Keepon... ");
  while (analogRead(0) < 512); // Wait until we see voltage on A0 pin
  statusOut("My Keepon detected.");
  
  delay(1000);
  Wire.begin();
  TWBR = ((F_CPU / MK_FREQ) - 16) / 2;
  //Serial.write((byte)0);
}

/*****************************************************************************************************
Parsing message support functions
*****************************************************************************************************/
boolean isNextWord(char* msg, char* wordToCompare, int* i) {
  int j = 0;
  while (msg[j] == wordToCompare[j++]);
  if (j >= strlen(wordToCompare)) {
    *i = *i+j;
    return 1;
  } 
  else return 0;
}

int nextInt(char* msg) {
  int j = 0;
  int value = 0;
  int negative = 0;
  while (!(isDigit(msg[j]) || msg[j]=='-')) j++;
  if (msg[j] == '-') {
    negative = 1;
    j++;
  }
  while (isDigit(msg[j])) {
    value *= 10;
    value += msg[j++]-'0';
  }
  if (negative) value *= -1;
  return value;
}


/*****************************************************************************************************
Parse Message
*****************************************************************************************************/

boolean parseMsg(char* msg, byte* cmd, byte* device) {
  int i = 0, value;

  if (isNextWord(&msg[i], "SOUND", &i)) {
    *device = SOUND;
    if (isNextWord(&msg[i], "PLAY", &i)) {
      cmd[0] = 0x01;
      cmd[1] = B10000000 | (63 & nextInt(&msg[i]));
    } 
    else if (isNextWord(&msg[i], "REPEAT", &i)) {
      cmd[0] = 0x01;
      cmd[1] = B11000000 | (63 & nextInt(&msg[i]));
    } 
    else if (isNextWord(&msg[i], "DELAY", &i)) {
      cmd[0] = 0x03;
      cmd[1] = (byte)nextInt(&msg[i]);
    } 
    else if (isNextWord(&msg[i], "STOP", &i)) {
      cmd[0] = 0x01;
      cmd[1] = B00000000;
    } 
    else {
      //Serial.println("Unknown command.");
      return false;
    }
  } 
  else if (isNextWord(&msg[i], "SPEED", &i)) {
    *device = MOTOR;
    if (isNextWord(&msg[i], "PAN", &i)) {
      cmd[0] = 5;
      cmd[1] = (byte)nextInt(&msg[i]);
    } 
    else if (isNextWord(&msg[i], "TILT", &i)) {
      cmd[0] = 3;
      cmd[1] = (byte)nextInt(&msg[i]);
    } 
    else if (isNextWord(&msg[i], "PONSIDE", &i)) {
      cmd[0] = 1;
      cmd[1] = (byte)nextInt(&msg[i]);
    } 
    else {
      //Serial.println("Unknown command.");
      return false;
    }
  } 
  else if (isNextWord(&msg[i], "MOVE", &i)) {
    *device = MOTOR;
    if (isNextWord(&msg[i], "PAN", &i)) {
      cmd[0] = 4;
      cmd[1] = (byte)(nextInt(&msg[i]) + 127);
    } 
    else if (isNextWord(&msg[i], "TILT", &i)) {
      cmd[0] = 2;
      cmd[1] = (byte)(nextInt(&msg[i]) + 127);
    } 
    else if (isNextWord(&msg[i], "SIDE", &i)) {
      cmd[0] = 0;
      if (isNextWord(&msg[i], "CYCLE", &i))
        cmd[1] = 0;
      else if (isNextWord(&msg[i], "CENTERFROMLEFT", &i))
        cmd[1] = 1;
      else if (isNextWord(&msg[i], "RIGHT", &i))
        cmd[1] = 2;
      else if (isNextWord(&msg[i], "CENTERFROMRIGHT", &i))
        cmd[1] = 3;
      else if (isNextWord(&msg[i], "LEFT", &i))
        cmd[1] = 4;
      else {
        //Serial.println("Unknown command.");
        return false;
      }
    } 
    else if (isNextWord(&msg[i], "PON", &i)) {
      cmd[0] = 0;
      if (isNextWord(&msg[i], "UP", &i))
        cmd[1] = -1;
      else if (isNextWord(&msg[i], "HALFDOWN", &i))
        cmd[1] = -2;
      else if (isNextWord(&msg[i], "DOWN", &i))
        cmd[1] = -3;
      else if (isNextWord(&msg[i], "HALFUP", &i))
        cmd[1] = -4;
      else {
        //Serial.println("Unknown command.");
        return false;
      }
    } 
    else if (isNextWord(&msg[i], "STOP", &i)) {
      cmd[0] = 6;
      cmd[1] = 16;
      //statusOut("Stopping Motion!");
    } 
    else {
      //Serial.println("Unknown command.");
      return false;
    }    
  } 
  else if (isNextWord(&msg[i], "MODE", &i)) {
    if (isNextWord(&msg[i], "DANCE", &i)) {
      cmd[0] = 6;
      cmd[1] = 0;
    } 
    else if (isNextWord(&msg[i], "TOUCH", &i)) {
      cmd[0] = 6;
      cmd[1] = 1;
    } 
    else if (isNextWord(&msg[i], "TEMPO", &i)) {
      cmd[0] = 6;
      cmd[1] = 2;
    } 
    else if (isNextWord(&msg[i], "SLEEP", &i)) {
      cmd[0] = 6;
      cmd[1] = 240;
    } 
    else {
      //Serial.println("Unknown command.");
      return false;
    }
  } 
  else {
    //Serial.println("Unknown command.");
    statusOut("Unknown Command Recieved");
    return false;
  }
  return true;
}


/*****************************************************************************************************
Main Loop:
*****************************************************************************************************/
/*look for new messages, if there is one parse it and transmit the instruction to keepon
*/
void loop() {
  char msg[32];
  byte device, cmd[2];
  

   nh.spinOnce();
   if (command != "NULL") {
    int commandLength = command.length()+1;
    char msg[commandLength];
    command.toCharArray(msg, sizeof(msg) );
    statusOut(msg);
    
    if (parseMsg(msg, cmd, &device)) {
      int result = 1;
      int attempts = 0;
      //comment this if not connected to Keepon!!!
      
      while (result != 0 && attempts++ < 50) {
        Wire.beginTransmission(device);
        Wire.write((byte)cmd[0]);
        Wire.write((byte)cmd[1]);
        result = (int)Wire.endTransmission();
      }
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
      //delay(250);
      
    
    }
    command = "NULL";
  }
}

