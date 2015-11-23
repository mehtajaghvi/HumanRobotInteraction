/*
joy_keepon_translator_node
Roger Nasci
HRI Fall 2015

based on http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

Read joystick input from joy message, convert to a keepon 
command and publish that

Version 0: intial build
Version 1: rnasci: added semicolon to end of command line.
*/



#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <cmath>
#include <stdio.h>
#include <string.h>


class TeleopKeepon
{
public:
	TeleopKeepon();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	ros::NodeHandle nh_;
	
	ros::Publisher cmd_pub_;
	ros::Subscriber joy_sub_;

	int speed; //stores speed setpoint
	int speedStep; //speed change with dipad increment/ decrement

	std::string ponPositions[4];
	int ponState;
	
};

TeleopKeepon::TeleopKeepon()
{
	cmd_pub_ = nh_.advertise<std_msgs::String>("keepon/command", 50);	
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopKeepon::joyCallback, this);

	//initialize values
	speed = 150;
	speedStep = 10; //speed change with dipad increment/ decrement

	ponPositions[3] = "UP";
	ponPositions[2] = "HALFUP";
	ponPositions[1] = "HALFDOWN";
	ponPositions[0] = "DOWN";
	ponState = 0;
}

/*
called when message from joystick.
gets action associated with each button/ control
sends action for each action state noted.

Concern! flooding with fast messages?
*/
void TeleopKeepon::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	std_msgs::String command_Out;

	//get button states
	int buttonA = joy->buttons[0];
	int buttonB = joy->buttons[1];
	int buttonX = joy->buttons[2];
	int buttonY = joy->buttons[3];
	int rb = joy->buttons[5]; //pon increase
	int lb = joy->buttons[4]; //pon decrease

	float rightJoyLateral = joy->axes[3]; //pan
	float rightJoyVertical = joy->axes[4]; //tilt

	int diPadVertical = joy->axes[7]; //set speed up or down by increments
	int diPadHorizontal = joy->axes[6];

	int leftJoyVertical = joy->axes[1];
	int leftJoyLateral = joy->axes[0];

	float leftTrigger = joy->axes[2];
	float rightTrigger = joy->axes[5];


	//Stop
	if (buttonA)
	{
		command_Out.data = "MOVE STOP";
		cmd_pub_.publish(command_Out);
	}

	//set speed 
	if (abs(diPadVertical))
	{
		speed = speed + speedStep * diPadVertical;
		if (speed < 10) speed = 10;
		if (speed > 250) speed = 250;
		char temp[3];
		sprintf(temp, "%i", speed);

		std::string name;
		name = "SPEED PAN ";
		command_Out.data = name + temp;
		cmd_pub_.publish(command_Out);

		name = "SPEED TILT ";
		command_Out.data = name + temp;
		cmd_pub_.publish(command_Out);

		name = "SPEED PONSIDE ";
		command_Out.data = name + temp;
		cmd_pub_.publish(command_Out);
	}

	//set tilt
	if (rightJoyVertical > 0.001 || rightJoyVertical < -0.001) //0.01 for buffer for reset to 0. abs does int
	{
		std::string cmd;
		cmd = "MOVE TILT ";

		int value = rightJoyVertical*100;

		//invert controls
		value = value * 1;

		char temp[4];
		sprintf(temp, "%i", value);
		command_Out.data = cmd + temp;		
		cmd_pub_.publish(command_Out);
	}

	//set pan
	if (rightJoyLateral > 0.001 || rightJoyLateral < -0.001) //0.01 for buffer for reset to 0. abs does int
	{
		std::string cmd;
		cmd = "MOVE PAN ";

		int value = rightJoyLateral*100;

		//invert controls
		value = value * -1;

		char temp[4];
		sprintf(temp, "%i", value);
		command_Out.data = cmd + temp;		
		cmd_pub_.publish(command_Out);

	}

	//side cycle
	if (buttonY)
	{
		std::string cmd;
		cmd = "MOVE SIDE CYCLE";
		command_Out.data = cmd;
		cmd_pub_.publish(command_Out);
	}


	//change PON (vertical) state
	if(rb)
	{
		/*
		if (ponState < 3)
		{
			ponState ++;
			std::string cmd = "MOVE PON ";
			command_Out.data = cmd + ponPositions[ponState];
			cmd_pub_.publish(command_Out);
		}
		*/
		std::string cmd = "MOVE PON UP";
		command_Out.data = cmd;
		cmd_pub_.publish(command_Out);
	}
	if(lb)
	{
		/*
		if (ponState > 0)
		{
			ponState --;
			std::string cmd = "MOVE PON ";
			command_Out.data = cmd + ponPositions[ponState];
			cmd_pub_.publish(command_Out);
		}
		*/
		std::string cmd = "MOVE PON DOWN";
		command_Out.data = cmd;
		cmd_pub_.publish(command_Out);
	}
	

}
	
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_keepon");
  TeleopKeepon teleop_keepon;

  ros::spin();
}
