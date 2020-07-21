// rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
#define USE_USBCON //dueで通信するときに必要
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

//--------ros設定--------------
ros::NodeHandle node;
std_msgs::Float32 yaw_sub_value;

//---ブラシレスモーター設定---------
Servo brushlessmotor1;
Servo brushlessmotor2;
Servo brushlessmotor3;
Servo brushlessmotor4;
//lateral rotor
Servo lateral1_brushlessmotor;
Servo lateral2_brushlessmotor;
//lateral Servo
Servo lateral1_servo1;
Servo lateral1_servo2;
Servo lateral1_servo3;
Servo lateral2_servo1;
Servo lateral2_servo2;
Servo lateral2_servo3;

const float standard_throttle = 1700.0; //機体の自重分のスロットル
const float lateral_standard_throttle = 1800.0;
int brushless1_command;
int brushless2_command;
int brushless3_command;
int brushless4_command;
int lateral_brushless5_command;
int lateral_brushless6_command;
int lateral1_servo1_command;
int lateral1_servo2_command;
int lateral1_servo3_command;
int lateral2_servo1_command;
int lateral2_servo2_command;
int lateral2_servo3_command;

//サーボのコマンドを1100から1900に制限
int limit_servo_command_value(float value)
{
	float limit_max = 1900.0;
	float limit_min = 1100.0;
	if (value <= limit_min)
	{
		return (int)limit_min;
	}
	else if (value >= limit_max)
	{
		return (int)limit_max;
	}
	else
	{
		return (int)value;
	}
}

//コールバック関数
//自重分のスロットル+yaw制御分にリミットをかけモータに出力
void yawCallback(const std_msgs::Float32 &command_value)
{
	brushless1_command = limit_servo_command_value(standard_throttle);
	brushless2_command = limit_servo_command_value(standard_throttle);
	brushless3_command = limit_servo_command_value(standard_throttle);
	brushless4_command = limit_servo_command_value(standard_throttle);
	lateral_brushless5_command = limit_servo_command_value(lateral_standard_throttle);
	lateral_brushless6_command = limit_servo_command_value(lateral_standard_throttle);

	brushlessmotor1.writeMicroseconds(brushless1_command);
	brushlessmotor2.writeMicroseconds(brushless2_command);
	brushlessmotor3.writeMicroseconds(brushless3_command);
	brushlessmotor4.writeMicroseconds(brushless4_command);
	lateral1_brushlessmotor.writeMicroseconds(lateral_brushless5_command);
	lateral2_brushlessmotor.writeMicroseconds(lateral_brushless6_command);
}

ros::Subscriber<std_msgs::Float32> sub("yaw_command", &yawCallback);

void setup()
{
	node.initNode();
	node.subscribe(sub);
	int start_time = millis();
	//-----------------------------------
	brushlessmotor1.attach(10);
	brushlessmotor2.attach(11);
	brushlessmotor3.attach(12);
	brushlessmotor4.attach(13);
	lateral1_brushlessmotor.attach(8);
	lateral2_brushlessmotor.attach(9);
	lateral1_servo1.attach(2);
	lateral1_servo2.attach(3);
	lateral1_servo3.attach(4);
	lateral2_servo1.attach(5);
	lateral2_servo2.attach(6);
	lateral2_servo3.attach(7);
	while (start_time + 10000 < millis())
	{
		brushlessmotor1.writeMicroseconds(1000);
		brushlessmotor2.writeMicroseconds(1000);
		brushlessmotor3.writeMicroseconds(1000);
		brushlessmotor4.writeMicroseconds(1000);
		lateral1_brushlessmotor.writeMicroseconds(1000);
		lateral2_brushlessmotor.writeMicroseconds(1000);
	}
	//-----------------------------------
}

void loop()
{
	node.spinOnce();
	delayMicroseconds(500);
}