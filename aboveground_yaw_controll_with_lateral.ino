// rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
#define USE_USBCON //dueで通信するときに必要
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>

//--------ros設定--------------
ros::NodeHandle nh;

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

const float lateral_standard_throttle = 1800.0;
int brushless1_command;
int brushless2_command;
int brushless3_command;
int brushless4_command;
int lateral_brushless5_command;
int lateral_brushless6_command;
int lateral1_servo_command;
int lateral1_servo1_command;
int lateral1_servo2_command;
int lateral1_servo3_command;
int lateral2_servo_command;
int lateral2_servo1_command;
int lateral2_servo2_command;
int lateral2_servo3_command;

//サーボのコマンドを1900に制限
int limit_servo_command_value(float value)
{
	float limit_max = 1900.0;
	if (value >= limit_max)
	{
		return (int)limit_max;
	}
	else
	{
		return (int)value;
	}
}

//servo_command=lateral1_thrust_to_servo_command(thrust)--------------------------------------
const float lateral1_param1[4] = {1.42000573e+03, -1.27692693e+01, -4.62346562e-01, -2.79686070e-02};
const float lateral1_param2[4] = {1410.018664, -47.59048111, -7.52998927, 24.85048449};
const float lateral1_param3[4] = {1391.83293, -14.2056781, 0.863936724, -0.0510286803};
int lateral1_thrust_to_servo_command(float thrust)
{
	int command;
	float x1 = thrust;
	float x2 = thrust * x1;
	float x3 = thrust * x2;
	if (thrust < -0.7448)
	{
		command = int(lateral1_param1[0] + lateral1_param1[1] * x1 + lateral1_param1[2] * x2 + lateral1_param1[3] * x3);
	}
	else if (-0.7448 <= thrust && thrust <= 0.882)
	{
		command = int(lateral1_param2[0] + lateral1_param2[1] * x1 + lateral1_param2[2] * x2 + lateral1_param2[3] * x3);
	}
	else if (0.882 < thrust)
	{
		command = int(lateral1_param3[0] + lateral1_param3[1] * x1 + lateral1_param3[2] * x2 + lateral1_param3[3] * x3);
	}

	//limitter
	if (command > 1550)
	{
		command = 1550;
	}
	else if (command < 1230)
	{
		command = 1230;
	}
	return command;
}

const float lateral2_param1[4] = {1.33485718e+03, -1.26789886e+01, -5.94512376e-01, -3.30347612e-02};
const float lateral2_param2[4] = {1326.84944302, -62.67557326, 7.02881259, 106.85549092};
const float lateral2_param3[4] = {1.31021169e+03, -1.40550417e+01, 6.30551538e-01, -3.50974486e-02};
int lateral2_thrust_to_servo_command(float thrust)
{
	int command;
	thrust = -thrust;
	float x1 = thrust;
	float x2 = thrust * x1;
	float x3 = thrust * x2;
	if (thrust < -0.668)
	{
		command = int(lateral2_param1[0] + lateral2_param1[1] * x1 + lateral2_param1[2] * x2 + lateral2_param1[3] * x3);
	}
	else if (-0.668 <= thrust && thrust <= 0.393)
	{
		command = int(lateral2_param2[0] + lateral2_param2[1] * x1 + lateral2_param2[2] * x2 + lateral2_param2[3] * x3);
	}
	else if (0.393 < thrust)
	{
		command = int(lateral2_param3[0] + lateral2_param3[1] * x1 + lateral2_param3[2] * x2 + lateral2_param3[3] * x3);
	}

	//limmitter
	if (command > 1450)
	{
		command = 1450;
	}
	else if (command < 1180)
	{
		command = 1180;
	}
	return command;
}
//-------------------------------------------------------------------------------------

//コールバック関数
//自重分のスロットル+yaw制御分にリミットをかけモータに出力
void yawCallback(const std_msgs::Float32MultiArray &command_value)
{
	brushless1_command = limit_servo_command_value(command_value.data[0]);
	brushless2_command = limit_servo_command_value(command_value.data[0]);
	brushless3_command = limit_servo_command_value(command_value.data[0]);
	brushless4_command = limit_servo_command_value(command_value.data[0]);
	lateral_brushless5_command = limit_servo_command_value(command_value.data[1]);
	lateral_brushless6_command = limit_servo_command_value(command_value.data[1]);
	lateral1_servo_command = lateral1_thrust_to_servo_command(command_value.data[2]);
	lateral1_servo1_command = 3000 - lateral1_servo_command;
	lateral1_servo2_command = lateral1_servo_command;
	lateral1_servo3_command = lateral1_servo_command;
	lateral2_servo_command = lateral2_thrust_to_servo_command(command_value.data[2]);
	lateral2_servo1_command = 3000 - lateral2_servo_command;
	lateral2_servo2_command = lateral2_servo_command;
	lateral2_servo3_command = lateral2_servo_command;

	// // output motor
	brushlessmotor1.writeMicroseconds(brushless1_command);
	brushlessmotor2.writeMicroseconds(brushless2_command);
	brushlessmotor3.writeMicroseconds(brushless3_command);
	brushlessmotor4.writeMicroseconds(brushless4_command);
	lateral1_brushlessmotor.writeMicroseconds(lateral_brushless5_command);
	lateral2_brushlessmotor.writeMicroseconds(lateral_brushless6_command);
	lateral1_servo1.writeMicroseconds(lateral1_servo1_command);
	lateral1_servo2.writeMicroseconds(lateral1_servo2_command);
	lateral1_servo3.writeMicroseconds(lateral1_servo3_command);
	lateral2_servo1.writeMicroseconds(lateral2_servo1_command);
	lateral2_servo2.writeMicroseconds(lateral2_servo2_command);
	lateral2_servo3.writeMicroseconds(lateral2_servo3_command);
	nh.loginfo(String(brushless1_command).c_str());
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("yaw_command", &yawCallback);

void setup()
{
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	nh.subscribe(sub);
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

	brushless1_command = 1000;
	brushless1_command = 1000;
	brushless1_command = 1000;
	brushless1_command = 1000;

	lateral_brushless5_command = 1000;
	lateral_brushless6_command = 1000;

	lateral1_servo_command = lateral1_thrust_to_servo_command(0.0);
	lateral1_servo1_command = 3000 - lateral1_servo_command;
	lateral1_servo2_command = lateral1_servo_command;
	lateral1_servo3_command = lateral1_servo_command;

	lateral2_servo_command = lateral2_thrust_to_servo_command(0.0);
	lateral2_servo1_command = 3000 - lateral2_servo_command;
	lateral2_servo2_command = lateral2_servo_command;
	lateral2_servo3_command = lateral2_servo_command;

	brushlessmotor1.writeMicroseconds(brushless1_command);
	brushlessmotor2.writeMicroseconds(brushless2_command);
	brushlessmotor3.writeMicroseconds(brushless3_command);
	brushlessmotor4.writeMicroseconds(brushless4_command);
	lateral1_brushlessmotor.writeMicroseconds(lateral_brushless5_command);
	lateral2_brushlessmotor.writeMicroseconds(lateral_brushless6_command);
	lateral1_servo1.writeMicroseconds(lateral1_servo1_command);
	lateral1_servo2.writeMicroseconds(lateral1_servo2_command);
	lateral1_servo3.writeMicroseconds(lateral1_servo3_command);
	lateral2_servo1.writeMicroseconds(lateral2_servo1_command);
	lateral2_servo2.writeMicroseconds(lateral2_servo2_command);
	lateral2_servo3.writeMicroseconds(lateral2_servo3_command);

	//-----------------------------------
}

void loop()
{
	if (!nh.connected())
	{
		brushlessmotor1.writeMicroseconds(1000);
		brushlessmotor2.writeMicroseconds(1000);
		brushlessmotor3.writeMicroseconds(1000);
		brushlessmotor4.writeMicroseconds(1000);
		lateral1_brushlessmotor.writeMicroseconds(1000);
		lateral2_brushlessmotor.writeMicroseconds(1000);
	}
	nh.spinOnce();
	delayMicroseconds(500);
}
