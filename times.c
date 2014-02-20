#pragma config(Sensor, S4,     touchSensor,         sensorTouch)
#define KP 2.0 //damping ratio
#define KV 1.0 //scaling from velocity to power
#define KD 1.0
#define LX -0.19
#define LY 0.0
#define LEFT motorC
#define RIGHT motorB
#define VELOCITY_UPDATE_TIME 4.0
#define DEAD_UPDATE_TIME 3.0
#define PID_UPDATE_TIME 3.0
#define GEAR_RATIO 3.0

int fxn = 8;

float R = 0.02154;
float L = 0.178;

float epsilon = 0.05;
int flag = 0;
float pen_x = 0.0;
float pen_y = 0.0;
float robot_X = 0.0;
float robot_Y = 0.0;
float robot_TH = 0.0;

float target_x = 0.0;
float target_y = 0.0;
float dt = 0.1;

float left_Vel = 0.0;
float right_Vel = 0.0;


float modulus (float div, float mod) {
	float q = div/mod;
	int qI = floor(q);
	float r = div - mod*qI;
	return r;
}


task pollVelocity()
{

	float newticksL = nMotorEncoder[LEFT];
	float newticksR = nMotorEncoder[RIGHT];

	float milliToSec = 0.001;
	wait1Msec(VELOCITY_UPDATE_TIME);
	while(true)
	{
		float oldticksL = newticksL;
		float oldticksR = newticksR;
		newticksL = nMotorEncoder[LEFT];
		newticksR = nMotorEncoder[RIGHT];
		//writeDebugStreamLine("%f %f", newticksL, newticksR);
		wait1Msec(VELOCITY_UPDATE_TIME);

		left_Vel = (newticksL - oldticksL)/(milliToSec*GEAR_RATIO*VELOCITY_UPDATE_TIME)*PI/180;
		right_Vel = (newticksR - oldticksR)/(milliToSec*GEAR_RATIO*VELOCITY_UPDATE_TIME)*PI/180;

		//writeDebugStreamLine("leftVel rightVel: %f ,%f", left_Vel, right_Vel);
	}
}//end poll velocity

float xfxn1(float t)
{
	return 0.25*cos(t/10)*sin(t/10);
}
float yfxn1(float t)
{
	return 0.1*sin(t/10)*sin(t/5);
}

float xfxn2(float t)
{
	float A = 0.1;
	float alpha = .6;
	float phi_x = 0;

	return A*sin(t/alpha+phi_x);
}

float yfxn2(float t)
{
	float A = 0.1;
	float beta = .4;
	float phi_y = PI/2;

	return A*cos(t/beta+phi_y);
}

float xfxn3(float t)
{
	return 0.2*cos(t/10.0)*cos(t/5.0);
}

float yfxn3(float t)
{
	return 0.2*cos(3*t/10.0)*sin(t/10.0);
}

float xfxn4(float t)
{
	return 0.16*(1.0/2.0*cos(3.0*t/10.0)-3.0/4.0*cos(t/5.0));
}

float yfxn4(float t)
{
  return 0.16*(-3.0/4.0*sin(t/5.0)-1.0/2.0*sin(3.0*t/10.0));
}

float xfxn5(float t)
{
	return 0.1*(-2.0*cos(t/5.0)*cos(t/5.0)-sin(t/10.0)+1.0)*sin(t/5.0);
}

float yfxn5(float t)
{
	return 0.1*cos(t/5.0)*(-2*cos(t/5.0)*cos(t/5.0)*cos(t/5.0)-sin(t/10.0)+1);
}

float xfxn6(float t)
{
	return 0.1*(2.0*cos(t/12.0)*cos(t/12.0)*cos(t/12.0)+1.0)*sin(t/4.0);
}

float yfxn6(float t)
{
	return 0.1*(cos(t/4.0)*(1.0-2.0*sin(t/4.0)*sin(t/4.0)*sin(t/4.0)*sin(t/4.0)));
}

float xfxn7(float t)
{
	return 0.01*(5.0*cos(9.0*t/20.0)-4.0*cos(t/4.0));
}

float yfxn7(float t)
{
	return 0.01*(-4.0*sin(t/4.0)-5.0*sin(9.0*t/20.0));
}
float xnum(float t)
{
	if (t < 10.0) {
		t = 10.0-t;
		return 0.05*cos(t*2.0/10.0*0.75*PI);
	}
	if (t >= 10.0 && t < 20.0) {
		t = t - 10.0;
		return 0.05+0.005*t;
	}
	if (t >= 20.0 && t < 30.0) {
		return 0.1;
	}
	if (t >= 30.0 && t < 35.0) {
		return 0.15;
	}
	if (t >= 35.0 && t < 50.0) {
		t = t - 35.0;
		return 0.1 + 0.05*cos(t*15.0/(2.0*PI));
	}
	return 0.15;
}

float ynum(float t)
{
	if (t < 10.0) {
		t = 10.0-t;
		return 0.05*sin(t/10.0*0.75*2.0*PI)+0.05;
	}
	if (t >= 10.0 && t < 20.0) {
		t = t - 10.0;
		return 0.05-0.01*t;
	}
	if (t >= 20.0 && t < 30.0) {
		t = 30.0-t;
		return 0.015*t-0.05;
	}
	if (t >= 30.0 && t < 35.0) {
		return -0.05;
  }
  if (t >= 35.0 && t < 50.0) {
		t = t - 35.0;
		return -0.05 + 0.05*sin(t*15.0/(2.0*PI));
	}
  return -0.05;
}

void setup()
{
	bMotorReflected[LEFT] = true;
	bMotorReflected[RIGHT] = true;
	if (fxn == 3) {
		pen_x = 0.2;
    pen_y = 0.0;
  }
  if (fxn == 4) {
  	pen_x = -0.05;
  	pen_y = 0.0;
  }
  if (fxn == 5) {
  	pen_x = 0.0;
  	pen_y = -0.1;
  }
  if (fxn == 6) {
  	pen_x = 0.0;
  	pen_y = 0.1;
  }
  if (fxn == 7) {
  	pen_x = 0.04;
  	pen_y = 0.0;
  }
	robot_X = -1*LX +pen_x;
	robot_Y = -1*LY + pen_y;
	target_x = pen_x;
	target_y = pen_y;

}

task dead_reckoning()
{
	int first_time = 1;
	float prev_time = nPgmTime;
	wait1Msec(10);
	while(1)
	{
		float cur_time = nPgmTime;
		if(first_time){
			prev_time = cur_time;
			first_time = 0;
			wait1Msec(DEAD_UPDATE_TIME);
			continue;
		}
		//
		//Fill in code for numerical integration / position estimation here
		//

		float dt_1 = (cur_time-prev_time)/1000;

		float left_lin_v = left_Vel*(PI/180);
		float right_lin_v = right_Vel*(PI/180);
		float v = (left_lin_v+right_lin_v)/2;
		float w = (right_lin_v-left_lin_v)/L;
		//writeDebugStreamLine("v and w: %f, %f", v, w);

		float k00 = v*cos(robot_TH);
		float k01 = v*sin(robot_TH);
		float k02 = w;
		float k10 = v*cos(robot_TH+(dt_1/(2))*k02);
		float k11 = v*sin(robot_TH+(dt_1/(2))*k02);
		float k12 = w;
		float k20 = v*cos(robot_TH+(dt_1/(2))*k12);
		float k21 = v*sin(robot_TH+(dt_1/(2))*k12);
		float k22 = w;
		float k30 = v*cos(robot_TH+(dt_1)*k22);
		float k31 = v*sin(robot_TH+(dt_1)*k22);
		float k32 = w;

		robot_X = robot_X + (dt_1/6)*(k00+2*(k10+k20)+k30);
		robot_Y = robot_Y + (dt_1/6)*(k01+2*(k11+k21)+k31);
		robot_TH = robot_TH + (dt_1/6)*(k02+2*(k12+k22)+k32);
		robot_TH = modulus(robot_TH, 2*PI);
		//writeDebugStreamLine("Rob: %f %f %f", robot_X, robot_Y,57.2958*robot_TH);
		//Code that plots the robot's current position and also prints it out as text
		// nxtSetPixel(50 + (int)(100.0 * robot_X), 32 + (int)(100.0 * robot_Y));
		// nxtDisplayTextLine(0, "X: %f", robot_X);
		// nxtDisplayTextLine(1, "Y: %f", robot_Y);
		// nxtDisplayTextLine(2, "t: %f", 57.2958 * robot_TH);
		prev_time = cur_time;
		pen_x = cos(robot_TH)*LX - sin(robot_TH)*LY + robot_X;
		pen_y = sin(robot_TH)*LX + cos(robot_TH)*LY + robot_Y;
		//writeDebugStreamLine("%f", dt_1);
		//writeDebugStreamLine("%d", flag);
		writeDebugStreamLine("pen: %f %f target: %f %f", pen_x, pen_y, target_x, target_y);

		wait1Msec(DEAD_UPDATE_TIME);
	}
}//end dead reckoning


//run a pid loop for time t, where t is in milliseconds
task pid_loop()
{
	float start_time = nPgmTime;
	float linV = 0;
	float angV = 0;
	float xerror_scaled = 0;
	float yerror_scaled = 0;
	float vl = 0;
	float vr = 0;

	while(/*nPgmTime-start_time < (int)(1000*t)*/true)
	{
		xerror_scaled = KP*(target_x-pen_x);
		yerror_scaled = KP*(target_y-pen_y);
		linV = (cos(robot_TH) - sin(robot_TH)*LY/LX)*xerror_scaled + (sin(robot_TH)+cos(robot_TH)*LY/LX)*yerror_scaled;
		angV = (-1*sin(robot_TH)/LX)*xerror_scaled + (cos(robot_TH)/LX)*yerror_scaled;
		vl = (90/PI)*(2*linV-L*angV)/(R);
		vr = (90/PI)*(2*linV+L*angV)/(R);
		motor[LEFT] = .1*vl*GEAR_RATIO;
		motor[RIGHT] = .1*vr*GEAR_RATIO;
		//writeDebugStream("Power:  %d %d ", motor[LEFT], motor[RIGHT]);
		//writeDebugStreamLine("Pen Pos (%f,%f)  Target Pos (%f, %f)",pen_x,pen_y,target_x,target_y);
		wait1Msec(PID_UPDATE_TIME);
	}


}//end pid_loop

task main() {

	setup();
	StartTask(pollVelocity);
	wait1Msec(100);
	StartTask(dead_reckoning);
	wait1Msec(100);
	while(SensorValue(touchSensor) == 0){}

	//Main Drive loop


	//Dead Reckoning Unit Test
	/*
	motor[LEFT] = 50;
	motor[RIGHT] = 50;
	wait1Msec(5000);
	motor[LEFT] = -50;
	motor[RIGHT] = -50;
	wait1Msec(5000);
	motor[LEFT] = 0;
	motor[RIGHT] = 0;
	wait1Msec(5000);
	*/

	//PID UNIT TEST
	/*
	StartTask(pid_loop);
	wait1Msec(1000);
	while(SensorValue(touchSensor) == 0)
	{
			target_x = .1;
			target_y = .1;
	}
	*/

  float count = 0.0;
	float RunTime = 65.0;
	StartTask(pid_loop);
	wait1Msec(1000);
	for(float i = 0.0; i < RunTime; i = i+dt)
	{
		if (fxn == 1) {
		  target_y = yfxn1(i);
		  target_x = xfxn1(i);
		  wait1Msec(100);
	  }
		if (fxn == 2) {
		  target_y = yfxn2(i);
		  target_x = xfxn2(i);
	  }
	  if (fxn == 3) {
		  target_y = yfxn3(i);
		  target_x = xfxn3(i);
		  wait1Msec(301);
	  }
	  if (fxn == 4) {
	  	RunTime = 64.5;
	  	target_y = yfxn4(i);
	  	target_x = xfxn4(i);
	  	wait1Msec(100);
	  }
	  if (fxn == 5) {
	  	RunTime = 69.0;
	  	target_y = yfxn5(i);
	  	target_x = xfxn5(i);
	  	wait1Msec(100);
	  }
	  if (fxn == 6) {
	  	RunTime = 75.5;
	  	target_y = yfxn6(i);
	  	target_x = xfxn6(i);
	  	wait1Msec(100);
	  }
	  if (fxn == 7) {
	  	RunTime = 125.5;
	  	target_y = yfxn7(i);
	  	target_x = xfxn7(i);
	  	wait1Msec(100);
	  }
	  if (fxn == 8) {
	  	RunTime = 50.0;
	  	target_y = ynum(i);
	  	target_x = xnum(i);
	  	wait1Msec(100);
	  }

		count = count + 1.0;
		writeDebugStreamLine("%f", count);
	}

}//end main
