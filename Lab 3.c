#pragma config(Sensor, S4,     touchSensor,         sensorTouch)
#define FULL_FORWARD 10
/*****************************************
* Lab 3 : Starter code
* Written by Kaushik Viswanathan
*****************************************/

//Global variables - you will need to change some of these
float robot_X = 0.0, robot_Y = 0.0, robot_TH = 0.0;
int velocityUpdateInterval = 2;
int PIDUpdateInterval = 2;
int inputB[3] = {0, 0, 0};
int inputC[3] = {0, 0, 0};
int previous_tick_l = 0;
int previous_tick_r = 0;
float prev_time = 0;
float Radius = .0275;//.0269875;
float L = .11;
float lx = -.1714;
float ly = .0001;
//float Left_Wheel_Velocity = 0;
//float Right_Wheel_Velocity = 0;

/*****************************************
* Complete this function so that it
* continuously updates the robot's position
*****************************************/

float modulus (float div, float mod) {
	float q = div/mod;
	int qI = floor(q);
	float r = div - mod*qI;
  return r;
}

task dead_reckoning()
{
	int first_time = 1;
	while(1)
	{
		float cur_time = nPgmTime;
		if(first_time){
			prev_time = cur_time;
			first_time = 0;
			wait1Msec(velocityUpdateInterval);
			continue;
		}
		//
		//Fill in code for numerical integration / position estimation here
		//

		float dt = 1000*(cur_time-prev_time);
		/* Numerical Integration */
		float cur_r = nMotorEncoder[motorB];
		float cur_l = nMotorEncoder[motorC];
    writeDebugStreamLine("cur_r = %f, cur_l = %f", cur_r, cur_l);
		float vl = (cur_l - previous_tick_l)/dt*(PI/180);
		float vr = (cur_r - previous_tick_r)/dt*(PI/180);

		//Left_Wheel_Velocity = vl;
		//Right_Wheel_Velocity = vr;
		writeDebugStreamLine("previous_tick_r = %f, previous_tick_l = %f", previous_tick_r, previous_tick_l);


		float Vleft = vl*Radius;
		float Vright = vr*Radius;

		float v = (Vleft+Vright)/2;
		float w = (Vright-Vleft)/L;

		float k00 = v*cos(robot_TH);
		float k01 = v*sin(robot_TH);
		float k02 = w;
		float k10 = v*cos(robot_TH+(dt/2)*k02);
		float k11 = v*sin(robot_TH+(dt/2)*k02);
		float k12 = w;
		float k20 = v*cos(robot_TH+(dt/2)*k12);
		float k21 = v*sin(robot_TH+(dt/2)*k12);
		float k22 = w;
		float k30 = v*cos(robot_TH+dt*k22);
		float k31 = v*sin(robot_TH+dt*k22);
		float k32 = w;

    robot_X = robot_X + (dt/6)*(k00+2*(k10+k20)+k30);
    robot_Y = robot_Y + (dt/6)*(k01+2*(k11+k21)+k31);
    robot_TH = robot_TH + (dt/6)*(k02+2*(k12+k22)+k32);
    robot_TH = modulus(robot_TH, 2*PI);
		//Code that plots the robot's current position and also prints it out as text
		nxtSetPixel(50 + (int)(100.0 * robot_X), 32 + (int)(100.0 * robot_Y));
		nxtDisplayTextLine(0, "X: %f", robot_X);
		nxtDisplayTextLine(1, "Y: %f", robot_Y);
		nxtDisplayTextLine(2, "t: %f", 57.2958 * robot_TH);
		prev_time = cur_time;
		previous_tick_l = cur_l;
		previous_tick_r = cur_r;

		float xp, yp;
		xp = cosDegrees(57.2958*robot_TH)*lx - sinDegrees(57.2958*robot_TH)*ly;
		yp = sinDegrees(57.2958*robot_TH)*lx + cosDegrees(57.2958*robot_TH)*ly;
		nxtDisplayTextLine(3, "xp: %f", xp);
		nxtDisplayTextLine(4, "yp: %f", yp);

		wait1Msec(velocityUpdateInterval);
	}
}

/*****************************************
* Function that draws a grid on the LCD
* for easier readout of whatever is plot
*****************************************/
void draw_grid()
{
	for(int i = 0; i < 65; i++)
	{
		nxtSetPixel(50, i);
		int grid5 = (i - 32) % 5;
		int grid10 = (i - 32) % 10;
		if(!grid5 && grid10)
		{
			for(int j = -2; j < 3; j++)
			{
				nxtSetPixel(50 + j, i);
			}
		}
		else if(!grid10)
		{
			for(int j = -4; j < 5; j++)
			{
				nxtSetPixel(50 + j, i);
			}
		}
	}
	for(int i = 0; i < 101; i++)
	{
		nxtSetPixel(i, 32);
		int grid5 = (i - 100) % 5;
		int grid10 = (i - 100) % 10;
		if(!grid5 && grid10)
		{
			for(int j = -2; j < 3; j++)
			{
				nxtSetPixel(i, 32 + j);
			}
		}
		else if(!grid10)
		{
			for(int j = -4; j < 5; j++)
			{
				nxtSetPixel(i, 32 + j);
			}
		}
	}
}

/*****************************************
* Function that fills in the inputB and
* inputC arrays with values inputted
* through motorA
*****************************************/
void getInput()
{
	int i = 0;
	nMotorEncoder[motorA] = 0;
	nNxtButtonTask = 0;
	while(i < 3)
	{
		while(nNxtButtonPressed != kEnterButton)
		{
			inputB[i] = (int)(nMotorEncoder[motorA] / 5.0);
			if(inputB[i] > 100) {inputB[i] = 100; nMotorEncoder[motorA] = 100;}
			else if(inputB[i] < -100) {inputB[i] = -100; nMotorEncoder[motorA] = -100;}
			nxtDisplayTextLine(2 * i, "B : %d", inputB[i]);
			wait10Msec(1);
		}
		wait10Msec(30);
		while(nNxtButtonPressed != kEnterButton)
		{
			inputC[i] = (int)(nMotorEncoder[motorA] / 5.0);
			if(inputC[i] > 100) {inputC[i] = 100; nMotorEncoder[motorA] = 100;}
			else if(inputC[i] < -100) {inputC[i] = -100; nMotorEncoder[motorA] = -100;}
			nxtDisplayTextLine(2 * i + 1, "C : %d", inputC[i]);
			wait10Msec(1);
		}
		i++;
		nMotorEncoder[motorA] = 0;
		wait10Msec(30);
	}
	for(int j = 0; j < 8; j++)
		nxtDisplayClearTextLine(j);
}

/*****************************************
* Main function - it is not necessary to
* modify this
*****************************************/
task main()
{
	/* Reset encoders and turn on PID control */
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
	nPidUpdateInterval = PIDUpdateInterval;

	//getInput();

	draw_grid();

	wait1Msec(500);

	time1[T1] = 0;
	StartTask(dead_reckoning);
	motor[motorB] = 30;
	motor[motorC] = -30;
	wait1Msec(3000);
	motor[motorB] = 0;
	motor[motorC] = 0;
	while (SensorValue(touchSensor) == 0) {
	}
	motor[motorB] = -80;
	motor[motorC] = 60;
	wait1Msec(5000);
	motor[motorB] = -40;
	motor[motorC] = -60;
	wait1Msec(5000);
	motor[motorB] = 0;
	motor[motorC] = 0;
	for(int i = 0; i < 3; i++)
	{

		/*int start_time = nPgmTime;
		int cur_time = nPgmTime;
		while(cur_time - start_time > 5000){*/
	  /*motor[motorB] = inputB[i];
		motor[motorC] = inputC[i];*/
			/*wait1Msec(PIDUpdateInterval);*/

	/*	wait10Msec(100 * 5);*/
	}
	motor[motorB] = 0;
	motor[motorC] = 0;
	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
