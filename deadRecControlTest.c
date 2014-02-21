#pragma config(Sensor, S4,     touchSensor,         sensorTouch)
#define FULL_FORWARD 10
#define LEFT motorC
#define RIGHT motorB
/*****************************************
* Lab 3 : Starter code
* Written by Kaushik Viswanathan
*****************************************/

//Global variables - you will need to change some of these
float KP = 1.0;
float robot_X = 0.0, robot_Y = 0.0, robot_TH = 0.0;
int velocityUpdateInterval = 2;
int PIDUpdateInterval = 2;
int PID_UPDATE_TIME = 2;
int inputB[3] = {0, 0, 0};
int inputC[3] = {0, 0, 0};
int previous_tick_l = 0;
int previous_tick_r = 0;
int first_time = 1;
float prev_time = 0;
float Radius = 0.0275;//.0269875;
float GEAR_RATIO = 1.0;
float L = 0.11;
float LX = -0.1714;
float LY = 0.0001;
float vl = 0.0;
float vr = 0.0;
float pen_X = 0.0;
float pen_Y = 0.0;
float target_x = 0.0;
float target_y = 0.0;
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

void dead_reckoning()
{
		float cur_time = nPgmTime;
		if(first_time){
			prev_time = cur_time;
			first_time = 0;
			wait1Msec(velocityUpdateInterval);
			return;
		}
		//
		//Fill in code for numerical integration / position estimation here
		//

		float dt = 1000*(cur_time-prev_time);
		/* Numerical Integration */
		float cur_r = nMotorEncoder[motorB];
		float cur_l = nMotorEncoder[motorC];
    //writeDebugStreamLine("cur_r = %f, cur_l = %f", cur_r, cur_l);
		vl = (cur_l - previous_tick_l)/dt*(PI/180);
		vr = (cur_r - previous_tick_r)/dt*(PI/180);

		//Left_Wheel_Velocity = vl;
		//Right_Wheel_Velocity = vr;
		//writeDebugStreamLine("previous_tick_r = %f, previous_tick_l = %f", previous_tick_r, previous_tick_l);


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

		pen_X = cos(robot_TH)*LX - sin(robot_TH)*LY + robot_X;
		pen_Y = sin(robot_TH)*LX + cos(robot_TH)*LY + robot_Y;
/*		float xp, yp;
		xp = cosDegrees(57.2958*robot_TH)*lx - sinDegrees(57.2958*robot_TH)*ly;
		yp = sinDegrees(57.2958*robot_TH)*lx + cosDegrees(57.2958*robot_TH)*ly;
		nxtDisplayTextLine(3, "xp: %f", xp);
		nxtDisplayTextLine(4, "yp: %f", yp);*/

		wait1Msec(velocityUpdateInterval);
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

void setup()
{
	bMotorReflected[LEFT] = false;
	bMotorReflected[RIGHT] = false;
	robot_X = -LX + pen_X;
	robot_Y = -LY + pen_Y;
	target_x = pen_X;
	target_y = pen_Y;

}

void pid_loop()
{
	//	float start_time = nPgmTime;
	float linV = 0.0;
	float angV = 0.0;
	float xerror_scaled = 0.0;
	float yerror_scaled = 0.0;
	vl = 0.0;
	vr = 0.0;
	xerror_scaled = KP*(target_x-pen_X);//robot_x
		yerror_scaled = KP*(target_y-pen_Y);//robot_y
		linV = (cos(robot_TH) - sin(robot_TH)*LY/LX)*xerror_scaled + (sin(robot_TH)+cos(robot_TH)*LY/LX)*yerror_scaled;
		angV = (-1*sin(robot_TH)/LX)*xerror_scaled + (cos(robot_TH)/LX)*yerror_scaled;
		vl = (90/PI)*(2*linV-L*angV)/(Radius);
		vr = (90/PI)*(2*linV+L*angV)/(Radius);
		writeDebugStreamLine("%f %f\n", vl, vr);
		float power_l = .1*vl*GEAR_RATIO;
		float power_r = .1*vr*GEAR_RATIO;

		motor[LEFT] = power_l;
		motor[RIGHT] = power_r;

		wait1Msec(PID_UPDATE_TIME);

}//end pid_loop

/*****************************************
* Main function - it is not necessary to
* modify this
*****************************************/
task main()
{
	/* Reset encoders and turn on PID control */
	setup();
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
	nPidUpdateInterval = PIDUpdateInterval;
	float tempx = pen_X;// - cos(robot_TH)*LX;
	float tempy = pen_Y;// - sin(robot_TH)*LX;
	float tempTH = robot_TH;
	float theta;
	float i = 0.0;
	while (1) {
	  /*theta = -1.0*PI/60.0*i +tempTH;
		target_x = cos(theta)*LX - sin(theta)*LY + tempx;
		target_y = sin(theta)*LX + cos(theta)*LY + tempy;*/
		target_x = tempx + i;
		target_y = tempy + i;
		writeDebugStreamLine("%f %f %f %f\n", target_x, target_y, pen_X, pen_Y);
    dead_reckoning();
    pid_loop();
    wait1Msec(200);
    i = i+0.01;
  }
}
