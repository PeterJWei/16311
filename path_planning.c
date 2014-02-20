#pragma config(Sensor, S4,     touchSensor,         sensorTouch)
#define KP 2.0 //damping ratio
#define KU 1.0 //Feed Forward
#define LX 0.06
#define LY 0.0
#define LEFT motorC
#define RIGHT motorB
#define VELOCITY_UPDATE_TIME 4.0
#define DEAD_UPDATE_TIME 5.0
#define PID_UPDATE_TIME 3.0
#define GEAR_RATIO 1.0
#define HEIGHT 16
#define WIDTH 32
#define SQLEN 0.1524/2.0

#define X_START_DIR 1
#define Y_START_DIR 0
float R = 0.042;
float L = 0.1143;//L = 0.08255;


//float epsilon = 0.05;
//int flag = 0;
float pen_x = 0.0;
float pen_y = 0.0;
float robot_X = 0.0;
float robot_Y = 0.0;
float robot_TH = 0.0;

float target_x = 0.0;
float target_y = 0.0;
//float dt = 0.1;

float left_Vel = 0.0;
float right_Vel = 0.0;

float modulus (float div, float mod) {
	float q = div/mod;
	int qI = floor(q);
	float r = div - mod*qI;
	return r;
}

int xpath[HEIGHT*WIDTH];
int ypath[HEIGHT*WIDTH];
int turns[HEIGHT*WIDTH];

/*int grid[HEIGHT][WIDTH] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},
	{0,0,1,1,0,1,1,0,0,1,1,0,0,0,0,0},
	{1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0},
	{1,1,1,1,1,1,1,1,1,1,0,0,1,1,0,0},
	{0,0,0,0,1,1,0,0,1,0,0,0,1,1,0,0},
	{0,0,0,0,0,0,0,0,0,0,1,1,0,1,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0}
};*/

int grid[HEIGHT][WIDTH] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0},
	{1,1,1,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,1,1,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,1,1,1,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,1,1,1,1,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,1,1,1,1,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0}};

void printgrid() {
	int i,j;
	writeDebugStream ("[\n");
	for (i=HEIGHT-1; i >= 0; i--) {
		for (j = 0;j < WIDTH; j++) {
			writeDebugStream("%d ", grid[i][j]);
		}
		writeDebugStream("\n");
	}
	writeDebugStream("]\n");
}

void printpath(int plen) {
	int i;
	for(i=0; i< plen; i++)
	{
		writeDebugStreamLine("(%d,%d)",xpath[i],ypath[i]);
 	}
}

bool isvalid(int xpos, int ypos, int xdir, int ydir) {
	return (xpos + xdir < WIDTH && xpos + xdir >= 0 &&
	ypos + ydir < HEIGHT && ypos + ydir >= 0 &&
	grid[ypos + ydir][xpos + xdir] > 1);
}

int preplanning(int startx, int starty, int goalx, int goaly) {
	int i,j;
	int n = 1;
	bool changes = true;
	grid[goaly][goalx] = 2;

	while (grid[starty][startx] == 0 && changes) {
		//printgrid();
		n = n+1;
		changes = false;
		for (i = 0; i < HEIGHT; i++) {
			for (j = 0; j < WIDTH; j++) {
				if (grid[i][j] == 0) {//unexplored
					//					if (i >= 0 && i < HEIGHT-1 && j > 0 && j < WIDTH-1) {
					//writeDebugStreamLine("inbounds (%d,%d) is %d",i,j,grid[i][j]);
					if (i > 0 && grid[i-1][j] == n) {
						grid[i][j] = n+1;
						changes = true;
						continue;
					}
					if (i < HEIGHT-1 && grid[i+1][j] == n) {
						changes = true;
						grid[i][j] = n+1;
						continue;
					}
					if (j > 0 && grid[i][j-1] == n) {
						changes = true;
						grid[i][j] = n+1;
						continue;
					}
					if (j < WIDTH-1 && grid[i][j+1] == n) {
						changes = true;
						grid[i][j] = n+1;
						continue;
					}
				}
			}
		}
		//		}
	}
	printgrid();
	if (changes == false) {
		writeDebugStreamLine("No Path Exists");
		return -1;
	}
	wait1Msec(2000);
	int pathlen = grid[starty][startx]-grid[goaly][goalx] + 1;

	int k;
	xpath[0] = startx;
	ypath[0] = starty;
	int xdir = X_START_DIR;
	int ydir = Y_START_DIR;
	int temp = 0;
	for (k = 1; k < pathlen; k++) {
		writeDebugStream("%d %d\n", xpath[k-1],ypath[k-1]);
		if (isvalid(xpath[k-1],ypath[k-1],xdir,ydir)) { //check if we can still go forward
			if (grid[ypath[k-1]+ydir][xpath[k-1]+xdir] < grid[ypath[k-1]][xpath[k-1]]) {
				xpath[k] = xpath[k-1] + xdir;
				ypath[k] = ypath[k-1] + ydir;
				turns[k] = 0;
				continue;
			}
			else
			{
				writeDebugStream("Hit obsticle at (%d,%d)",ypath[k-1]+ydir,xpath[k-1]+xdir);
			}
		}
		if (isvalid(xpath[k-1],ypath[k-1],ydir,xdir)) { //flip x and y dir to rotate 90
			if (grid[ypath[k-1]+xdir][xpath[k-1]+ydir] < grid[ypath[k-1]][xpath[k-1]]) {
				xpath[k] = xpath[k-1] + ydir;
				ypath[k] = ypath[k-1] + xdir;
				temp = xdir;
				xdir = ydir;
				ydir = temp;
				if (ydir != 0) {
					turns[k] = 1;
				}
				else {
					turns[k] = -1;
				}
				continue;
			}
		}
		if (isvalid(xpath[k-1],ypath[k-1],-1*ydir,-1*xdir)) { //flip x and y and signs to rotate -90
			if (grid[ypath[k-1]-xdir][xpath[k-1]-ydir] < grid[ypath[k-1]][xpath[k-1]]) {
				xpath[k] = xpath[k-1] - ydir;
				ypath[k] = ypath[k-1] - xdir;
				temp = -1*xdir;
				xdir = -1*ydir;
				ydir = temp;
				if (ydir != 0) {
					turns[k] = -1;
				}
				else {
					turns[k] = 1;
				}
				continue;
			}
		}
		if (isvalid(xpath[k-1],ypath[k-1],-1*xdir,-1*ydir)) { //flip signs to look backwards
			if (grid[ypath[k-1]-ydir][xpath[k-1]-xdir] < grid[ypath[k-1]][xpath[k-1]]) {
        xpath[k] = xpath[k-1] - xdir;
				ypath[k] = ypath[k-1] - ydir;
				turns[k] = 2;
				xdir = -1*xdir;
				ydir = -1*ydir;
			}
		}
		else {
			writeDebugStreamLine("We got to a bad place (didn't find next place)");
		}
	}
	return pathlen;
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

		wait1Msec(VELOCITY_UPDATE_TIME);

		left_Vel = (newticksL - oldticksL)*KU/(milliToSec*GEAR_RATIO*VELOCITY_UPDATE_TIME)*(PI/180);
		right_Vel = (newticksR - oldticksR)*KU/(milliToSec*GEAR_RATIO*VELOCITY_UPDATE_TIME)*(PI/180);


	}
}//end poll velocity

void setup()
{
	bMotorReflected[LEFT] = false;
	bMotorReflected[RIGHT] = false;
	robot_X = -LX + pen_x;
	robot_Y = -LY + pen_y;
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

		float dt_1 = DEAD_UPDATE_TIME*.001;

		float left_lin_v = left_Vel*R;
		float right_lin_v = right_Vel*R;
		float v = (left_lin_v+right_lin_v)/2;
		float w = (right_lin_v-left_lin_v)/L;


		float k00 = v*cos(robot_TH);
		float k01 = v*sin(robot_TH);
		float k02 = w;
		float k10 = v*cos(robot_TH+(dt_1/(2.0))*k02);
		float k11 = v*sin(robot_TH+(dt_1/(2.0))*k02);
		float k12 = w;
		float k20 = v*cos(robot_TH+(dt_1/(2.0))*k12);
		float k21 = v*sin(robot_TH+(dt_1/(2.0))*k12);
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


		pen_x = cos(robot_TH)*LX - sin(robot_TH)*LY + robot_X;
		pen_y = sin(robot_TH)*LX + cos(robot_TH)*LY + robot_Y;

		nxtDisplayTextLine(0, "X: %f", pen_x);
		nxtDisplayTextLine(1, "Y: %f", pen_y);
		nxtDisplayTextLine(2, "t: %f", 57.2958 * robot_TH);


		wait1Msec(DEAD_UPDATE_TIME);
	}
}//end dead reckoning

task pid_loop()
{
	//	float start_time = nPgmTime;
	float linV = 0;
	float angV = 0;
	float xerror_scaled = 0;
	float yerror_scaled = 0;
	float vl = 0;
	float vr = 0;

	while(true)
	{
		xerror_scaled = KP*(target_x-pen_x);//robot_x
		yerror_scaled = KP*(target_y-pen_y);//robot_y
		linV = (cos(robot_TH) - sin(robot_TH)*LY/LX)*xerror_scaled + (sin(robot_TH)+cos(robot_TH)*LY/LX)*yerror_scaled;
		angV = (-1*sin(robot_TH)/LX)*xerror_scaled + (cos(robot_TH)/LX)*yerror_scaled;
		vl = (90/PI)*(2*linV-L*angV)/(R);
		vr = (90/PI)*(2*linV+L*angV)/(R);
		float power_l = .1*vl*GEAR_RATIO;
		float power_r = .1*vr*GEAR_RATIO;

		motor[LEFT] = power_l;
		motor[RIGHT] = power_r;

		wait1Msec(PID_UPDATE_TIME);
	}
}//end pid_loop

void turnleft() {
	float i;
	//want to turn about robot center
	float tempx = robot_X;
	float tempy = robot_Y;
	float tempTH = robot_TH;
	float theta;
	for (i = 0.0; i < 39.5; i = i + 1.0) {
		theta = PI/60.0*i +tempTH;
		target_x = cos(theta)*LX - sin(theta)*LY + tempx;
		target_y = sin(theta)*LX + cos(theta)*LY + tempy;
		writeDebugStreamLine("%f %f %f\n", target_x,target_y, 3*i);
		wait1Msec(25);
	}
}

void turnright() {
	float i = 0.0;
	float tempx = robot_X;

	float tempy = robot_Y;
	float tempTH = robot_TH;
	float theta;

	/*while(abs(tempTH - robot_TH) < PI/2) {
		theta = -1.0*PI/60.0*i +tempTH;
		target_x = cos(theta)*LX - sin(theta)*LY + tempx;
		target_y = sin(theta)*LX + cos(theta)*LY + tempy;
		writeDebugStreamLine("%f %f %f\n", target_x,target_y, 3*i);
		wait1Msec(25);
		i = i+1.0;
	}*/
	for (i = 0.0; i < 39.5; i = i + 1.0) {
		theta = -1.0*PI/60.0*i +tempTH;
		target_x = cos(theta)*LX - sin(theta)*LY + tempx;
		target_y = sin(theta)*LX + cos(theta)*LY + tempy;
		writeDebugStreamLine("%f %f %f\n", target_x,target_y, 3*i);
		wait1Msec(25);
	}
	wait1Msec(4000);
}

task main()
{
	setup();
	int pathlen = preplanning(28,6,16,14);
	if (pathlen == -1) {
		return;
	}
	printgrid();
  printpath(pathlen);
	StartTask(pollVelocity);
	wait1Msec(100);
	StartTask(dead_reckoning);
	wait1Msec(100);
	StartTask(pid_loop);
	wait1Msec(100);
  /*turnright();
	wait1Msec(2000);*/
	//while(SensorValue(touchSensor) == 0){}
	int i = 0;
	/*target_x = -0.06;
	target_y = 0.06;*/
	for (i = 0; i < pathlen; i++) {
		writeDebugStreamLine("in pathing %f\n", i);
		if (turns[i] == 1) {
			wait1Msec(100);
			turnleft();
			wait1Msec(1000);
			continue;
		}
		if (turns[i] == -1) {
			wait1Msec(100);
			turnright();
			wait1Msec(1000);
			continue;
		}
		if (turns[i] == 2) {
			wait1Msec(100);
			turnright();
			wait1Msec(1000);
			turnright();
			wait1Msec(1000);
			continue;
		}

		float newx = SQLEN*(float)(xpath[i]-xpath[0]);
		float newy = SQLEN*(float)(ypath[i]-ypath[0]);
		float oldx = target_x;
		float oldy = target_y;
		float iterator;
		writeDebugStreamLine("Old: %f %f",xpath[i]-xpath[0],ypath[i]-ypath[0]);
		writeDebugStreamLine("Goal: %f %f",newx,newy);
		for (iterator = 0.0; iterator < 30.5; iterator = iterator + 1.0) {
			target_x = (newx*iterator + oldx*(30.0-iterator))/30.0;
			target_y = (newy*iterator + oldy*(30.0-iterator))/30.0;
			writeDebugStreamLine("%f %f %f\n", target_x,target_y, iterator);
			wait1Msec(25);
		}

	}

	wait1Msec(1000);
	while(SensorValue(touchSensor) == 0){}
}
