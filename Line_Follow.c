#pragma config(Sensor, S1, lightSensor, sensorLightActive)
#pragma config(Sensor, S4, touchSensor, sensorTouch)
#define LEFT motor[motorC]
#define RIGHT motor[motorB]
#define REF_LEFT bMotorReflected[motorC]
#define REF_RIGHT bMotorReflected[motorB]
#define FORWARD_SPEED 40
#define BACKWARD_SPEED 0

task main()
{
	int light_thresh = 30;
	while (SensorValue(touchSensor) == 0)
	{
		int light_data = SensorValue(lightSensor);
		if	(light_data < light_thresh)
		{
			REF_LEFT = false;
			REF_RIGHT = true;

			LEFT = FORWARD_SPEED;
			RIGHT = BACKWARD_SPEED;
		}
		else
		{
			REF_LEFT = true;
			REF_RIGHT = false;
			LEFT = BACKWARD_SPEED;
			RIGHT = FORWARD_SPEED;
		}
	}
}
