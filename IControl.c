#pragma config(Sensor, S1,     lightSensor,         sensorLightActive)
#define KI 3
int decReduction(int error_acc){
	if(error_acc == 0){
		return 0;
	}
	return 100-KI*error_acc;
}
task main()
{
int light_thresh = 40;
wait1Msec(50);
int prev_color = SensorValue(lightSensor) < light_thresh;
int error_acc = 0;
while(true){
	int light_data = SensorValue(lightSensor);
	int current_color = light_data < light_thresh;
	if(current_color == prev_color){
		error_acc++;
	} else {
		error_acc = 0;
		prev_color = !prev_color;
	}

  if (current_color) {
  	//Go Right if black
  	motor[motorC] = 100;
  	motor[motorB] = decReduction(error_acc);
  	wait1Msec(200);



  }
  else {
  	//Go Left if white
  	motor[motorB] = 100;
  	motor[motorC] = decReduction(error_acc);
  }
}

}
