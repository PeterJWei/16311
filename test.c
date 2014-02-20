#pragma config(Sensor, S1,     lightSensor,         sensorLightActive)

task main()
{
	int light_thresh = 45;
	wait1Msec(50);
	while(true){
		float light_data = SensorValue(lightSensor);
		int current_color = light_data < light_thresh;
		if (current_color) {
			//Go forward if black
			motor[motorC] = 70;//60
			motor[motorB] = 70;
		}
		else {
			//Overshoot
			motor[motorC] = 40;
			motor[motorB] = 40;
			wait1Msec(250);
			//look right a bit
			motor[motorC] = 40;
			motor[motorB] = -40;
			int startTime = nPgmTime;
			int endTime = nPgmTime;
			while(!current_color && endTime - startTime < 400) {

				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
			  endTime = nPgmTime;
		  }
		  if (current_color) {
		  	continue;
      }
			//look left a bit
			motor[motorC] = -40;
			motor[motorB] = 40;

			startTime = nPgmTime;
			endTime = nPgmTime;
			while(!current_color && endTime - startTime < 400) {

				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
			  endTime = nPgmTime;
		  }
		  if (current_color) {
		  	continue;
    }
      /*light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			if (current_color) {
				continue;
			}*/

      /*light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			if (current_color) {
				continue;
			}*/
						//look left a bit
			motor[motorC] = -40;
			motor[motorB] = 40;
			light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			startTime = nPgmTime;
			endTime = nPgmTime;
			while(!current_color && endTime - startTime < 700) {

				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
			  endTime = nPgmTime;
		  }
		  if (current_color) {

		  	continue;
    }
			//look right a bit
			motor[motorC] = 40;
			motor[motorB] = -40;//30
			light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			startTime = nPgmTime;
			endTime = nPgmTime;
      while(!current_color && endTime - startTime < 1600) {
				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
			  endTime = nPgmTime;
		  }
		  if (current_color) {
		  	continue;
		  }
			//look left a bit
			motor[motorC] = -40;
			motor[motorB] = 40; //30
			light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			startTime = nPgmTime;
			endTime = nPgmTime;
			while(!current_color && endTime - startTime < 1800) {

				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
			  endTime = nPgmTime;
		  }
		  if (current_color) {
		  	continue;
    }
			//look right a bit
			motor[motorC] = 40;
			motor[motorB] = -40;//30
			light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			startTime = nPgmTime;
			endTime = nPgmTime;
      while(!current_color && endTime - startTime < 2400) {
				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
			  endTime = nPgmTime;
		  }
		  if (current_color) {
		  	continue;
		  }

			/*
			motor[motorC] = -20;
			motor[motorB] = -20;
			wait1Msec(150);
			motor[motorC] = 0;
			motor[motorB] = 0;
			wait1Msec(1000);
			motor[motorC] = -10;
			motor[motorB] = 10;
			light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;
			int startTime, endTime;
			startTime = nPgmTime;
			while(current_color) {
				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
		  }
			endTime = nPgmTime;
			motor[motorC] = 0;
			motor[motorB] = 0;
			wait1Msec(1000);
			if (endTime - startTime < 400) {
				motor[motorC] = 10;
				motor[motorB] = -10;
				wait1Msec(endTime - startTime);
				wait1Msec(1000);
			}
			else {
				motor[motorC] = 10;
			  motor[motorB] = -10;
			  wait1Msec(100);
			  continue;
		  }
		  motor[motorC] = 0;
			motor[motorB] = 0;
			wait1Msec(2000);
		  /*motor[motorC] = 0;
			motor[motorB] = 0;
			wait1Msec(2000);*/
			/*motor[motorC] = 10;
			motor[motorB] = -10;
			light_data = SensorValue(lightSensor);
			current_color = light_data < light_thresh;

			while(current_color) {
				light_data = SensorValue(lightSensor);
			  current_color = light_data < light_thresh;
		  }
		  motor[motorC] = -10;
		  motor[motorB] = 10;
		  wait1Msec(100);*/

		}
	}
}
