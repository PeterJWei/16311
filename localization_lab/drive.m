
%speed is the foward spped of the robot, turning is the angular
%acceleration desired of the robot, dt is the amount of time which you wish
%to have the robot drive for.
function [B_ticks,C_ticks] = drive(speed,turning,dt)
%implment foward kinematics here %%%%TODO 
leftSpeed = 0;
rightSpeed = 0;


mB = NXTMotor('B', 'Power', leftSpeed);
mC = NXTMotor('C', 'Power', rightSpeed);
%resets motors
mB.ResetPosition();
mC.ResetPosition();
mB.SendToNXT();
mC.SendToNXT();
pause(dt);
B_ticks = mB.ReadFromNXT().position;
C_ticks = mC.ReadFromNXT().position;

end

