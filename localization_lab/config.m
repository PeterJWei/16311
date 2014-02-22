function config()
global DX        %discretization along x axis
global DY;       %discretization along y axis
global DTH;      %discretization along th
global MAXDTH    %max speed at which we allow for the robot to move
global MAXSPEED  %max speed at which we allow the robot to translate
global MAXD      %MAXIMUM distince that the sensor can read
global STEPERROR %amount of error added to each cell that could possibly be
global MAXERRORX
global MAXERRORY
global MAXERRORTH
global LEFTMOTOR
global RIHGTMOTOR
global ROBOTMODEL % model of what the robot's foot print looks like, 
                  % when in doubt make this bigger then the actuall robots
                  % footprint. Used to calculate the robots configuration space. 

ROBOTMODEL = [-.05, -.05,    0 , 0.05,  0;
               .05, -.05, -0.05, 0   , 0.05;
                 1,    1,     1,    1, 1 ];                
                  
                  
%Global values are set, must happen before any other funciton are called
DX = .05;%discretization along intiral x axis
DY = .05;%discretization along intiral y axis
DTH = .05;
MAXD = .5;%MAXIMUM distince that the sensor can read
STEPERROR = 0.0000001; %make this more deterministick
MAXERRORX = 0.1;
MAXERRORY = 0.1;
MAXERRORTH = 0.2;
LEFTMOTOR = 'B';
RIHGTMOTOR = 'C';


end