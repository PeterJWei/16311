classdef robot < handle
    
    properties
        pose = [];
        WHEELRADIUS = .1; %meters
        AXLELENGTH = .5;%assuming both wheels are equal spaced from the center of the robot
        modelColor = 'r';%default color is red
        model = [];
    end
    
    methods
        function thisRobot = robot(x,y,th)
            thisRobot.pose = [x,y,th];
        end
        
        function setModel(thisRobot,newModel,newModelColor)
           thisRobot.model = newModel; 
           thisRobot.modelColor = newModelColor;
        end
        
        function setOdomTransform(thisRobot,newOdomTransform)
           thisRobot.odomTransform = newOdomTransform;
        end
        
        function DrawRobot(thisRobot)
            x  = thisRobot.pose(1);
            y  = thisRobot.pose(2);
            th = thisRobot.pose(3);
            H = [cos(th) -sin(th) x;
                sin(th) cos(th) y;
                0         0     1;];
            robot = H*thisRobot.model;
            hold on;
            %is just above the map
            fill3(robot(1,:),robot(2,:),ones(size(robot,2),1),thisRobot.modelColor);
            drawAxis(x,y,th,.1);
        end
        
        function objects =  Sense(thisRobot,map)
            global DX;
            global DY;
            global MAXD;
            objects = [];
            minAngle = deg2rad(-30);
            maxAngle = deg2rad(30);
            dAngle = deg2rad(1);

            
            for th = minAngle:dAngle:maxAngle
                d = 0;
               while d < MAXD
                   x = round((cos(th+thisRobot.pose(3))*d + thisRobot.pose(1))/DX);
                   y = round((sin(th+thisRobot.pose(3))*d + thisRobot.pose(2))/DY); 
                   if(x>0 && x < size(map,1) && y > 0 && y < size(map,2))
                       if(map(x,y) == 2)
                           objects = cat(1,objects,[th, d]);%d %bearing only instead of distince
                       end
                    if(map(x,y) == 1)
                        %hit a wall
                        break
                    end
                   end
                   d = d+.02;
               end
            end
            
        end
             
        
        %commands input as a speed and a change in orintation
        function [x,y,th] = Move(thisRobot,speed,dr,map)
            x = 0;
            y = 0;
            th = 0;
            MAXSPEED = .05;
            MAXTH = pi/4;
            if abs(speed) > MAXSPEED
                speed = MAXSPEED*speed/abs(speed);
            end
            
            if abs(dr) > MAXTH
                dr = MAXTH*dr/abs(dr);
            end            
                  
            dth = dr;
            dx = cos(thisRobot.pose(3))*speed;
            dy = sin(thisRobot.pose(3))*speed;
            p = thisRobot.pose + [dx,dy,dth];
             if(incontact(p,map,thisRobot.model))
                 display('can not move due to be in contatct with obsticle')
                 %if you are incontact with an obsticle end
                 %the move routine
                 return
             end            
            WHEELBASE = .5;
            th = wrapTo2Pi(thisRobot.pose(3)); 
             
            %makes sure that speed/rotation is possible
            if th > .5*pi && th < 1.5*pi
                if dy > 0
                    sign = -1;
                else
                    sign = 1;
                end
            else
                if dy > 0
                    sign = 1;
                else
                    sign = -1;
                end
            end
            olddx = dx;
            olddy = dy;
            dx = cos(thisRobot.pose(3))*olddx -sin(thisRobot.pose(3))*olddy;
            dy = sin(thisRobot.pose(3))*olddx + sin(thisRobot.pose(3))*olddy;
            
            LE = (sign*sqrt(dx*dx + dy*dy) - dth*(WHEELBASE)/2);
            RE = LE + (dth*WHEELBASE);
            LE = floor(LE);%Discretization of  encoder
            RE = floor(RE);%Discretization of  encoder
            thisRobot.pose = thisRobot.pose + [olddx,olddy,dth];
            thisRobot.pose(3) = wrapTo2Pi(thisRobot.pose(3));
            v = LE+RE/2;
            w = RE-LE/WHEELBASE;
            x = v*cos(thisRobot.pose(3)); 
            y = v*sin(thisRobot.pose(3));
            th = w;
        end

    end
    
end

function val =  distince(a,b)
d = b-a;
val = sqrt(d(1)*d(1) + d(2)*d(2));
end

function d = lineDistince(p,q1,q2)
dq = q2' - q1';
if(dq == 0)
    d = distince(q1,p);
    return 
end
%projects all of the lines onto the same plane
dq(3) = 0;
p(3) = 0;
d = max(max(abs(cross(dq,p'-q1'))/abs(dq)));
end

function val = near(p,q1,q2)
if distince(q1,p) + distince(q2,p) < distince(q1,q2) +.5
    val = 1;
else
    val = 0;
end
end

%evaluates to val = 1 if in contact, and 0 if not in contact
function val = incontact(p,map,model)
global DX
global DY
x  = p(1);
y  = p(2);
th = p(3);
H = [cos(th) -sin(th) x;
     sin(th) cos(th) y;
    0         0     1;];
   robot = H*model;
val = 0;
for k = 1:size(robot,2)
    i = round(robot(1,k)/DX);
    j = round(robot(2,k)/DY);
    if(i > 0 && i <= size(map,1) && j > 0 && j <=size(map,2))
        map(i,j)
        if(map(i,j) ~= 0)
         val = 1;
        end
    else
        display('outside of board')
        val = 1;
    end
end
end
