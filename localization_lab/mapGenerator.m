function [ map ] = mapGenerator( input_args )
global FT_TO_METERS
global DX
global DY
FT_TO_METERS = 0.3048;
width = 8*FT_TO_METERS;% 8 ft but width  is in meters
height = 4*FT_TO_METERS;%4 ft but heigth is in meters
map = zeros(30,10);
map = zeros(width/DX,height/DY);
map = addBorder(map);

%map = addTennisBall(map,1.1,.45);
%map = addTennisBall(map,.3,.45);
%map = addTennisBall(map,.5,.45);

map = addBox(map,0.5,0.4,1);
map = addBox(map,1.9,0.3,0);
map = addBox(map,1.4,.6,0);

%map = addTennisBall(map,1.6,1.2);
%map = addTennisBall(map,1,.9);
map = addTennisBall(map,1.46,.1);
map = addTennisBall(map,2.4,.1);
map = addTennisBall(map,.2,.9);
map = addTennisBall(map,.4,.6);
map = addTennisBall(map,1,1);
map = addTennisBall(map,1.2,.5);
map = map';


end



function [map] = addBox(map,x,y,r) %Lower left corner
        global DX
        global DY
        global FT_TO_METERS
        x1 = x/DX;
        y1 = y/DY;
        if r 
            x2 = x1 + (2*FT_TO_METERS)/DX;
            y2 = y1 + (.5*FT_TO_METERS)/DY;
        else
            x2 = x1 + (.5*FT_TO_METERS)/DX;
            y2 = y1 + (2*FT_TO_METERS)/DY;
        end
        x1 = floor(x1);
        x2 = floor(x2);
        y1 = floor(y1);
        y2 = floor(y2);
        map(x1:x2,y1:y2) = 1;
end

function [map] = addTennisBall(map,x,y)
        global DX
        global DY
        global FT_TO_METERS
        x = floor(x/DX);
        y = floor(y/DY);
    map(x,y) = 2;
end

function [map] = addBorder(map)
    map(:,1) =1;
    map(:,size(map,2)) =1;
    map(size(map,1),:) =1;
    map(1,:) = 1;
end
