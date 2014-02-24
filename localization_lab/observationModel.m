%pM stands for probability map, map is the input map showing where obstacles
%and tennis balls are located, measurement is the reading from the sensor
%pipeline in the simulator case this is a n by 2 matrix where the first
%column is the bearing to an observed tennis ball and the second column is
%the range to a observed tennis ball. 
function [pM] = observationModel(map,measurement,pM,r)
global DX;
global DY;
global DTH;
[m,n,p] = size(pM);



tennisball_map = double(map == 2);



H = fspecial('gaussian', 11, 5);

locations = tennisball_map;
% locations = convn(tennisball_map, H, 'same');
[numMeasurments,~] = size(measurement);
if(numMeasurments == 0)
    measPM = ones(m,n,p);
    'no tennis ball';
else
    measPM = zeros(m,n,p);
    'Tennis Ball found';
end
for mi = 1:numMeasurments
%     mask = makeCircleMask(round(measurement(mi,2)/DX));
%     convloc = convn(locations,mask,'same');
%     convloc = double(convloc ~= 0);
    for i = 1:p
        radius = measurement(mi,2);
        theta = measurement(mi,1);
        if (i*DTH < .1)
            x = round((radius * cos(theta + r.pose(3)) + r.pose(1))/DX);
            y = round((radius * sin(theta + r.pose(3)) + r.pose(2))/DY);
        end
        mask = makeSliceMask(round(measurement(mi,2)/DX), i*DTH + measurement(mi,1));
        convloc = convn(locations,mask,'same');
        convloc = double(convloc ~= 0);
        measPM(:,:,i) = measPM(:,:,i) + convloc;
    end
end
pM = measPM .* pM;

for i = 1:m
    for j = 1:n
      if(map(i,j) > 0)
%          pM(i,j,:) = 0;
      end
    end
end


end

function [mask] = makeCircleMask(radius)
   if radius >0 
       mask = fspecial('disk',radius+1);
       inner = zeros(size(mask));
       inner(2:end-1,2:end-1) = fspecial('disk',radius);
       mask = mask - inner;
       mask = mask > 0;
   else
       mask = 1;
   end
end

function [mask] = makeSliceMask(radius, theta)
       x = -radius * cos(theta);
       y = -radius * sin(theta);

       mask = zeros((2*radius) + 7, (2*radius) + 7);
       x = round(x);
       y = round(y);
       
       maskfill = [1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1; 1 1 1 1 1];
       %mask(x+radius+3,y+radius+3) = 1;
       mask(x+radius+2:x+radius+6, y+radius+2:y+radius+6) = maskfill;
       
       
%        mask = fspecial('disk',radius+1);
%        inner = zeros(size(mask));
%        inner(2:end-1,2:end-1) = fspecial('disk',radius);
%        mask = mask - inner;
%        mask = mask > 0;
end