%pM stands for probability map, map is the input map showing where obstacles
%and tennis balls are located, measurement is the reading from the sensor
%pipeline in the simulator case this is a n by 2 matrix where the first
%column is the bearing to an observed tennis ball and the second column is
%the range to a observed tennis ball. 
function [pM] = observationModel(map,measurement,pM)

[m,n,p] = size(pM);
for i = 1:m
    for j = 1:n
      if(map(i,j) > 0)
          pM(i,j,:) = 0;
      end
    end
end


tennisball_map = double(map == 2);



H = fspecial('gaussian', 10, .5);

locations = convn(tennisball_map, H, 'same');
[numMeasurments,~] = size(measurement);
if(numMeasurments == 0)
    measPM = ones(m,n,p);    
else
    measPM = zeros(m,n,p);
end
for mi = 1:numMeasurments
    mask = makeCircleMask(measurement(mi,2));
    convloc = convn(locations,mask,'same');
    for i = 1:p
        measPM(:,:,i) = measPM(:,:,i) + convloc;
    end
end
measPM = normilize(measPM);
pM = measPM .* pM;




end

function [mask] = makeCircleMask(radius)
   mask = fspecial('disk',radius+1);
   inner = zeros(size(mask));
   inner(2:end-1,2:end-1) = fspecial('disk',radius);
   mask = mask - inner;
   mask = mask > 0;
end