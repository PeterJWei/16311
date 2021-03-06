%pM is the initial probability map, dPose is the change in pose from your
%measurement 
function [pM] = transitionModel(pM,dPose)
global DX;
global DY;
global DTH;
shift = -1*round(dPose./[DX; DY; DTH]);
if(shift(1) ~=0 || shift(2) ~=0)
 %   shift(3)=0;
end
[~,~,p] = size(pM);
for i = 1:p
    mask = makeTransMask(shift,i*DTH);
    pM(:,:,i) = convn(pM(:,:,i), mask, 'same');
end
pM = circshift(pM,[0,0,shift(3)]);

shift = shift

end


function [mask] = makeTransMask(shift,theta)
   sigma = .5;
   gausssize = 5;
   H = fspecial('gaussian',gausssize,sigma);
   
   mask = zeros(2*(abs(shift(1))+gausssize)+1,2*(abs(shift(2))+gausssize)+1);
   centerptx = shift(1)+gausssize+1;
   centerpty = shift(2)+gausssize+1;
%    if(theta > pi/4 && theta < 3*pi/4)
%       gausscenterx = centerptx + shift(1);
%       gausscentery = centerpty + shift(2);
%    end
%    if(theta >5*pi/4 && theta < 7*pi/4)
%       gausscenterx = centerptx - shift(1);
%       gausscentery = centerpty - shift(2);
%    end
%    if(theta > pi/4 && theta < 3*pi/4)
%       gausscenterx = centerptx + shift(1);
%       gausscentery = centerpty + shift(2);
%    end
%    if(theta > pi/4 && theta < 3*pi/4)
%       gausscenterx = centerptx + shift(1);
%       gausscentery = centerpty + shift(2);
%    end
   gausscenterx = centerptx-shift(1);
   gausscentery = centerpty-shift(2);

   mask((gausscenterx-floor(gausssize/2)):(gausscenterx+floor(gausssize/2)),...
        (gausscentery-floor(gausssize/2)):(gausscentery+floor(gausssize/2))) = H;
     
end


