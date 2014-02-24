function [x,y,th,p] = getBestDistribution(pM)
  global DX;
  global DY;
  global DTH;
   %Given an array of maps we extract the state which is most likely to
   %estimate the position of the robot correctly 
   
   %temp values should be overriden 
   x = 0;
   y = 0;
   th = 0;
   p = 0;
    best = max(max(max(pM)));
    for i = 1:size(pM,1)
        for j = 1:size(pM,2)
            for k = 1:size(pM,3)
                if pM(i,j,k) == best
                    x = i*DX;
                    y = j*DY;
                    th = k*DTH;
                    %th = wrapTo2Pi(th + pi);
                    p = best;
                    return
                end
            end
        end
    end
    

end

