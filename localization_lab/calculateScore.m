    %caulates and displays the score of the run 
function calculateScore(r,finish)
FT_TO_METERS = 0.3048;
IN_TO_METERS = FT_TO_METERS/12;
METERS_TO_IN = 1/IN_TO_METERS;

fprintf('end Location: x: %d y: %d th: %d \n',r.pose(1),r.pose(2),r.pose(3));
fprintf('end Goal:     x: %d y: %d th: %d \n',finish(1),finish(2),finish(3));
error = abs(r.pose - finish)*METERS_TO_IN;
error(3) = r.pose(3) - finish(3);
fprintf('error:  x: %d y:%d th: %d \n',error(1),error(2),error(3));

L1 = error(1) + error(2);
if(L1 < 3)
    score = 10;
elseif(L1 < 6)
    score = 5;
elseif(L1 < 12)
    score = 2;
else
    score = 0;
end
fprintf('score: %d',score);
end