%bp = belived Pose
%dp = desired Pose
function val = atDesiredLocation(bp,dp)
global  MAXERRORX;
global  MAXERRORY;
    val = abs(bp(1)-dp(1))> MAXERRORX || abs(bp(2) -dp(2)) > MAXERRORY;
end