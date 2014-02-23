function [ angles, averagedist ] = disttoball6( im, numpix, averagex, averagey)
%computes distance to the board. We do this by using the focal length and
%the lens width and height of the iPhone 5 camera, which was used to take
%the pictures. The camera dimensions are .135x.1787 inches, with a focal
%length of .1714 inches.

h = size(im, 2);
w = size(im, 1);
radius2 = numpix/pi;
diameter = 2*sqrt(radius2)-20;
nh = size(numpix, 2);
nw = nh;
%balldisty = max(averagey)-min(averagey);
%balldistx = max(averagex)-min(averagex);

%need to know assumption about the distance between 2 balls
%if (big > .5)
%    knownlen = 15; %if the big board
%else
%    knownlen = 5; %if the small board
%end
knownlen = 2.5;
h1 = ones(1, nh) * h;
w1 = ones(1, nw) * w;

%imageplaney = h/balldisty*knownlen;
%imageplanex = w/balldistx*knownlen;
imageplaney = h1./diameter*knownlen;
imageplanex = w1./diameter*knownlen;
cameraheight = .135;
camerawidth = .1787;
focallength = .1614;

horizontalangle = 57.937;

angles = averagex/h*horizontalangle - (horizontalangle/2);
d2 = imageplaney*focallength/cameraheight;
w2 = imageplanex*focallength/camerawidth;
averagedist = (d2+w2)/2; %take the average of the calculated distance from height and width

end