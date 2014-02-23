function [ iB2, finalnum, finalx, finaly ] = analysis( count, colors, iB2 )
%analyzes the pixels in an object, stores the number of pixels and the
%average in the x and y directions of the object to locate the centroid.
width = size(iB2, 1);
height = size(iB2, 2);

red = colors(:,1)';
green = colors(:,2)';
blue = colors(:,3)';

numpixels = zeros(1,count);
averagex = zeros(1,count);
averagey = zeros(1,count);
minxa = ones(1, count) * -1;
maxxa = ones(1, count) * -1;
minya = ones(1, count) * -1;
maxya = ones(1, count) * -1;

for i = 1:width;
    for j = 1:height;
        for k = 1:count;
            if abs(iB2(i,j,1)-red(k)) < .001 && abs(iB2(i,j,2)-green(k)) <.001 && abs(iB2(i,j,3)-blue(k)) < .001;
                if (minxa(k) < 0)
                    minxa(k) = i;
                end
                if (maxxa(k) < 0)
                    maxxa(k) = i;
                end
                if (minya(k) < 0)
                    minya(k) = j;
                end
                if (maxya(k) < 0)
                    maxya(k) = j;
                end
                
                if (minxa(k) > i)
                    minxa(k) = i;
                end
                if (maxxa(k) < i)
                    maxxa(k) = i;
                end
                if (minya(k) > j)
                    minya(k) = j;
                end
                if (maxya(k) < j)
                    maxya(k) = j;
                end
                numpixels(k) = numpixels(k)+1; %increment number of pixels
                averagex(k) = averagex(k)+i; %add the x value
                averagey(k) = averagey(k)+j; %add the y value
            end
        end
    end
end

distx = maxxa-minxa;
disty = maxya-minya;

finalcount = 0;
finalx = zeros(1, size(averagex,2));
finaly = zeros(1, size(averagex,2));
finalnum = zeros(1, size(averagex,2));
for i = 1:size(averagex,2)
    if (distx(i) > 1 && disty(i) > 1 && ((abs(distx(i) - disty(i))/min(distx(i),disty(i))) < .2))
        finalcount = finalcount+1;
        finalx(finalcount) = averagex(i);
        finaly(finalcount) = averagey(i);
        finalnum(finalcount) = numpixels(i);
    end
end

finalx = finalx(1:finalcount);
finaly = finaly(1:finalcount);
finalnum = finalnum(1:finalcount);

finalx = finalx./finalnum; %divide by number of pixels to get x of centroid
finaly = finaly./finalnum; %divide by number of pixels to get y of centroid

%offset = 30;
%%%%%%%%%%DRAW LINES THAT MARK THE OBJECTS!
%line([(averagex(1)-offset) (averagex(1)+offset)], [(averagey(1)-offset) (averagey(1)+offset)], 'Color', colors(1,:))
%line([(averagex(1)+offset) (averagex(1)-offset)], [(averagey(1)-offset) (averagey(1)+offset)], 'Color', colors(1,:))
%line([(averagex(2)-offset) (averagex(2)+offset)], [(averagey(2)-offset) (averagey(2)+offset)], 'Color', colors(2,:))
%line([(averagex(2)+offset) (averagex(2)-offset)], [(averagey(2)-offset) (averagey(2)+offset)], 'Color', colors(2,:))
%line([(averagex(3)-offset) (averagex(3)+offset)], [(averagey(3)-offset) (averagey(3)+offset)], 'Color', colors(3,:))
%line([(averagex(3)+offset) (averagex(3)-offset)], [(averagey(3)-offset) (averagey(3)+offset)], 'Color', colors(3,:))
%line([(averagex(4)-offset) (averagex(4)+offset)], [(averagey(4)-offset) (averagey(4)+offset)], 'Color', colors(4,:))
%line([(averagex(4)+offset) (averagex(4)-offset)], [(averagey(4)-offset) (averagey(4)+offset)], 'Color', colors(4,:))
end