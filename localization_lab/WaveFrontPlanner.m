function [ plan ] =WaveFrontPlanner(map,start,finish)
%NOTE mapGenerator must be run before this funciton b
global DX
global DY
assert(DX >0)
assert(DY >0)
%transitions from meters to grid coordinates 
start  = round(start/DX);
finish = round(finish/DY);

%returns -1 if it is not possible to plan from the start loction given 
if start(1) < 1 || start(1) > size(map,1) || start(2) <1 || ... 
   start(2) > size(map,2) || map(start(1),start(2)) ~= 0
    plan = -1;
    return 
end

%makes sure that the start is on the map
if start(1) > size(map,1)-1
    start(1) = size(map,1)-1;
elseif start(1) < 2 
        start(1) = 2;
end

if start(2) > size(map,2)-1
    start(2) = size(map,2)-1;
elseif start(2) < 2
        start(2) = 2;
end
%temp value for plan in case we have already reached the destination
plan = start;
map = applayWave(map,start);
%NOTE THIS WILL NOT WORK IF THEIR IS NO PATH FROM FINISH TO START
%finds the shortest path to the start by using gradiant assent 
current = finish;
path = [];
min = map(current(1),current(2));
minCell = current;
while current(1) > 0 && current(1) < size(map,1) && ...
      current(2) > 0 && current(2) < size(map,2) && ...
      map(current(1),current(2)) ~= -1
     [min,minCell] = checkMin(current(1)+1,current(2),map,min,minCell);
     [min,minCell] = checkMin(current(1)-1,current(2),map,min,minCell);
     [min,minCell] = checkMin(current(1),current(2)+1,map,min,minCell);
     [min,minCell] = checkMin(current(1),current(2)-1,map,min,minCell);
     path = cat(1,minCell,path);
     current = path(1,:);
end
%converts from gird coordinates to metes
if(size(path,1) > 0)
    path(:,1) = DX*path(:,1);
    path(:,2) = DY*path(:,2);
    %plotPath(path)
    plan = path(2:size(path,1),:);
end
end

function [min,minCell] = checkMin(i,j,map,min,minCell)
 if i > 0 && i < size(map,1) && j > 0 && j < size(map,2) ...
 && map(i,j) > min && map(i,j) <0
     min = map(i,j);
     minCell = [i,j];
 end
end


%applys the wave to the map
function map = applayWave(map,start)
toProcess = start;
map(start(1),start(2)) = -1;
while(1) 
    %we have checked all possible paths 
    if(size(toProcess,1) == 0)
        break;
    end
    [current,toProcess] =pop(toProcess);
    new = [];
    [new,map] = check(current(1)+1,current(2),map,new,map(current(1),current(2))-1);
    [new,map] = check(current(1)-1,current(2),map,new,map(current(1),current(2))-1);
    [new,map] = check(current(1),current(2)+1,map,new,map(current(1),current(2))-1);
    [new,map] = check(current(1),current(2)-1,map,new,map(current(1),current(2))-1);
    toProcess = push(new,toProcess);
end

end

function [new,map] = check(i,j,map,new,value)
     if(i > 0 && i <= size(map,1) &&  ...
        j > 0 && j <= size(map,2))
        %location to check is on the baord so we should continue 
     if((map(i,j) < value || map(i,j) == 0))
         %we should update the map
         map(i,j) = value;
         new = cat(1,new,[i,j]);
     end
     end
         %else case no point to add a new element to the to process list as it will
         %any path that uses this one as a building block will be longer
         %then this one.
end

function [element,newList] =  pop(list)
    element = list(1,:);
    newList = list(2:size(list,1),:);
end

function [newList] = push(element,oldList)
    newList = cat(1,oldList,element);    
end

function plotPath(path)
    global DX
    global DY
    %draws path in the center of the grid that it will take
     hold on
     plot3(path(:,1)+DX/2,path(:,2)+DY/2,ones(size(path,1),1),'r');
end


