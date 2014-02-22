function plotMap(map,c)
%if c is 1 then use default colormap
global DX
global DY
%displays the map

map = cat(1,map,map(size(map,1),:));
map = cat(2,map,map(:,size(map,2)));

x = zeros(size(map,1),size(map,2));
y = zeros(size(map,1),size(map,2));
for i = 1:size(x,1)
    x(i,:) = i*DX;
end
for j = 1:size(y,2)
    y(:,j) = j*DY;
end

cMap = zeros(size(map,1),size(map,2),3);
for i = 1:size(map,1)
    for j = 1:size(map,2)
    if map(i,j) == 1
        cMap(i,j,:) = [0,0,0];
    elseif map(i,j) == 2
        cMap(i,j,:) = [0,1,0];
    else
        cMap(i,j,:) = [1,1,1];
    end
    end
end


if(c)
    surface(x,y,map,cMap);
else
    surface(x,y,map)
hold on
daspect([max(daspect)*[1 1] 1])     
xlabel('x');
ylabel('y');
zlabel('th');        
end