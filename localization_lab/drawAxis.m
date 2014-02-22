%draws an axis at the desired location
function  drawAxis(x,y,zRot,scale)
    hold on
    xs = [x x+scale*cos(zRot)];
    ys = [y y+scale*sin(zRot)];
    plot(xs,ys,'r')
    xs = [x x+scale*cos(zRot+pi/2)];
    ys = [y y+scale*sin(zRot+pi/2)];
    plot(xs,ys,'b')
end