function [ output_args ] = gui(r,map,pMap)

figure(1)
clf();
 global DTH;
 global DX;
 global DY;
 global ROBOTMODEL;
subplot(1,2,1)
plotMap(map,1);
r.DrawRobot()
[x,y,th,p] = getBestDistribution(pMap);
br = robot(x,y,th);%belived robot for visulizing the belived robo 
br.setModel(ROBOTMODEL,'g');

br.DrawRobot();
th = mod(round(th/DTH)-1,size(pMap,3) -1)+1;
subplot(1,2,2)
plotMap(pMap(:,:,th),0)
end