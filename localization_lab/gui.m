function [ output_args ] = gui(r,map,pMap)
figure(1)
clf();
 global DTH;
 global DX;
 global DY;
subplot(1,2,1)
plotMap(map,1);
r.DrawRobot()
[x,y,th,p] = getBestDistribution(pMap);
br = robot(x,y,th);%belived robot for visulizing the belived robo 
br.setModel([-.05, -.05, 0 , 0.05, 0;
            .05, -.05, -0.05, 0 ,0.05;
             1,1,1,1,1],'g');
         
br.DrawRobot();
 th = mod(round(th/DTH)-1,size(pMap,3) -1)+1;
subplot(1,2,2)

%  maxPMap = max(pMap,[],3);
 plotMap(pMap(:,:,th),0)
%  plotMap(maxPMap, 0)
% subplot(2,2,3)
% plotMap(pMap(:,:,4),0)
% subplot(2,2,4)
% plotMap(pMap(:,:,8),0)
end