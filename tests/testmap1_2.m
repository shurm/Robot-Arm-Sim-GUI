%THIS TEST TOOK LESS THAN 0.2 seconds
jointAnglesStart = [0, 0, 0, 0, 0, 0];
jointAnglesGoal =  [-1.1,0.2,-0.2,-1,0,0];

map = loadmap("map_1.txt");

[jointPositions,T0e] = calculateFK_sol(jointAnglesStart);
%disp(jointPositions);
[jointPositions2,T0e2] = calculateFK_sol(jointAnglesGoal);
%disp(jointPositions2);

startPosition = T0e(1:3,4).';
goalPosition = T0e2(1:3,4).';

%start = [292.10000,0,222.2500];
tic 
jointAnglesPath = findpath(map,startPosition,goalPosition);
timeElapsed = toc;

disp(jointAnglesPath);
disp("finding a path took "+num2str(timeElapsed));

lynxStart();
q = [0 0 0 0 0 0];
plotLynx(q);
plotmap(map);
for i = 1:size(jointAnglesPath,1)
    angles = jointAnglesPath(i,:);

    plotLynx([angles 0]);
    pause(0.2);
    
end
