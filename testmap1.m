%THIS TEST TOOK LESS THAN 1 second
jointAnglesGoal =  [1.2,0.5,-0.7,-0.7,0,0];

%the robot almost grazes an obstacle, I believe this is due to the rigid
%link between joint3 and joint4 jerking back and forth because of the 
%tension that results from the robot moving then suddenly then stopping 
%between poses/configurations    

map = loadmap("map_1.txt");

[jointPositions2,T0e2] = calculateFK_sol(jointAnglesGoal);

goalPosition = T0e2(1:3,4).';

startPosition = [292.10000,0,222.2500];
tic 
jointAnglesPath = findpath(map,startPosition,goalPosition);
timeElapsed = toc;

disp(jointAnglesPath);
disp("finding a path took "+num2str(timeElapsed));

%tests the computed jointAnglesPath using the physical robot
lynxStart();

q = [0 0 0 0 0 0];
plotLynx(q);
plotmap(map);
for i = 1:size(jointAnglesPath,1)
    angles = jointAnglesPath(i,:);

    plotLynx([angles 0]);
    pause(0.2);
    
end
