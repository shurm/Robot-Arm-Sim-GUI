%THIS TEST TOOK LESS THAN 1 second
jointAnglesGoal =  [1.2,0.5,-0.7,-0.7,0,0];
%115.4980  297.0784  323.6597

%the robot almost grazes an obstacle, I believe this is due to the rigid
%link between joint3 and joint4 jerking back and forth because of the 
%tension that results from the robot moving then suddenly then stopping 
%between poses/configurations    

map = loadmap("map_2.txt");

[jointPositions2,T0e2] = calculateFK_sol(jointAnglesGoal);

goalPosition = T0e2(1:3,4).';
goalPosition = [292.1 0 100];
startPosition = [292.10000,0,222.2500];
start  = [0 0 0 0];
tic 
jointAnglesPath = findpath(map,start,goalPosition);
timeElapsed = toc;

disp(jointAnglesPath);
disp("finding a path took "+num2str(timeElapsed));

%tests the computed jointAnglesPath using the physical robot
lynxStart();

q = [0 0 0 0 0 0];
plotLynx(q);
plotmap(map);
%scatter3(0,0,0);

pointsPlotted = showPath(jointAnglesPath);
lastConfig = jointAnglesPath(size(jointAnglesPath,1),:);
disp(lastConfig);