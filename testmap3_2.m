%test fails (takes forever and doesnt find a path)
jointAngles = [1.4,0,1,0,0,0];
map = loadmap("map_3.txt");

[jointPositions,T0e] = calculateFK_sol(jointAngles);



position = T0e(1:3,4).';


start = [292.10000,0,222.2500];  
jointAnglesPath = findpath(map,start,position);

disp(jointAnglesPath);


%tests the computed jointAnglesPath using the physical robot
%{
for i = 1:size(jointAnglesPath,1)
    angles = jointAnglesPath(i,:);
    lynxServo([angles 0]);
    pause(0.2);
    
end
%}