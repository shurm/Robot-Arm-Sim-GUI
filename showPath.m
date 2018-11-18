function pointsPlotted = showPath(jointAnglesPath)
%DISPLAYPATHANDMAP Summary of this function goes here
%   Detailed explanation goes here
    pauseIntervalInSeconds = 0.2;

    pointsPlotted = zeros(size(jointAnglesPath,1),1);
    for i = 1:size(jointAnglesPath,1)
        angles = jointAnglesPath(i,:);

        plotLynx([angles 0]);
        pause(pauseIntervalInSeconds);

        hold on;
        [~,T0e2] = calculateFK_sol(angles);
        eePos = T0e2(1:3,4).';
        
        pointsPlotted(i,:) = scatter3(eePos(1),eePos(2),eePos(3));
       
    end
end

