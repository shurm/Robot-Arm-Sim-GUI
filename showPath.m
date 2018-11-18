function showPath(jointAnglesPath)
%DISPLAYPATHANDMAP Summary of this function goes here
%   Detailed explanation goes here
    pauseIntervalInSeconds = 0.2;

    
    for i = 1:size(jointAnglesPath,1)
        angles = jointAnglesPath(i,:);

        plotLynx([angles 0]);
        pause(pauseIntervalInSeconds);

    end
end

