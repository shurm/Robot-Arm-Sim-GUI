function pointsPlotted = showPath(jointAnglesPath)
%DISPLAYPATHANDMAP Summary of this function goes here
%   Detailed explanation goes here
    global currentconf
	global currentlyShowingPath
	
	currentlyShowingPath = true;
    pauseIntervalInSeconds = 0.2;

    pointsPlotted = zeros(size(jointAnglesPath,1),1);
    for i = 1:size(jointAnglesPath,1)
	
		if ~currentlyShowingPath
			break
		end
        angles = jointAnglesPath(i,:);
        
        currentconf = angles;
        
        plotLynx([angles 0]);
        
        hold on;
        [~,T0e2] = calculateFK_sol(angles);
        eePos = T0e2(1:3,4).';
        
        pointsPlotted(i,:) = scatter3(eePos(1),eePos(2),eePos(3));
		
		pause(pauseIntervalInSeconds);

    end
    if ~currentlyShowingPath
        pointsPlotted = pointsPlotted(1:i-1,:);
    end
   
end

