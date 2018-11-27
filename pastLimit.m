function outOfBounds = pastLimit(index,jointAngles)
%T Checks if the joint angle specified by index, is within the joint limits
%given in lynxServoPhysical

    lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm)
    upperLim = [1.4 1.4 1.7 1.7 1.5 30];

    outOfBounds = false;
    if(jointAngles(index) < lowerLim(index) || jointAngles(index) > upperLim(index))
        outOfBounds = true;
    end

end

