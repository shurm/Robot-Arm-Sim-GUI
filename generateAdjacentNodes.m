function successors = generateAdjacentNodes(jointAngles)
% Input: jointAngles - 1 x 4 vector of joint angles [q1,q2,q3,q4] 
%           a set of joint angle is a configuration.
%
% Outputs: successors - n x 4 matrix, where each row represents one 
%               sightly configuration from jointAngles. 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% generates a matrix containing 2 different configurations for each joint
% angle, each of these new configurations is checked to ensure that it does
% not violate the joint limits, 

    %the amount in radians to add or subtract from a jointAngle to obtain a 
    %new configuration
    difference = 0.05;

    l = size(jointAngles,2);
    successors = [];

    for i = 1:l
        copy = jointAngles(:,1:l);
        copy(i) = jointAngles(i)+difference;
        if(~pastLimit(i, copy))
            successors = [successors; copy];
        end

        copy = jointAngles(:,1:l);

        copy(i) = jointAngles(i)-difference;
        if(~pastLimit(i, copy))
            successors = [successors; copy];
        end

    end

end

