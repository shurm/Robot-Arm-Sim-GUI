function successors = generateAdjacentNodes(jointAngles)
%T generates a matrix (each row contains a configuration)  
%T 

% generates a matrix containing 2 different configurations for each joint
% angle, each of these new configurations is checked to ensure that it does
% not violate the joint limits, 


l = size(jointAngles,2);
successors = [];
difference = 0.05;

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

