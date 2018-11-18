function collided = checkforcollisions(jointPositions,obstacles)
%CHECKFORCOLLISIONS Summary of this function goes here
%   Detailed explanation goes here
    collided = false;
    jointPositions = jointPositions(2:size(jointPositions,1),:);
    
    %adds rectangler padding around the obstacles
    padding = 50;
    
    for obstacleIndex = 1:size(obstacles,1)
        for jointIndex = 1:size(jointPositions,1)
            box = obstacles(obstacleIndex,:);
            if distPointToBox(jointPositions(jointIndex,:),box) < padding
                collided = true;
                return
            end
        end
    end
end

