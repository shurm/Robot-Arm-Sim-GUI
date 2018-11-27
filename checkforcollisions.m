function collided = checkforcollisions(jointPositions, obstacles)
% Input:    jointPositions - 6 x 3 matrix, where each row represents one 
%               joint along the robot. Each row contains the [x,y,z]
%               coordinates of the respective joint's center (mm). For
%               consistency, the first joint should be located at [0,0,0].
%           obstacles - n x 6 matrix, where each row represents one 
%               rectangular prism obstacle that exists in the physical 
%               environment. 
%			
% Outputs:	True - if the distance between any of the joints and any of the
%               obstacles in the environment is within the collisionRadius 
%           False - if all of the joints are far enough away from the
%               obstacles in the environment (outside the collisionRadius)
%           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %the minimum threhold distance that a joint position can be from safely
    %from an obstacle. (assume it collides if it gets closer).
    collisionRadius = 50;
    
    %assume no collision at first
    collided = false;
    
    %ignore first 2 joint positions, because those joints are fixed
    jointPositions = jointPositions(2:size(jointPositions,1),:);
    
    %goes through every obstacle
    for obstacleIndex = 1:size(obstacles,1)
        
        %goes through every joint
        for jointIndex = 1:size(jointPositions,1)
            box = obstacles(obstacleIndex,:);
            if distPointToBox(jointPositions(jointIndex,:),box) < collisionRadius
                collided = true;
                return
            end
        end
    end
end

