function path = findpath(map, startingConfig, goal, stoppingDist)

% function for finding an obstacle free path through the provided map from
% the start location to the goal location
% 
% map   : a representation of the environment including obstacles used for
%         planning
%
% start : a 1x3 vector containing the coordinates (x,y,z) of the start
%         location
%
% goal  : a 1x3 vector containing the coordinates (x,y,z) of the goal
%         location
%
% stoppingDist : a scalar specifying the how far the end effector of the
%               robot arm can be before it becomes ("close enough") to the 
%               goal
%
% path  : an Nx5 vector of joint variable values which constitute the path found
%         using your planner; Note: N is not known beforehand and will vary
%         depending how the path is constructed

    
    % YOUR CODE GOES HERE
   
    %%goal =[236.9421   48.0305   81.4802];
    %%start = [292.10000,0,222.2500];
    
    %the end effector position at when joint angles are zero
    %zeroPosition = [292.10000,0,222.2500];
    
    %I chose 40mm to be negligible distance between two 3d points in space  
    %EPSILON = 40;
    EPSILON = stoppingDist * 10;
    
    %the maximum time this algorithem can take before it times out  
    maximumWaitTime = 1;
    
    startingConfig = startingConfig(:,1:4);
    
    
    [~,T0e] = calculateFK_sol([startingConfig 0]);
    start = T0e(1:3,4).';
           
    
    %%open list has format [jointAngles cost f path]
    openList = [startingConfig 0 norm(goal - start) cellstr(mat2str(startingConfig))];
    
    closedList=startingConfig;
    %%disp(openList);
    timeElapsed = 0;
    
    tic 
    while (~isempty(openList) && timeElapsed<maximumWaitTime)

        minRowIndex = getMin(openList);
        minRow = openList(minRowIndex,:);

        openList=[openList(1:minRowIndex-1,:) ; openList(minRowIndex+1:size(openList,1)-1,:)];

        currentJointAngles = cell2mat(minRow(:,1));
        cost = cell2mat(minRow(:,2));
        strPath = char(minRow(:,4));
     
        successors = generateAdjacentNodes(currentJointAngles);

        %disp(currentJointAngles);


        %evaluate each successor configuration
        for i = 1:size(successors,1)

            successorJointAngles = successors(i,:);

            % if successor configuration is in the closed list ignore it
            [~,index] = ismember(successorJointAngles, closedList,'rows');
            if index ~= 0
                continue;
            end

            [jointPositions,T0e] = calculateFK_sol([successorJointAngles 0 0]);
            ePosition = T0e(1:3,4).';
            
            %check for collisions
            collided = checkforcollisions(jointPositions,map.obstacles);
            
            %Since this configuration causes a collision, add it to the 
            %closed list to avoid testing the same configuration in the
            %future
            if collided
                closedList = [successorJointAngles;closedList];
                continue;
            end
            
            

            currentSuccessorJointAnglesString = mat2str(successorJointAngles);
            newPath  = strcat(currentSuccessorJointAnglesString,strPath);
            
            %if the end effector position is at the goalPosition/close
            %enough store the path and stop the search
            if norm(goal - ePosition)<EPSILON
                
                %converts the path string into an Nx5 matrix
                splitString = newPath;
                newStr = split(splitString,']');
                path = [];
                for i2 = 1:size(newStr,1)-1
                    configurationStr = newStr(i2);
                    configurationStr = strcat(configurationStr,' 0]');
                    configuration = eval(char(configurationStr));
                    path = [configuration;path];
                end
               
                %stops the search
                return;
            end
            % openlist is empty add the successor to it (this will happen
            % during the first iteration)
            if isempty(openList)
                openList = [successorJointAngles cost+1 norm(goal - ePosition)+cost+1 cellstr(newPath);openList];
            else
                openListSlice = cell2mat(openList(:,1));


                %add successor configuration to openlist, along with its cost,f
                %and path values only if that successor configuration is not 
                %already in the openlist
                [~,index] = ismember(successorJointAngles, openListSlice,'rows');
                if index == 0
                    openList = [successorJointAngles cost+1 norm(goal - ePosition)+cost+1 cellstr(newPath);openList];
                else
                     %disp(successorJointAngles);
                     %disp("is a member of");
                     %disp(openListSlice);
                end
            end  
        end
     
      closedList = [currentJointAngles;closedList];
      timeElapsed = toc;
    end
    path = "The position you specified is either outside the robot's reach or too close to an obstacle";
end
    