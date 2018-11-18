function path = findPathFromStart( goal)

% finds a path from the start configuration to the specified goal 
% effector position provided
% 
%
%
% goal  : a 1x3 vector containing the coordinates (x,y,z) of the goal
%         location
%
% path  : an Nx5 vector of joint variable values which constitute the path found
%         using your planner; Note: N is not known beforehand and will vary
%         depending how the path is constructed

    
    %the end effector position at when joint angles are zero
    zeroPosition = [292.10000,0,222.2500];
    
    %I chose 5mm to be negligible distance between two 3d points in space  
    EPSILON = 5;
    
    
    robotsJointAngles = [0,0,0,0];
    path = [];
    

    %%open list has format [jointAngles cost f path]
    openList = [robotsJointAngles 0 norm(goal - zeroPosition) cellstr(mat2str(robotsJointAngles))];
    
    closedList=robotsJointAngles;
    %%disp(openList);

    while ~isempty(openList)

        minRowIndex = getMin(openList);
        minRow = openList(minRowIndex,:);

        openList=[openList(1:minRowIndex-1,:) ; openList(minRowIndex+1:size(openList,1)-1,:)];

        currentJointAngles = cell2mat(minRow(:,1));
        cost = cell2mat(minRow(:,2));
        strPath = char(minRow(:,4));
        %%disp(path);
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

                %disp(openListSlice);
                %disp("------------------------------------------");
                %%disp(size(successorJointAngles,2));
                %disp(size(openListSlice,2));

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
      %disp(size(openList(:,3),1));
      closedList = [currentJointAngles;closedList];
    end

end
    