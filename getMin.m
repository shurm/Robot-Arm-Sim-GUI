function minRow = getMin(openList)
%T returns the index of the row in the openlist that has the least f value
    column_where_F_Is = 3;
    currentMin = cell2mat(openList(1,column_where_F_Is));
    currentMinRow = 1;
    for r = 2:size(openList,1)
        min =  cell2mat(openList(r,column_where_F_Is));
        if(min<currentMin)
            currentMinRow = r;
            currentMin = min;
        end
    end
    minRow = currentMinRow; 
end