function deletePointsFromPlot(pointsPlotted)
%Removes the following scattered objects/points from the current
%scatter plot
    for i = 1:size(pointsPlotted,1)
        point = pointsPlotted(i,:);
        delete(point);
    end
end

