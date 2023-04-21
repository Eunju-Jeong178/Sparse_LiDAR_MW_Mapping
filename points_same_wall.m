%--------------------------------------------------------------------------
% Description : 
% 
% Function to find the points corresponding to the line stored in MW_Map_FPLiDAR
%--------------------------------------------------------------------------

function [pointsIdxSameWall] = points_same_wall(pointCloud, MW_Map_FPLiDAR, th_distance_between_refitted_line, th_distance_between_endPoint) 
 
    % set parameters
    numPoint = size(pointCloud, 2);
    distanceEachPoint = zeros(1,numPoint);
 

    if MW_Map_FPLiDAR.alignment == 'x' % orthogonal to X-axis of MF
        x1 = MW_Map_FPLiDAR.offset;
        y1 = MW_Map_FPLiDAR.min_xyz_M(2);
        x2 = MW_Map_FPLiDAR.offset;
        y2 = MW_Map_FPLiDAR.max_xyz_M(2);
        lineEndPoint1 = [x1; y1];
        lineEndPoint2 = [x2; y2];
    elseif MW_Map_FPLiDAR.alignment == 'y' % orthogonal to Y-axis of MF
        x1 = MW_Map_FPLiDAR.min_xyz_M(1);
        y1 = MW_Map_FPLiDAR.offset;
        x2 = MW_Map_FPLiDAR.max_xyz_M(1);
        y2 = MW_Map_FPLiDAR.offset;
        lineEndPoint1 = [x1; y1];
        lineEndPoint2 = [x2; y2];
    end


    % distance between each point and line
    if MW_Map_FPLiDAR.alignment == 'x' % orthogonal to X-axis of MF
        for k = 1:numPoint
            distanceEachPoint(k) = abs(pointCloud(1,k) - MW_Map_FPLiDAR.offset); % |x value of each point - x_offset|
        end
    elseif MW_Map_FPLiDAR.alignment == 'y' % orthogonal to Y-axis of MF
        for k = 1:numPoint
            distanceEachPoint(k) = abs(pointCloud(2,k) - MW_Map_FPLiDAR.offset); % |y value of each point - y_offset|
        end
    end

    % Include the points if the distance between the point and the line is less than or equal to threshold
    pointsIdxInThres = find(distanceEachPoint <= th_distance_between_refitted_line); 

    if isempty(pointsIdxInThres) ~= 0
        pointsIdxSameWall = [];
    else
        % calculate the Euclidean distance between the end points of line and points in pointsIdxInThres
        numPointInThres = size(pointsIdxInThres,2); % the number of points in threshold
        minDistanceLineEndPoint = zeros(1,numPointInThres); % The closest distance between the endpoint of line and the point
        
        % the x,y coordinates of the point within the threshold
        pointCloud_x = pointCloud(1,:);
        pointCloud_y = pointCloud(2,:);
        pointCloudInThres = [pointCloud_x(pointsIdxInThres);pointCloud_y(pointsIdxInThres)];  
        
        % distance between each point and the end point of line
        for k = 1:numPointInThres
            minDistanceLineEndPoint(k) = min(norm(lineEndPoint1 - pointCloudInThres(:,k)), norm(lineEndPoint2 - pointCloudInThres(:,k)));
        end
    
        pointsIdxSameWall = find(minDistanceLineEndPoint <= th_distance_between_endPoint);
        pointsIdxSameWall = pointsIdxInThres(pointsIdxSameWall);
    end
end