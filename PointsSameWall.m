%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr, eunjujeong178@gmail.com)
%
% Description : 
% 
% Input
% 1) pointCloud : 
% 2) refittedLineModel : MF에 맞게 수정된 line model(a,b,c)
% 3) TH_DISTANCE :
%
% Output 
% 1) pointsIdxInThres : 직선과의 거리가 TH_DISTANCE 안에 있는 poinCloud 안의 모든 점들
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [pointsIdxSameWall] = PointsSameWall(pointCloud, walls, TH_DISTANCE_BETWEEN_REFITTED_LINE, TH_DISTANCE_BETWEEN_ENDPOINT) 
 
    % set parameters
    numPoint = size(pointCloud, 2);
    distanceEachPoint = zeros(1,numPoint);
 
    % assign line model parameters (ax + by + c = 0)
    a = walls.refittedLineModel(1);
    b = walls.refittedLineModel(2);
    c = walls.refittedLineModel(3);


    if walls.alignment == 'x'
        x1 = (walls.min_xy_M(2) - walls.refittedLineModel(3))/walls.refittedLineModel(1);
        y1 = walls.min_xy_M(2);
        x2 = (walls.max_xy_M(2) - walls.refittedLineModel(3))/walls.refittedLineModel(1);
        y2 = walls.max_xy_M(2);
        lineEndPoint1 = [x1; y1];
        lineEndPoint2 = [x2; y2];
    elseif walls.alignment == 'y'
        x1 = walls.min_xy_M(1);
        y1 = walls.refittedLineModel(1)*x1 + walls.refittedLineModel(3);
        x2 = walls.max_xy_M(1);
        y2 = walls.refittedLineModel(1)*x1 + walls.refittedLineModel(3);
        lineEndPoint1 = [x1; y1];
        lineEndPoint2 = [x2; y2];
    end


    % distance between each point and line
    denominator = sqrt(a^2 + b^2);
    for k = 1:numPoint
        distanceEachPoint(k) = abs((a*pointCloud(1,k) + b*pointCloud(2,k) + c) / (denominator));
    end

    pointsIdxInThres = find(distanceEachPoint <= TH_DISTANCE_BETWEEN_REFITTED_LINE);

    if isempty(pointsIdxInThres) ~= 0
        pointsIdxSameWall = [];
    else
        % calculate the Euclidean distance between the end points of line and points in pointsIdxInThres
        numPointInThres = size(pointsIdxInThres,2); % TH_DISTANCE 안에 있는 점들의 개수
        minDistanceLineEndPoint = zeros(1,numPointInThres); % 각 점들과 line의 끝점 간의 거리 (각각 양 끝점간의 거리 중 가장 가까운 값)
        
        pointCloud_x = pointCloud(1,:);
        pointCloud_y = pointCloud(2,:);
        pointCloudInThres = [pointCloud_x(pointsIdxInThres);pointCloud_y(pointsIdxInThres)]; % TH_DISTANCE 안에 있는 점들의 x y 값
        
    
        % distance between each point and the end point of line
        for k = 1:numPointInThres
            minDistanceLineEndPoint(k) = min(norm(lineEndPoint1 - pointCloudInThres(:,k)), norm(lineEndPoint2 - pointCloudInThres(:,k)));
        end
    
        pointsIdxSameWall = find(minDistanceLineEndPoint <= TH_DISTANCE_BETWEEN_ENDPOINT);
        pointsIdxSameWall = pointsIdxInThres(pointsIdxSameWall);
    end
end