%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
%
% Description : 
% 
% Input
% 1) pointCloud
% 2) refittedLineModel
% 3) TH_DISTANCE
%
% Output 
% 1) pointsIdxInThres : 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [pointsIdxInThres] = PointsInThres(pointCloud, refittedLineModel, TH_DISTANCE)

    % set parameters
    numPoint = size(pointCloud, 2);
    distanceEachPoint = zeros(1,numPoint);

    % assign line model parameters
    a = refittedLineModel(1);
    b = refittedLineModel(2);
    c = refittedLineModel(3);

    % distance between each point and line
    denominator = sqrt(a^2 + b^2);
    for k = 1:numPoint
        distanceEachPoint(k) = abs((a*pointCloud(1,k) + b*pointCloud(2,k) + c) / (denominator));
    end

    % find the index of point that the distance with line is same or less than TH_DISTANCE
    % distance = |ax + by + c|/sqrt(a.^2 + b.^2)
    pointsIdxInThres = find(distanceEachPoint <= TH_DISTANCE);

end