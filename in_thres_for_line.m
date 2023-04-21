function [matchingNum, goodMatchingIdx] = inThresforLine(pointCloudRef, normLineModel, distance)
%-------------------------------------------------------------------------- 
% Description:
%   get the index of 2D points satisfying [Distance] < [Threshold],
%   'distance' means the length between the 2D points and the line.
%
%   OUTPUT :
%   matchingNum: the number of 3D points satisfying [DIstance] < [Threshold]
%   goodMatchingIdx: index of 3D points satisfying [DIstance] < [Threshold]
%
%   INPUT :
%   pointCloudRef : 3D feature points expressed in camera frame [m] - [x;y;z]
%   normLineModel: normalized line model - [a,b,c]
%                           'a*x + b*y + c = 0'
%   distance: threshold distance [m] between the point and the line
%--------------------------------------------------------------------------


% set parameters
numPoint = size(pointCloudRef, 2);
distanceEachPoint = zeros(1, numPoint);


% assign line model parameters
a = normLineModel(1);
b = normLineModel(2);
c = normLineModel(3);


% distance between each point and line
denominator = sqrt(a^2 + b^2);
for k = 1:numPoint
    distanceEachPoint(k) = abs((a*pointCloudRef(1,k) + b*pointCloudRef(2,k) + c) / (denominator));
end


% determine inlier or not
matchingNum = sum(distanceEachPoint <= distance);
goodMatchingIdx = find(distanceEachPoint <= distance);


end
