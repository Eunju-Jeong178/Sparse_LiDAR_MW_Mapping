function [maxMatchingIdx, maxMatchingNum, maxLineModel] = detectLineRANSAC(pointCloudRef, distance)
%--------------------------------------------------------------------------
% Description:
%   determine the input's 3D points (pointCloudRef) are on the same line or not
%
% Example:
%   OUTPUT :
%   maxMatchingIdx: the index of 2D points on the same line at maximum satisfaction.
%   maxMatchingNum: the number of 2D points on the same line at maximum satisfaction.
%   maxLineModel: the line model (a,b,c) at maximum satisfaction.
%
%   INPUT :
%   pointCloudRef: 3D feature points expressed in camera frame [m] - [x;y;z]
%   distance: threshold distance between point and the line [m]
%--------------------------------------------------------------------------

% initialize RANSAC model parameters
totalPointNum = size(pointCloudRef, 2);
samplePointNum = 2;
ransacMaxIterNum = 1000;
ransacIterNum = 50;
ransacIterCnt = 0;

maxMatchingNum = 0;
maxMatchingIdx = [];

% do RANSAC with line model
while (true)
    
    % sample 2 feature points
    [sampleIdx] = randsample(totalPointNum, samplePointNum);
    pointsSample = pointCloudRef(:,sampleIdx);
    P1 = pointsSample(1:2,1).';
    P2 = pointsSample(1:2,2).';
      
    % estimate line model parameters with 2 feature points
    normLineModel = estimate_line_model(P1, P2);
        
    % check number of inliers
    [matchingNum, goodMatchingIdx] = in_thres_for_line(pointCloudRef, normLineModel, distance);
        
    % save the large consensus set
    if (matchingNum > maxMatchingNum)
        maxMatchingNum = matchingNum;
        maxMatchingIdx = goodMatchingIdx;
        maxLineModel = normLineModel;
        
        % calculate the number of iterations (http://en.wikipedia.org/wiki/RANSAC)
        matchingRatio = matchingNum / totalPointNum;
        ransacIterNum = ceil(log(0.01)/log(1-(matchingRatio)^samplePointNum));
    end
    
    ransacIterCnt = ransacIterCnt + 1;
    if (ransacIterCnt >= ransacIterNum || ransacIterCnt >= ransacMaxIterNum)
        break;
    end
end

end






