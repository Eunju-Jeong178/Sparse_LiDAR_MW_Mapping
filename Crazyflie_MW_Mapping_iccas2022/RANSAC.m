%%
% Load and plot a set of noisy 2-D points
clear all; clc;

%% common setting to read text files
delimiter = ','; % txt file에서 comma 제거
headerlinesIn = 1;

%textGlobalPointCloudYFileDir = 'input\0620_B2_straignt_4direction_noyaw_pyresult_globalpointcloud_1x3.txt';
textGlobalPointCloudYFileDir = 'input\0618_B2_straight_02_pyresult_globalpointcloud_1x3.txt';

textGlobalPointCloudData = importdata(textGlobalPointCloudYFileDir, delimiter, headerlinesIn);
points = textGlobalPointCloudData.data(:,[1:2]); % x,y
points3D = textGlobalPointCloudData.data(:,[1:3]); % x,y,z
numPoints = size(points,1);

save pointcloud_1x2.mat points; % save 2D point cloud as mat file

rng('default'); % For reproducibility

load pointcloud_1x2.mat
plot(points(:,1),points(:,2),'o'); % x, y
hold on

%%
% Fit a line to the points using the MSAC algorithm.
% Define the sample size, the maximum distance for inliers,
% the fit funciton, and the distance evaluation function.
% Call ransac to run the MSAC algorithm.

sampleSize = 2; % number of points to sample per trial
maxDistance = 2; % max allowable distance for inliers

fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
evalLineFcn = ...   % distance evaluation function
  @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);

% fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
% evalLineFcn = ...   % distance evaluation function
%   @(model, points) sum((points([200:250], 2) - polyval(model, points([200:250],1))).^2,2);

[modelRANSAC, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...
  sampleSize,maxDistance);
%%
% Refit a line to the inlier using ployfit.

modelInliers = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);
%%
% Display the final fit line. This is robust to the outliers that ransac
% identified and ignored.

inlierPts = points(inlierIdx,:);
x = [min(inlierPts(:,1)) max(inlierPts(:,1))];
y = modelInliers(1)*x + modelInliers(2); % 일차 
plot(x, y, 'g-','LineWidth',3) % Robust fit
legend('Points','Robust fit');
hold off