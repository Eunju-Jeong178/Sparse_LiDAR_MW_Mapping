%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_ransac(points_cluster)
    
    save pointcloud_cluster_1x2.mat points_cluster;
    load pointcloud_cluster_1x2.mat
    plot(points_cluster(:,1),points_cluster(:,2),'.'); % x, y
    hold on

    %%
    ceiling_height = 2.5; %****************************** (up distance + down distance)

    %%
    % Fit a line to the points using the MSAC algorithm.
    % Define the sample size, the maximum distance for inliers,
    % the fit funciton, and the distance evaluation function.
    % Call ransac to run the MSAC algorithm.

    sampleSize = 2; % minimum sample size from data that is required by fitFcn, specified as a positive scalar integer.
    maxDistance = 0.05; % max allowable distance for inliers (positive scalar) ************************

    fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
    
    % distance evaluation function
    evalLineFcn = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);

    % inlierIdx: logical 1D array (num_cluster_index x 1), (1: inlier, 0: outlier)
    [modelRANSAC, inlierIdx] = ransac(points_cluster,fitLineFcn,evalLineFcn,sampleSize,maxDistance); % do RANSAC

    %%
    % Refit a line to the inlier using ployfit.

    modelInliers = polyfit(points_cluster(inlierIdx,1),points_cluster(inlierIdx,2),1);

    %%
    % Display the final fit line. This is robust to the outliers that ransac
    % identified and ignored.

    inlierPts = points_cluster(inlierIdx,:); % inlier points in points_cluster
    x = [min(inlierPts(:,1)) max(inlierPts(:,1))]; % range of x, the size of line
    %x = [-1 1]; % range of x
    y = modelInliers(1)*x + modelInliers(2); % y = ax + b, 기울기: modelInliers(1)
   
    plot(x, y, 'b-', 'LineWidth',2) % Robust fit
    %legend('Points','Robust fit');
    hold on

    %% plot_plane

    % x, y, z of plane 1)
    x1 = min(inlierPts(:,1)); 
    y1 = modelInliers(1)*min(inlierPts(:,1)) + modelInliers(2); 
    z1 = 0;
    
    % x, y, z of plane 2)
    x2 = max(inlierPts(:,1));
    y2 = modelInliers(1)*max(inlierPts(:,1)) + modelInliers(2);
    z2 = 0;

    % x, y, z of plane 3)
    x3 = max(inlierPts(:,1));
    y3 = modelInliers(1)*max(inlierPts(:,1)) + modelInliers(2);
    z3 = ceiling_height;

    
    % x, y, z of plane 4)
    x4 = min(inlierPts(:,1)); 
    y4 = modelInliers(1)*min(inlierPts(:,1)) + modelInliers(2); 
    z4 = ceiling_height;

    %plot_plane(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4) % plot plane
    %legend('Points','Robust fit','Wall');

end
