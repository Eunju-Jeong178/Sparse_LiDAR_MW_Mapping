function plot_body_frame(R_gc, p_gc)
%--------------------------------------------------------------------------
% Description:
%   draw current ARKit camera frame in the current figure
%   INPUT:
%   R_gc: rotation matrix of camera frame (T_gc(1:3,1:3))
%   p_gc: position vector of camera frame (T_gc(1:3,4))
%--------------------------------------------------------------------------


% translate the camera body and frame
camFrame = R_gc + [p_gc(1:3) p_gc(1:3) p_gc(1:3)];

% draw the camera frame
line([camFrame(1,1) p_gc(1)],[camFrame(2,1) p_gc(2)],[camFrame(3,1) p_gc(3)],'Color','r','LineWidth',2) % x axis
line([camFrame(1,2) p_gc(1)],[camFrame(2,2) p_gc(2)],[camFrame(3,2) p_gc(3)],'Color','g','LineWidth',2) % y axis
line([camFrame(1,3) p_gc(1)],[camFrame(2,3) p_gc(2)],[camFrame(3,3) p_gc(3)],'Color','b','LineWidth',2) % z axis

end