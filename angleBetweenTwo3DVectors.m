%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
%
% Description : Find the angle between the two 3D vectors
% 
% Input 
% 1) MF_X, MF_Y : slope of the Manhattan frame (X,Y axis) 
% 2) slope_line : slope of the straight line to compare
%
% Output 
% 1) thetaInDegrees : the angle between the two 3D vectors [deg]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function thetaInDegrees = angleBetweenTwo3DVectors(MF_X,slope_line)

    u1 = MF_X; % Manhattan frame
    u2 = [1 slope_line 0]; % the vector to compare

    cosTheta = max(min(dot(u1,u2)/(norm(u1)*norm(u2)),1),-1);
    thetaInDegrees = real(acosd(cosTheta)); % [degree]

end
