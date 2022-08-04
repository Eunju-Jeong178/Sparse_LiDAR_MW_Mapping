%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
%
% Description : Find the angle between the two straight lines obtained
%               by RANSAC
% 
% Input 
% 1) m1 : slope of the reference straight line 
%         (with the highest number of points) 
% 2) m2 : slope of the straight line to compare
%
% Output 
% 1) thetaInDegrees : the angle between the two straight lines [deg]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% reference)
% https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=post_human&logNo=120171970762

function thetaInDegrees = angleBetweenTwoVectors(m1,m2)

    u1 = [1 m1]; % the reference vector
    u2 = [1 m2]; % the vector to compare

    cosTheta = max(min(dot(u1,u2)/(norm(u1)*norm(u2)),1),-1);
    thetaInDegrees = real(acosd(cosTheta));

end