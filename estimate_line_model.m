function [normLineModel] = estimateLineModel(P1, P2)
%--------------------------------------------------------------------------
% Description:
%   get the line model parameters [a,b,c] <= 'a*x + b*y + c = 0'
%
%   OUTPUT :
%   normLineModel: normalized line model parameters - [a,b,c]
%
%   INPUT :
%   P1 : Point #1 on the line [X,Y];
%   P2 : Point #2 on the line [X,Y];
%--------------------------------------------------------------------------

% calculate the [a, b, c] in the line equation
a = P2(2) - P1(2);
b = P1(1) - P2(1);
c = P2(1) * P1(2) - P1(1) * P2(2);

% calculate the model parameters of the line equation
lineModel = [a, b, c];
normLineModel = lineModel ./ (min(abs(lineModel)));


end
