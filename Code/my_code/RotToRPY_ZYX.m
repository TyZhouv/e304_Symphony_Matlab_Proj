function [phi,theta,psi] = RotToRPY_ZYX(R)
%RotToRPY_ZYX Extract Roll, Pitch, Yaw from a world-to-body Rotation Matrix
%   The rotation matrix in this function is body to world [wRb]  
%   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and
%   [wP] is a point in the world frame

%   R = [cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),...
%           sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);...
%          sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *...
%           sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);...
%          -sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll)];

theta = asin(-R(3,1));
phi = atan2(R(3,2),R(3,3));
psi = atan2(R(2,1),R(1,1));


end
