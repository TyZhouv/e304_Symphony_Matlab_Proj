function R = RPYtoRot_ZYX(roll,pitch,yaw)
%RPYtoRot_ZYX Converts roll, pitch, yaw to a body-to-world Rotation matrix
%   The rotation matrix in this function is body to world [wRb] 
%   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
%   is a point in the world frame
R = [cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),...
          sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);...
         sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *...
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);...
         -sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll)];

end
