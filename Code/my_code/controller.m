function [F, M, trpy, drpy] = controller(qd, t, params)

% position controller params
Kp = [3;3;30];
Kd = [2;2;15];

% attitude controller params
KpM = ones(3,1)*3000;
KdM = ones(3,1)*100;

acc_des = qd.acc_des + Kd.*(qd.vel_des - qd.vel) + Kp.*(qd.pos_des - qd.pos);

% Desired roll, pitch and yaw
phi_des = 1/params.grav * (acc_des(1)*sin(qd.yaw_des) - acc_des(2)*cos(qd.yaw_des));
theta_des = 1/params.grav * (acc_des(1)*cos(qd.yaw_des) + acc_des(2)*sin(qd.yaw_des));
psi_des = qd.yaw_des;

euler_des = [phi_des;theta_des;psi_des];
pqr_des = [0;0; qd.yawdot_des];
% Thurst
qd.acc_des(3);
F  = params.mass*(params.grav + acc_des(3));
% Moment
M =  params.I*(KdM.*(pqr_des - qd.omega) + KpM.*(euler_des - qd.euler));
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
