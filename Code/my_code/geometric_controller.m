function [F, M, trpy, drpy] = geometric_controller(qd, t, params)

%Current State
r=qd.pos;
dr=qd.vel;
Omiga=qd.omega;

R=qd.Rot;
z_B=R(:,3);

% Desire State
r_des=qd.pos_des;
dr_des=qd.vel_des;
psiC=qd.yaw_des;

%PID Parameter
% Kp_x=0.8;
% Kd_x=0.5;
% Kp_y=0.8;
% Kd_y=0.5;
% Kp_z=1;
% Kd_z=0.5;
Kp_x=5.6;
Kd_x=3.5;
Kp_y=5.6;
Kd_y=3.5;
Kp_z=30;
Kd_z=15;
%
% Kp_phi=3000;  Kd_phi=100;
% Kp_theta=3000; Kd_theta=100;
% Kp_psi=3000; Kd_psi=100;
Kp_phi=4100;  Kd_phi=205;
Kp_theta=4100; Kd_theta=205;
Kp_psi=4100; Kd_psi=200;
%
Kp=[Kp_x,0,0;0,Kp_y,0;0,0,Kp_z];
Kv=[Kd_x,0,0;0,Kd_y,0;0,0,Kd_z];
K_R=[Kp_phi,0,0;0,Kp_theta,0;0,0,Kp_psi];
K_omiga=[Kd_phi,0,0;0,Kd_theta,0;0,0,Kd_psi];

z_w=[0,0,1]';

ep=r-r_des;
ev=dr-dr_des;

F_des=-Kp*ep-Kv*ev+params.mass*params.grav*z_w;
F=F_des'*z_B;

z_B_des=F_des/norm(F_des);
y_C_des=[-sin(psiC),cos(psiC),0]';

x_B_des=cross(y_C_des,z_B_des)/norm(cross(y_C_des,z_B_des));
y_B_des=cross(z_B_des,x_B_des);
R_des=[x_B_des,y_B_des,z_B_des];

[phiC,thetaC,psiC] = RotToRPY_ZYX(R_des);


e_R=1/2*veemap(R_des'*R-R'*R_des);

% h_omiga=m/F*()
omiga_x_des=0;
omiga_y_des=0;
omiga_z_des=0;

e_omiga=[Omiga(1)-omiga_x_des,Omiga(2)-omiga_y_des,Omiga(3)-omiga_z_des]';

M=params.I*(-K_R*e_R-K_omiga*e_omiga)+cross(Omiga, params.I*Omiga);

% Output trpy and drpy as in hardware
trpy = [F, phiC, thetaC, psiC];
drpy = [0, 0,       0,         0];
end

function vector=veemap(cross_matrix)

vector(1) = -cross_matrix(2,3);
vector(2) = cross_matrix(1,3);
vector(3) = -cross_matrix(1,2);
vector=vector';
end

