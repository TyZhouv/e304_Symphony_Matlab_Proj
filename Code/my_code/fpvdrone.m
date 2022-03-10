function params = fpvdrone()

% m = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
m = 1.5;
g = 9.81;   % gravitational constant
% I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
%      0,         1.43e-5,    0;
%      0,         0,          2.89e-5];
I = [0.029125,   0,          0; % inertial tensor in m^2 kg
     0,         0.029125,    0;
     0,         0,          0.055225];
L = 0.046; % arm length in m
propr=0.02;
fov=0.08;

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = L;
params.propr=propr;
params.fov=fov;

params.maxangle = 180*pi/180; % you can specify the maximum commanded angle here
params.maxF     = 2.5*m*g;   % left these untouched from the nano plus
params.minF     = 0.05*m*g;  % left these untouched from the nano plus

% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m

end
