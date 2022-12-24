addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
clc
Ts = 1/20;
Tf = 10; 
H = 5;

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% 
% initial state setup
% wy beta vx x
x0 = [0; 0; 0; 0];
% wx alpha vy y
y0 = [0; 0; 0; 0];
% vz z
z0 = [0; 0];
% wz gamma
roll0 = [0; 0];


% reference target setup

ref_y = -3;
ref_z = -3;
ref_roll = deg2rad(35);

%% X system
mpc_x = MpcControl_x(sys_x, Ts , H);
% closed loop
ref_x = 4;
[T, X, Ux] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref_x);
phx = rocket.plotvis_sub(T, X, Ux, sys_x, xs, us, ref_x);

%% Y system
mpc_y = MpcControl_y(sys_y, Ts, H);
% closed loop
ref_y = 6;
[T, Y, Uy] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, ref_y);
phy = rocket.plotvis_sub(T, Y, Uy, sys_y, xs, us, ref_y);

%% Z system
mpc_z = MpcControl_z(sys_z, Ts, H);

% closed loop
[T, Z, Uz] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, ref_z);
phz = rocket.plotvis_sub(T, Z, Uz, sys_z, xs, us, ref_z);


%% Roll system
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% closed loop
[T, ROLL, Uroll] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, ref_roll);
phroll = rocket.plotvis_sub(T, ROLL, Uroll, sys_roll, xs, us, ref_roll);










