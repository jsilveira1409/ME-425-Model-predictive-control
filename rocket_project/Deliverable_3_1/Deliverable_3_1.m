addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
clc
Ts = 1/20;
Tf = 10; 
H = 3;

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%%
% initial state setup
% wy beta vx x
x0 = [0; 0; 0; 4];
% wx alpha vy y
y0 = [0; 0; 0; 4];
% vz z
z0 = [0; 4];
% wz gamma
roll0 = [0; deg2rad(45)];

%% X system
mpc_x = MpcControl_x(sys_x, Ts, H);
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0);

%open loop trajectory
U_opt(:, end+1) = nan;
phx = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);
%%
% closed loop
[T, X, Ux] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
phx = rocket.plotvis_sub(T, X, Ux, sys_x, xs, us);


%% Y system
mpc_y = MpcControl_y(sys_y, Ts, H);
[u, T_opt, Y_opt, U_opt] = mpc_y.get_u(y0);

%open loop trajectory
U_opt(:, end+1) = nan;
phy = rocket.plotvis_sub(T_opt, Y_opt, U_opt, sys_y, xs, us);
%%
% closed loop
[T, Y, Uy] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
phy = rocket.plotvis_sub(T, Y, Uy, sys_y, xs, us);


%% Z system
mpc_z = MpcControl_z(sys_z, Ts, H);
[u, T_opt, Z_opt, U_opt] = mpc_z.get_u(z0);

%open loop trajectory
U_opt(:, end+1) = nan;
phz = rocket.plotvis_sub(T_opt, Z_opt, U_opt, sys_z, xs, us);
%%
% closed loop
[T, Z, Uz] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
phx = rocket.plotvis_sub(T, Z, Uz, sys_z, xs, us);

%% Roll Syste
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
xroll = [3; 2];
[u, T_opt, ROLL_opt, U_opt] = mpc_roll.get_u(xroll);

%open loop trajectory
U_opt(:, end+1) = nan;
phz = rocket.plotvis_sub(T_opt, ROLL_opt, U_opt, sys_roll, xs, us);
%%
% closed loop
[T, ROLL, Uroll] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
phx = rocket.plotvis_sub(T, ROLL, Uroll, sys_roll, xs, us);