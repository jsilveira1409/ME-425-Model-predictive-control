addpath(fullfile('..', 'src'));

%% TODO: This file should produce all the plots for the deliverable
 Ts = 1/20;

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% mpc subsystem controller initialization
% horizon length 
H = 10;

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

%merge
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%% simulate non linear rocket with the linear controllers
x0 = zeros(12, 1);
ref4 = [1 1 1 deg2rad(45)]';
[u, T_opt, X_opt, U_opt] = mpc.get_u(x0, ref4);
U_opt(:, end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4);

%%

% setup ref function
ref = @(t_, x_) ref_EPFL(t_);

%simulatey
Tf = 30;
[T,X,U,Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

rocket.anim_rate = 5;
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Merged lin. MPC in non linear simulation';