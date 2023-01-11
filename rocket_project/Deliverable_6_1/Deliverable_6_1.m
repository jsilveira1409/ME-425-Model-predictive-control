addpath(fullfile('..', 'src'));


%% NMPC
Ts = 1/10;
Tf = 30;
x0 = zeros(12,1);
H = 10;

rocket = Rocket (Ts); 

nmpc = NmpcControl(rocket, H);

ref = @(t_, x_) ref_EPFL(t_);

x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

rocket.anim_rate = 4; 
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'MPC in nonlinear simulation'; % Set a figure title

%% Linear MPC
Ts = 1/20;
H = 10;
Tf = 30;
 x0 = zeros(12,1);


rocket = Rocket (Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

mpc_x = MpcControl_x(sys_x, Ts , H);
mpc_y = MpcControl_y(sys_y, Ts , H);
mpc_z = MpcControl_z(sys_z, Ts , H);
mpc_roll = MpcControl_roll(sys_roll, Ts , H);
% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

roll_max = deg2rad(50);
ref = @(t_, x_) ref_EPFL(t_, roll_max);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
rocket.anim_rate = 3; -
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation with gamma to 50°'; 



