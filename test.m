Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 50;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_x, Ts, H);
mpc_z = MpcControl_z(sys_x, Ts, H);
mpc_roll = MpcControl_roll(sys_x, Ts, H);


[u_x,T_opt, X_opt, U_opt] = mpc_x.get_u(x);
U_opt(:, end+1) = nan;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);
