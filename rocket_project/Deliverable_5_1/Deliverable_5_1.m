clear
addpath(fullfile('..', 'src'));


%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
x0 = zeros(12,1);
Tf = 30;

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
ref = @(t_, x_) ref_EPFL(t_);

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 10;

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
lmpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);


%% NO mass
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
rocket.anim_rate = 1;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lineare MPC in nonlinear simulation without mass'

%% 
rocket.mass = 1.794;
% Impact of changing the mass on system of part 5
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, mpc, ref, mpc_z, sys_z);

rocket.anim_rate = 3; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation, trajectory with disturbance and offset-free tracking'; % Set a figure title


 %% Plot Comparing
figure;
hold on
plot(T,X_nominal(12,:),'r');
plot(T,X_changed(12,:),'c');
plot(T,X(12,:),'b');
plot(T,abs(X_nominal(12,:)-X(12,:)),'g');
xlabel('Time [s]');
ylabel('z(t) [m]');
legend('Nominal trajectory along z','Trajectory with disturbance along z','Trajectory with disturbance and offset-free tracking along z','\Delta z = nominal trajectory - offset-free tracking');
grid on
hold off



