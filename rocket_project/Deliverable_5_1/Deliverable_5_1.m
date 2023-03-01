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
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

rocket.anim_rate = 3;
%% Original System, no mass change(as in part 4)
[T1, X1, U1, Ref1] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
%ph = rocket.plotvis(T1, X1, U1, Ref1);
ph.fig.Name = 'Merged lineare MPC in nonlinear simulation without mass'

%% Original System with mass change
rocket.mass = 1.794;
[T2, X2, U2, Ref2] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
%ph = rocket.plotvis(T2, X2, U2, Ref2);
ph.fig.Name = 'Merged lineare MPC in nonlinear simulation with disturbance'

%% Original System with Estimator, and mass change
[T3, X3, U3, Ref3, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
%ph = rocket.plotvis(T3, X3, U3, Ref3);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation, trajectory with disturbance and offset-free tracking';


 %% Plot Comparing
figure;
hold on
plot(T1,X1(12,:),'r');
plot(T2,X2(12,:),'c');
plot(T3,X3(12,:),'b');
plot(T1,abs(X1(12,:)-X3(12,:)),'g');

xlabel('Time [s]');
ylabel('z(t) [m]');

legend('Nominal trajectory along z','Trajectory with disturbance along z','Trajectory with disturbance and offset-free tracking along z','\Delta z = nominal trajectory - offset-free tracking');
grid on
hold off



