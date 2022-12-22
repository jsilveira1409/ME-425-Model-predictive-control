Ts = 0.1;
rocket = Rocket(Ts);
Tf = 2.0;

x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]';
u = [deg2rad([2 0]), 60, 0 ]';
[T, X, U] = rocket.simulate(x0, Tf, u);
rocket.anim_rate = 0.1;
rocket.vis(T, X, U);