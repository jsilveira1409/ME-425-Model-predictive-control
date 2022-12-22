Ts = 0.1;
rocket = Rocket(Ts);
[xs, us] = rocket.trim()
sys = rocket.linearize(xs, us);