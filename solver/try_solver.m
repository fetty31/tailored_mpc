%% SIMULATION LOOP
track = readtable('gro2_0.0490_0_501.csv');
curvature = track.curvature;

N = 40;
Npar = 27;
delta_t = 0.025;
n = 500; %param to choose in which curvature we start

%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');
param_data = [];
for i=1:N
    k = curvature(i+n);
    %param = [dRd, dRa, m, I, Lf, Lr, Dr, Df, Cr, Cf, Br, Bf, u_r, g, Cd, rho, Ar, q_slip, p_long, q_n, q_mu, lambda, q_s, ax_max, ay_max, Cm, k]
    param_k = [10, 10, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, 1, 0.5, 5.5, 0.0, 3, 200, 10, 12, 5000, k]';
    param_data = [param_data,param_k];
end

% Upper and lower bounds of the problem (variables + constrains)
z_lb = [-deg2rad(3), -0.005, -deg2rad(23), -1, -3, -deg2rad(150), 0, -5, -deg2rad(100)]';
z_ub = [deg2rad(3), 0.005, deg2rad(23), 1, 3, deg2rad(150), 30, 5, deg2rad(100)]';

lb = repmat(z_lb, N, 1);
ub = repmat(z_ub, N, 1);

% problem.lb = [lb(1:2);lb(10:end)];
% problem.ub = [ub(1:2);ub(10:end)];

problem.lb = lb;
problem.ub = ub;

%inequality constraints 
problem.hu = repmat([1.5;1.5],N,1);

x0 = (ub+lb)/2; %+ ub./4;
problem.x0 = x0;

problem.xinit = [0.01, 0.1, 0.1, 0.01, 20, 0.5, 0.01]';
problem.xfinal = x0;

problem.all_parameters = reshape(param_data(:,1:N),Npar*N,1);

[output,exitflag,info] = TailoredSolver(problem);
exitflag

t = 0:delta_t:delta_t*N;

figure()
plot(curvature)
xline(n)
xline(n+N)
title('Curvature')
xlabel('t [s]')
ylabel('k [m⁻¹]')

t = t(2:end);

opt_control = output.U;
opt_states = output.X;

figure()
plot(t,opt_control(4:4:end))
title('Fm')
xlabel('t [s]')
ylabel('Fm [N]')

figure()
subplot(2,2,1)
plot(t,opt_states(3:5:end));
title('Vx');
xlabel('t [s]');
ylabel('Vx [m/s]');

subplot(2,2,2)
plot(t,opt_control(3:4:end))
title('Delta');
xlabel('t [s]');
ylabel('delta [rad]');

subplot(2,2,3)
plot(t,opt_states(4:5:end))
title('Vy');
xlabel('t [s]');
ylabel('Vy [m/s]');

subplot(2,2,4)
plot(t,opt_states(5:5:end))
title('w');
xlabel('t [s]');
ylabel('w [rad/s]');
hold off
