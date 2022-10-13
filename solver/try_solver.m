%% SIMULATION LOOP
track = readtable('gro2_0.0490_0_501.csv');
curvature = track.curvature;

N = 40;
Npar = 27;
delta_t = 0.025;
n = 900; %param to choose in which curvature we start
sampleS = 10;

%param_data = textread('all_params.txt', '%f', 'delimiter', '\n');
param_data = [];
for i=1:N
    k = curvature(i*sampleS+n);
    %param = [dRd, dRa, m, I, Lf, Lr, Dr, Df, Cr, Cf, Br, Bf, u_r, g, Cd, rho, Ar, q_slip, p_long, q_n, q_mu, lambda, q_s, ax_max, ay_max, Cm, k]
    param_k = [0, 0, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .45, 9.81, .8727, 1.255, 1, 1, 0.5, 0, 0, 3, 10, 10, 12, 5000, k]';
    param_data = [param_data,param_k];
end

% Upper and lower bounds of the problem (variables + constrains)
% z_lb = [-deg2rad(2), -0.05, -deg2rad(23), -1, -3, -deg2rad(150), 0, -3, -deg2rad(100)]';
% z_ub = [deg2rad(2), 0.05, deg2rad(23), 1, 3, deg2rad(150), 30, 3, deg2rad(100)]';

z_lb = [0.002, 0.05, -deg2rad(23), -1, -3, -deg2rad(150), 0, -3, -deg2rad(100)]';
z_ub = [0.002, 0.05, deg2rad(23), 1, 3, deg2rad(150), 30, 3, deg2rad(100)]';

lb = repmat(z_lb, N, 1);
ub = repmat(z_ub, N, 1);

% problem.lb = [lb(1:2);lb(10:end)];
% problem.ub = [ub(1:2);ub(10:end)];

problem.lb = lb;
problem.ub = ub;

%inequality constraints 
problem.hu = repmat([1.5;1.5],N,1);

x0 = (ub+lb)/2 + ub./3;
problem.x0 = x0;

problem.xinit = [deg2rad(1), 0.01, 0.01, 0.05, 25, .1, .01]';

problem.all_parameters = reshape(param_data(:,1:N),Npar*N,1);

[output,exitflag,info] = TailoredSolver(problem);
exitflag

tcurv = 0:delta_t:delta_t*N;

t = tcurv(2:end);

opt_control = output.U;
opt_states = output.X;

%% State plots
figure()
subplot(3,2,1)
plot(t,opt_states(3:5:end));
title('Vx');
xlabel('t [s]');
ylabel('Vx [m/s]');

subplot(3,2,2)
plot(t,opt_states(1:5:end))
title('n');
xlabel('t [s]');
ylabel('n [mm]');

subplot(3,2,3)
plot(t,opt_states(4:5:end))
title('Vy');
xlabel('t [s]');
ylabel('Vy [m/s]');

subplot(3,2,4)
plot(t,opt_states(5:5:end))
title('w');
xlabel('t [s]');
ylabel('w [rad/s]');
hold off

subplot(3,2,5:6)
plot(t,opt_states(2:5:end))
title('mu')
xlabel('t [s]')
ylabel('mu [rad]')

%% Controls plot
figure()
subplot(2,2,1)
plot(t,opt_control(4:4:end))
title('F_m');
xlabel('t [s]');
ylabel('Fm');

subplot(2,2,2)
plot(t,opt_control(3:4:end))
title('delta');
xlabel('t [s]');
ylabel('delta [rad]');

subplot(2,2,3)
plot(t,opt_control(2:4:end))
title('diff F_m');
xlabel('t [s]');
ylabel('diff_Fm');

subplot(2,2,4)
plot(t,opt_control(1:4:end))
title('diff delta');
xlabel('t [s]');
ylabel('diff delta');
hold off

%% Curvature plot
figure()
plot(curvature)
xline(n)
xline(n+N*sampleS)
title('Curvature')
xlabel('t [s]')
ylabel('k [m⁻¹]')
