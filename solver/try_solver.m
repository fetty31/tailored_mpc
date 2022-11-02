%% SIMULATION LOOP

addpath('/home/fetty/Desktop/control_ws2022/src/control/tro/TROm/Tracks/');
track = readtable('gro2_0.0490_0_501.csv');
curvature = track.curvature;

N = 40;             %Horizon length
Npar = 28;          %Number of real time params
delta_t = 0.025;    %Time discretization
n = 1000;           %Param to choose in which curvature we start
sampleS = 1;        %Points sample
its = 30;           %Number of iterations
latency = 5;        %Latency

opt_control = [];
opt_states = [];

controls = [];
states = [];

exitflags = zeros(its,1);

for e=1:its
    %param_data = textread('all_params.txt', '%f', 'delimiter', '\n');
    param_data = [];
    for i=1:N
        k = curvature(i*sampleS+n+N*(e-1));
        %param = [dRd, dRa, m, I,   Lf,    Lr,  Dr,    Df,     Cr,  Cf,   Br,      Bf,      u_r,  g,    Cd,   rho, Ar, q_slip, p_long, q_n, q_mu, lambda, q_s, ax_max, ay_max, Cm, dMtv, k]
        param_k = [1, 1, 240, 93, .708, .822, 3152.3, 2785.4, 1.6, 1.6, 10.1507, 10.8529, .045, 9.81, .8727, 1.255, 1, 1,     0.5,    10,  1,    3,    100,  10,      12,  4000, 1,   k]';
        param_data = [param_data,param_k];
    end
    
    % Upper and lower bounds of the problem (variables + constrains)
    % z_lb = [-deg2rad(2), -0.05, -deg2rad(23), -1, -3, -deg2rad(150), 0, -3, -deg2rad(100)]';
    % z_ub = [deg2rad(2), 0.05, deg2rad(23), 1, 3, deg2rad(150), 30, 3, deg2rad(100)]';
    
    z_lb = [-100, -1000, -100, -deg2rad(23), -1, -3, -deg2rad(50), 0, -3, -deg2rad(40)]';
    z_ub = [100, 1000, 100, deg2rad(23), 1, 3, deg2rad(50), 25, 3, deg2rad(40)]';
    
    lb = repmat(z_lb, N, 1);
    ub = repmat(z_ub, N, 1);
    
    % problem.lb = [lb(1:2);lb(10:end)];
    % problem.ub = [ub(1:2);ub(10:end)];
    
    problem.lb = lb;
    problem.ub = ub;
    
    %inequality constraints 
    problem.hu = repmat([1.5;1.5],N,1);
    
    x0 = (ub+lb)/2;
    
    if e==1
        problem.x0 = x0;
        problem.xinit = [0,0,0,deg2rad(5), 0.01, 0.01, 0.05, 10, .1, .01]';
    else
        problem.x0 = repmat([0,0,0,controls(5*(e-2)+4),controls(5*(e-2)+5),states(5*(e-2)+1),states(5*(e-2)+2),states(5*(e-2)+3),states(5*(e-2)+4),states(5*(e-2)+5)]',N,1);
        problem.xinit = [0,0,0,controls(5*(e-2)+4),controls(5*(e-2)+5),states(5*(e-2)+1),states(5*(e-2)+2),states(5*(e-2)+3),states(5*(e-2)+4),states(5*(e-2)+5)]';
    end

    problem.all_parameters = reshape(param_data(:,1:N),Npar*N,1);
    
    [output,exitflag,info] = TailoredSolver(problem);

    opt_control = [opt_control;output.U];
    opt_states = [opt_states;output.X];

    controls = [controls;output.U(5*latency+1:5*latency+5)];
    states = [states;output.X(5*latency+1:5*latency+5)];

    exitflags(e) = exitflag;
end

exitflags
fprintf('sum: ')
sum(exitflags)

%% Time vectors
% tcurv = 0:delta_t:delta_t*N*its;
tcurv = 0:its;
t = tcurv(2:end);

%% State plots
figure()
subplot(3,2,1)
plot(t,states(3:5:end));
title('Vx');
xlabel('t [s]');
ylabel('Vx [m/s]');

subplot(3,2,2)
plot(t,states(1:5:end))
title('n');
xlabel('t [s]');
ylabel('n [mm]');

subplot(3,2,3)
plot(t,states(4:5:end))
title('Vy');
xlabel('t [s]');
ylabel('Vy [m/s]');

subplot(3,2,4)
plot(t,states(5:5:end))
title('w');
xlabel('t [s]');
ylabel('w [rad/s]');
hold off

subplot(3,2,5:6)
plot(t,states(2:5:end))
title('mu')
xlabel('t [s]')
ylabel('mu [rad]')

%% Controls plot
figure()
subplot(2,2,1)
plot(t,controls(5:5:end))
title('F_m');
xlabel('t [s]');
ylabel('Fm');

subplot(2,2,2)
plot(t,controls(4:5:end))
title('delta');
xlabel('t [s]');
ylabel('delta [rad]');

subplot(2,2,3)
plot(t,controls(2:5:end))
title('diff F_m');
xlabel('t [s]');
ylabel('diff_Fm');

subplot(2,2,4)
plot(t,controls(1:5:end))
title('diff delta');
xlabel('t [s]');
ylabel('diff delta');
hold off

%% Curvature plot
figure()
plot(curvature)
xline(n)
xline(n+N*sampleS*e)
title('Curvature')
xlabel('t [s]')
ylabel('k [m⁻¹]')
