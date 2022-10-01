function [model, codeoptions] = generate_solver(solverDir, rkTime, horizonLength, useFastSolver, floattype)
    %% Check function args    
    if (nargin < 3)
        error('The function ''generatePathTrackingSolver'' is not meant to run standalone! Please run ''PathTracking'' instead.');
    end
    if nargin < 4
        useFastSolver = false;
    end
    if nargin < 5
        floattype = 'double';
    end

    %% Problem dimensions
    N = horizonLength;
    n_states = 7;
    n_controls = 2;
    Npar = 24;
    model = {};
    model.N = N;             % horizon length
    model.nvar = 9;          % number of variables
    model.neq  = n_states;   % number of equality constraints
    model.nh = 4;            % number of inequality constraint functions
    model.npar = Npar;       % number of runtime parameters

    %% Runge Kutta integration time
    global Ts
    Ts = rkTime;

    %% Objective function 
    model.objective = @objective;
    
    %% Dynamics, i.e. equality constraints 
    model.eq = @integrated_dynamics;
    
    % Nonlinear inequalities
    model.ineq = @nonlin_const;
    model.huidx = (1:model.nh)';
    model.hlidx = [];
    model.hu = [];
    model.hl = [];
    
    % Indices on LHS of dynamical constraint - for efficiency reasons, make
    % sure the matrix E has structure [0 I] where I is the identity matrix.
    model.E = [zeros(n_states,n_controls), eye(n_states)];
    
    %% Inequality constraints
    % upper/lower variable bounds lb <= z <= ub
    %          inputs          |             states
    % z = [diff_delta, delta_Fm, delta, Fm, n, mu, vx, vy, w]    
    model.lbidx = [1, 2, 3, 4, 5, 6, 7, 8, 9]';
    model.ubidx = [1, 2, 3, 4, 5, 6, 7, 8, 9]';
    model.lb = []; 
    model.ub = [];
    
    %% Initial conditions
    % Initial conditions on all states
    model.xinitidx = 3:9; % use this to specify on which variables initial conditions are imposed
    
    %% Define solver options
%     if useFastSolver
%         codeoptions = ForcesGetDefaultOptions('TailoredSolver','SQP_NLP_fast',floattype);
%     else
%         codeoptions = ForcesGetDefaultOptions('TailoredSolver','SQP_NLP',floattype);
%     end

    codeoptions = getOptions('TailoredSolver');

    codeoptions.maxit = 150;    % Maximum number of iterations
    codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions.platform = 'Gnu-x86_64'; % Specify the platform
    codeoptions.printlevel = 0; % Optional, on some platforms printing is not supported
    codeoptions.cleanup = 1; % To keep necessary files for target compile
    
    % Necessary to set bounds dynamically
    codeoptions.nlp.stack_parambounds = 1;
    codeoptions.dump_formulation = 1;
    
    % Speed up the solver
%     codeoptions.nlp.checkFunctions = 0;
%     codeoptions.nlp.linear_solver = 'symm_indefinite_fast';
%     
%     codeoptions.nlp.BarrStrat = 'monotone';

    codeoptions.overwrite = 1;      % Overwrite existing solver
    codeoptions.BuildSimulinkBlock = 0;
    codeoptions.nohash = 1;         % Enforce solver regeneration

%     codeoptions.parallel = 1; % Internal Parallelization
%     codeoptions.nlp.max_num_threads = 5; % When using code generated integrators (RK4) we must specify the maximum number of threads available

    %% Generate FORCESPRO solver
    cd(solverDir);
    
end
    
    %% Eval and dynamics functions
    
function f = objective(z, p)

    dRd = p(1);
    dRa = p(2);
    Lf = p(5);
    Lr = p(6);
    q_slip = p(18);
    q_n = p(20);
    q_s = p(23);
    q_mu = p(21);
    k = p(24);
    
    % Progress rate
    sdot = (z(7)*cos(z(6)) - z(8)*sin(z(6)))/(1 - z(5)*k); % == (vx*cos(mu) - vy*sin(mu))/(1 - n*k)

    % Slip difference
    beta_dyn = atan(z(8)/z(7));
    beta_kin = atan(z(3)*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;

    %Objective function
    f = -q_s*sdot + dRd*(z(1))^2 + dRa*(z(2))^2 + q_slip*(diff_beta)^2 + q_mu*(z(6))^2 + q_n*(z(5))^2;
    
end


function xnext = integrated_dynamics(z, p)

    global Ts
%     Ts = 25e-3;
    u = z(1:2);
    x = z(3:9);
  
    xnext = RK4(x, u, @my_continuous_dynamics, Ts, p);
end

function xdot = my_continuous_dynamics(x, u, p)
    
    delta = x(1);
    Fm = x(2);
    n = x(3);
    mu = x(4);
    vx = x(5);
    vy = x(6);
    w = x(7);
    
    diff_delta = u(1);
    diff_Fm = u(2);
    
    m = p(3);
    I = p(4);
    Lf = p(5);
    Lr = p(6);
    Dr = p(7);
    Df = p(8);
    Cr = p(9);
    Cf = p(10);
    Br = p(11);
    Bf = p(12); 
    u_r = p(13);
    g = p(14);
    Cd = p(15);
    rho = p(16);
    Ar = p(17);
    k = p(24);
    
    % Slip angles
    alpha_R = atan((vy-Lr*w)/(vx));
    alpha_F = atan((vy+Lf*w)/(vx)) - delta;
    
    % Simplified Pacejka magic formula
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));
    Fx = Fm*(1+cos(delta)) - m*u_r*g - 0.5*Cd*rho*Ar*vx^2;
    
    
    %Progress rate change
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Differential equations (time dependent)
    xdot = [diff_delta;
            diff_Fm;
            (vx*sin(mu) + vy*cos(mu));
            w - k*sdot;
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w);
            (1/m)*(Fr + Fm*sin(delta) + Ff*cos(delta) - m*vx*w);
            (1/I)*((Ff*cos(delta) + Fm*sin(delta))*Lf - Fr*Lr)];
    
end

function h = nonlin_const(z, p)

    m = p(3);
    Lf = p(5);
    Lr = p(6);
    Dr = p(7);
    Df = p(8);
    Cr = p(9);
    Cf = p(10);
    Br = p(11);
    Bf = p(12); 
    
    delta = z(3);
    Fm = z(4);
    n = z(5);
    mu = z(6);
    vx = z(7);
    vy = z(8);
    w = z(9);
    
    % Length and width of the car
    long = 2.72;
    width = 1.2 + 0.4;

    % Maximum longitudinal & lateral acceleration
    Ax_max = 5.5;
    Ay_max = 8;
    
    alpha_R = atan((vy-Lr*w)/(vx));
    alpha_F = atan((vy+Lf*w)/(vx)) - delta;
    
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));

    Fx = Fm*(1+cos(delta));
    
    h = [(Fx/(Ax_max*m))^2 + (Fr/(Ay_max*m))^2; % <= lambda
         (Fx/(Ax_max*m))^2 + (Ff/(Ay_max*m))^2; % <= lambda
         n - long/2*sin(abs(mu)) + width/2*cos(mu);
         -n + long/2*sin(abs(mu)) + width/2*cos(mu)];
     
end