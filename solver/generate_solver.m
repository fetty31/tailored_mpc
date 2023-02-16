function [model, codeoptions] = generate_solver(solverDir, horizonLength, n_states, n_controls, useFastSolver, floattype)
    %% Check function args    
    if (nargin < 3)
        error('The function ''generatePathTrackingSolver'' is not meant to run standalone! Please run ''FORCES_problem'' instead.');
    end
    if nargin < 6
        useFastSolver = false;
    end
    if nargin < 7
        floattype = 'double';
    end

    %% Problem dimensions
    N = horizonLength;
    Npar = 25;
    model = {};
    model.N = N;                        % horizon length
    model.nvar = n_states+n_controls;   % number of variables
    model.neq  = n_states;              % number of equality constraints
    model.nh = 2;                       % number of inequality constraint functions
    model.npar = Npar;                  % number of runtime parameters

    %% Objective function 
    model.objective = @objective;
    
    %% Dynamics, i.e. equality constraints 
    model.eq = @integrated_dynamics;
%     model.continuous_dynamics = @my_continuous_dynamics;
    
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
    % z = [slack_track, diff_delta, Mtv, delta, n, mu, vy, r]    
    model.lbidx = [1, 2, 3, 4, 5, 6, 7, 8]';
    model.ubidx = [2, 3, 4, 5, 6, 7, 8]';
    model.lb = []; 
    model.ub = [];
    
    %% Initial conditions
    % Initial conditions on all states
    model.xinitidx = 2:8; % use this to specify on which variables initial conditions are imposed

    %% Linear subsystem
%     model.linInIdx = [1, 2]';
    
    %% Define solver options

    % Define integrator
%     codeoptions.nlp.integrator.type = 'ERK4';
%     codeoptions.nlp.integrator.Ts = rkTime;
%     codeoptions.nlp.integrator.nodes = 1;
%     codeoptions.nlp.integrator.differentiation_method = 'chainrule';
    
    codeoptions = getOptions('TailoredSolver');

    codeoptions.maxit = 150;    % Maximum number of iterations
    codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
    codeoptions.platform = 'Gnu-x86_64'; % Specify the platform
    codeoptions.printlevel = 0; % Optional, on some platforms printing is not supported
    codeoptions.cleanup = 0; % To keep necessary files for target compile
    
    % Necessary to set bounds dynamically
    codeoptions.nlp.stack_parambounds = 1;
    codeoptions.dump_formulation = 1;
    
    % Speed up the solver
    codeoptions.nlp.checkFunctions = 0;
    codeoptions.nlp.linear_solver = 'symm_indefinite_fast'; % 'symm_indefinite'
    
    codeoptions.nlp.BarrStrat = 'monotone'; % 'loqo' (default). Strategy for updating the barrier parameter
%     codeoptions.nlp.hessian_approximation = 'bfgs'; % hessian approximation ('gauss-newton' also supported)

    codeoptions.overwrite = 1;      % Overwrite existing solver
    codeoptions.BuildSimulinkBlock = 0;
    codeoptions.nohash = 1;         % Enforce solver regeneration

%     codeoptions.nlp.integrator.attempt_subsystem_exploitation = 1; % exploit possible linear subsystems

    codeoptions.init = 0; % Solver initialization method (0: cold start; 1: centered start; 2: primal warm start; see https://forces.embotech.com/Documentation/solver_options/index.html#compiler-optimization-level)

    codeoptions.parallel = 1; % Internal Parallelization
%     codeoptions.nlp.max_num_threads = 5; % When using code generated integrators (RK4) we must specify the maximum number of threads available
        
    % Embotech solution for server error
%     codeoptions.legacy_integrators = 1;

    %% Generate FORCESPRO solver
    cd(solverDir);
    
end
    
    %% Eval and dynamics functions
    
function f = objective(z, p) 

    dRd = p(1);
    Lf = p(4);
    Lr = p(5);
    q_slip = p(17);
    q_n = p(18);
    q_s = p(20);
    q_mu = p(19);
    q_Mtv = p(21);
    Ts = p(22);
    k = p(25);
    q_slack_track = p(23);

    vx = p(24);
    delta = z(4);
    n = z(5);
    mu = z(6);
    vy = z(7);
    
    % Progress rate
    sdot = Ts * (vx*cos(mu) - vy*sin(mu))/(1 - n*k);

    % Slip difference
    beta_dyn = atan(vy/vx);
    beta_kin = atan(delta*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;

    %Objective function
    f = -q_s*sdot + dRd*(z(2))^2 + q_Mtv*(z(3))^2 + q_slip*(diff_beta)^2 + q_mu*(mu)^2 + q_n*(n)^2 + q_slack_track*z(1);
    
end


function xnext = integrated_dynamics(z, p)

    u = z(1:3);
    x = z(4:8);
    Ts = p(22);

    xnext = RK4(x, u, @my_continuous_dynamics, Ts, p);

end

function xdot = my_continuous_dynamics(x, u, p)
    
    delta = x(1);
    n = x(2);
    mu = x(3);
    vx = p(24);
    vy = x(4);
    r = x(5);
    
    diff_delta = u(2);
    Mtv = u(3);
    
    m = p(2);
    I = p(3);
    Lf = p(4);
    Lr = p(5);
    Dr = p(6);
    Df = p(7);
    Cr = p(8);
    Cf = p(9);
    Br = p(10);
    Bf = p(11); 
    u_r = p(12);
    g = p(13);
    Cd = p(14);
    rho = p(15);
    Ar = p(16);
    k = p(25);
    
    % Slip angles
    alpha_R = atan((vy-Lr*r)/(vx));
    alpha_F = atan((vy+Lf*r)/(vx)) - delta;
    
    % Simplified Pacejka magic formula
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));
    
    %Progress rate change
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Differential equations (time dependent)
    xdot = [diff_delta;
            vx*sin(mu) + vy*cos(mu);
            r - k*sdot;
            (1/m)*(Fr + Ff*cos(delta) - m*vx*r);
            (1/I)*(Ff*cos(delta)*Lf - Fr*Lr + Mtv)];
end

function h = nonlin_const(z, p)

    s1 = z(1);
    n = z(5);
    mu = z(6);
    
    % Length and width of the car
    long = 2.72;
    width = 1.2 + 0.4;

    h = [ n - long/2*sin(abs(mu)) + width/2*cos(mu) - s1;  % <= L(s)
         -n + long/2*sin(abs(mu)) + width/2*cos(mu) - s1]; % <= R(s) 
     
end