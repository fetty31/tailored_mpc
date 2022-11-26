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
    Npar = 31;
    model = {};
    model.N = N;                        % horizon length
    model.nvar = n_states+n_controls;   % number of variables
    model.neq  = n_states;              % number of equality constraints
    model.nh = 5;                       % number of inequality constraint functions
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
    % z = [v_slack, track_slack, diff_delta, delta_Fm, Mtv, delta, Fm, n, mu, vx, vy, w]    
    model.lbidx = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]';
    model.ubidx = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12]';
    model.lb = []; 
    model.ub = [];
    
    %% Initial conditions
    % Initial conditions on all states
    model.xinitidx = 3:12; % use this to specify on which variables initial conditions are imposed

    %% Linear subsystem
%     model.linInIdx = [1, 2]';
    
    %% Define solver options
%     if useFastSolver
%         codeoptions = ForcesGetDefaultOptions('TailoredSolver','SQP_NLP_fast',floattype);
%     else
%         codeoptions = ForcesGetDefaultOptions('TailoredSolver','SQP_NLP',floattype);
%     end

    codeoptions = getOptions('TailoredSolver');

    % Define integrator
%     codeoptions.nlp.integrator.type = 'ERK4';
%     codeoptions.nlp.integrator.Ts = rkTime;
%     codeoptions.nlp.integrator.nodes = 1;
%     codeoptions.nlp.integrator.differentiation_method = 'chainrule';

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
    dRa = p(2);
    Lf = p(5);
    Lr = p(6);
    q_slip = p(18);
    q_n = p(20);
    q_s = p(23);
    q_mu = p(21);
    q_Mtv = p(27);
    Ts = p(28);
    k = p(31);
    q_slack_vx = p(29);
    q_slack_track = p(30);
    
    % Progress rate
    sdot = ( z(10)*cos(z(9)) - z(11)*sin(z(9)) )/(1 - z(8)*k) * Ts; % == (vx*cos(mu) - vy*sin(mu))/(1 - n*k)

    % Slip difference
    beta_dyn = atan(z(11)/z(10));
    beta_kin = atan(z(6)*Lr/(Lr+Lf));
    diff_beta = beta_dyn - beta_kin;

    %Objective function
    f = -q_s*sdot + dRd*(z(3))^2 + dRa*(z(4))^2 + q_Mtv*(z(5))^2 + q_slip*(diff_beta)^2 + q_mu*(z(9))^2 + q_n*(z(8))^2 + q_slack_vx*z(1) + q_slack_track*z(2);
    
end


function xnext = integrated_dynamics(z, p)

    u = z(1:5);
    x = z(6:12);
    Ts = p(28);

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
    
    diff_delta = u(3);
    diff_Fm = u(4);
    Mtv = u(5);
    
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
    Cm = p(26);
    k = p(31);
    
    % Slip angles
    alpha_R = atan((vy-Lr*w)/(vx));
    alpha_F = atan((vy+Lf*w)/(vx)) - delta;
    
    % Simplified Pacejka magic formula
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));
    Fx = Cm*Fm*(1+cos(delta)) - m*u_r*g - 0.5*Cd*rho*Ar*vx^2;
    
    
    %Progress rate change
    sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k);
    
    % Differential equations (time dependent)
    xdot = [diff_delta;
            diff_Fm;
            vx*sin(mu) + vy*cos(mu);
            w - k*sdot;
            (1/m)*(Fx - Ff*sin(delta) + m*vy*w);
            (1/m)*(Fr + Cm*Fm*sin(delta) + Ff*cos(delta) - m*vx*w);
            (1/I)*((Ff*cos(delta) + Cm*Fm*sin(delta))*Lf - Fr*Lr + Mtv)];
    
end

function h = nonlin_const(z, p)

    Lf = p(5);
    Lr = p(6);
    Dr = p(7);
    Df = p(8);
    Cr = p(9);
    Cf = p(10);
    Br = p(11);
    Bf = p(12); 
    Cm = p(26);
    p_long = p(19);
    
    s1 = z(1);
    s2 = z(2);

    delta = z(6);
    Fm = z(7);
    n = z(8);
    mu = z(9);
    vx = z(10);
    vy = z(11);
    w = z(12);
    
    % Length and width of the car
    long = 2.72;
    width = 1.2 + 0.4;
    
    alpha_R = atan((vy-Lr*w)/(vx));
    alpha_F = atan((vy+Lf*w)/(vx)) - delta;
    
    Fr = Dr*sin(Cr*atan(Br*alpha_R));
    Ff = Df*sin(Cf*atan(Bf*alpha_F));

    h = [ n - long/2*sin(abs(mu)) + width/2*cos(mu) - s2; % <= L(s)
         -n + long/2*sin(abs(mu)) + width/2*cos(mu) - s2; % <= R(s)
         p_long*(Cm*Fm/Dr)^2 + (Fr/Dr)^2; % <= lambda
         p_long*(Cm*Fm/Df)^2 + (Ff/Df)^2; % <= lambda
         vx - s1]; 
     
end