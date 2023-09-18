#include "optimizer.hh"

// Constructor
Optimizer::Optimizer(const Params& params){

    // NLOP params
    this->n_states = params.mpc.nlop.n_states;
    this->n_controls = params.mpc.nlop.n_controls;
    this->N = params.mpc.nlop.N;
    this->Npar = params.mpc.nlop.Npar + this->n_states 
                    + this->N + this->N; /* [ 23 (MPC parameters) + (initial state) + 
                                               + k (curvature points == N) + vx (target velocity points == N) ]
                                        */

    this->T = params.mpc.rk4_t; // Integration time

    // Vehicle params
    this->m = params.vehicle.m;
    this->longue = params.vehicle.longue;
    this->width = params.vehicle.width;
    this->Lr = params.vehicle.Lr;
    this->Lf = params.vehicle.Lf;

    // NLOP variables
    X = SX::sym("X", this->n_states * (this->N+1));
    U = SX::sym("U", this->n_controls * this->N);
    P = SX::sym("P", this->Npar);

    this->params_set = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

shared_ptr<Function> Optimizer::generate_solver(){

    if(params_set){

        auto start_time = std::chrono::system_clock::now();

        nlop_formulation();
        track_constraints();
        forces_constraints();

        SXDict nlp = {{"x", SX::vertcat({X,U})},
            {"f", obj},
            {"g", SX::vertcat(g)},
            {"p", P}
            };

        // Create solver object
        Function solver = nlpsol("MPCsolver", "ipopt", nlp, solverOptions);

        // Make solver obj a shared pointer
        auto solver_ptr = make_shared<Function>(move(solver));

        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        ROS_WARN( "OPTIMIZER ELAPSED TIME: %f ms", elapsed.count()*1000);

        return solver_ptr;

    }else{

        ROS_ERROR("Optimizer constructor must be called. Returning null ptr");
        return shared_ptr<Function>(nullptr);
    }

}

void Optimizer::nlop_formulation(){

    /*TO DO:
        - change parameters indexes    
    */

    SX dRd = P(0);
    // SX dRa = P(1);
    SX Lf = P(4);
    SX Lr = P(5);
    SX q_slip = P(17);
    SX q_n = P(19);
    SX q_s = P(22);
    SX q_mu = P(20);

    // Add initial conditions
    int idx_init = Npar-n_states-N;
    g.push_back( X(Slice(0,n_states)) - P(Slice(idx_init,idx_init+n_states)) );

    cout << "P(23:28) " << P(Slice(idx_init,idx_init+n_states)) << endl;
    cout << "X(0:5) " << X(Slice(0,n_states)) << endl;

    int idx = 0; // index for X variables (states) 
    int idu = 0; // index for U variables (controls) 
    for(int e = 0; e < N; e++){
        SX st = X(Slice(idx,idx+n_states));
        SX k = P(idx_init+e);
        SX vx = P(idx_init+N+e);
        SX steering = st(0) + U(idu);

        // Progress rate
        SX sdot = (vx*cos(st(2)) - st(3)*sin(st(2)))/(1-st(1)*k); // == (vx*cos(mu) - vy*sin(mu))/(1 - n*k)

        // Slip difference
        SX beta_dyn = atan(st(3)/vx);
        SX beta_kin = atan(steering*Lr/(Lr+Lf));
        SX diff_beta = beta_dyn - beta_kin;

        // Objective function
        obj += -q_s*sdot + dRd*pow(U(idu),2) /*+ dRa*pow(U(idu+1),2)*/ + q_slip*pow(diff_beta,2) + q_mu*pow(st(2),2) + q_n*pow(st(1),2);

        SX st_next = X(Slice(idx+n_states,idx+2*n_states)); // next stage state vector
        SX st_now = st(Slice(1,st.rows())); // current stage states (n,mu,vy,w)

        vector<SX> f_value = continuous_dynamics(st_now,steering,U(idu+1),k,vx); // ODE with next states prediction

        SX states = SX::sym("states", f_value.size());
        for(int row = 0; row < f_value.size(); row++){
            states(row) = st_now(row) + T*f_value.at(row); // euler
        }
        
        SX con_now = SX::vertcat({steering,U(idu+1)}); // current control vector (diffDelta, Mtv)
        SX st_next_euler = SX::vertcat({con_now,states}); //next state calculated 

        // Add continuity constraints
        g.push_back( st_next - st_next_euler );

        // Next indexes 
        idx+=this->n_states;
        idu+=this->n_controls;
    }

}

// Set track constraints
void Optimizer::track_constraints(){

    for(int i=0;i<N+1;i++){

        SX n = X(1+i*n_states);
        SX mu = X(2+i*n_states);

        g.push_back(n + longue/2*sin(abs(mu)) + width/2*cos(mu)); // track constraints
        g.push_back(-n + longue/2*sin(abs(mu)) + width/2*cos(mu));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------


vector<SX> Optimizer::continuous_dynamics( SX st, SX delta, SX Mtv, SX k, SX vx){

    vector<SX> f_values;

    SX alpha_R = atan((st(2)-P(5)*st(3))/vx);                   // alphaR = atan((vy-Lr*w)/(vx))
    SX alpha_F = atan((st(2)+P(4)*st(3))/vx) - delta;           // alphaL = atan((vy+Lf*w)/(vx)) - delta

    SX Fr = P(6)*sin(P(8)*atan(P(10)*alpha_R));                 // Fr = Dr*sin(Cr*atan(Br*alpha_R))
    SX Ff = P(7)*sin(P(9)*atan(P(11)*alpha_F));                 // Ff = Df*sin(Cf*atan(Bf*alpha_F))

    SX sdot = (vx*cos(st(1)) - st(2)*sin(st(1)))/(1-st(0)*k);   // sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k)

    SX ndot = (vx*sin(st(1)) + st(2)*cos(st(1)));               // ndot   =  (vx*sin(mu) + vy*cos(mu))
    SX mudot = st(3) - k*sdot;                                  // mudot  =  w - k*sdot
    SX vydot = 1/P(2)*(Fr + Ff*cos(delta) - P(2)*vx*st(3));     // vydot  =  (1/m)*(Fr + Ff*cos(delta) - m*vx*w)
    SX wdot = 1/P(3)*(Ff*P(4)*cos(delta) - Fr*P(5) + Mtv);      // wdot   =  (1/I)*(Ff*Lf*cos(delta) - Fr*Lr + Mtv)

    f_values.push_back(ndot);
    f_values.push_back(mudot);
    f_values.push_back(vydot);
    f_values.push_back(wdot);

    return f_values;

}