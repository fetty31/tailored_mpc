#include "optimizer.hh"

// Constructor
Optimizer::Optimizer(const Params& params){

    // NLOP params
    this->n_states = params.mpc.nlop.n_states;
    this->n_controls = params.mpc.nlop.n_controls;
    this->N = params.mpc.nlop.N;
    this->Npar = params.mpc.nlop.Npar + this->n_states + this->N; // [ 23 (MPC parameters) + (initial state) + n (curvature points == N) ]

    // MPC freq
    this->T = params.mpc.T;

    // Vehicle params
    this->m = params.vehicle.m;
    this->longue = params.vehicle.longue;
    this->width = params.vehicle.width;
    this->Lr = params.vehicle.Lr;
    this->Lf = params.vehicle.Lf;
    this->ax_max = params.vehicle.ax_max;
    this->ay_max = fabs(params.vehicle.ax_min); // (here we consider |ay_max| == |ax_min| )

    // NLOP variables
    X = SX::sym("X", this->n_states * (this->N+1));
    U = SX::sym("U", this->n_controls * this->N);
    P = SX::sym("P", this->Npar);

    this->params_set = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Principal functions--------------------------------------------------

shared_ptr<Function> init(){

    if(params_set){
        
        SXDict nlp = {{"x", SX::vertcat({X,U})},
            {"f", obj},
            {"g", SX::vertcat(g)},
            {"p", P}
            };

        solver = nlpsol("MPCsolver", "ipopt", nlp, solverOptions);

        return shared_ptr<Function>(solver);

    }else{

        ROS_ERROR("Optimizer constructor must be called. Returning null ptr");
        return shared_ptr<Function>(nullptr);
    }

}

void Optimizer::nlop_formulation(){

    SX dRd = P(0);
    SX dRa = P(1);
    SX Lf = P(4);
    SX Lr = P(5);
    SX q_slip = P(17);
    SX q_n = P(19);
    SX q_s = P(22);
    SX q_mu = P(20);

    // Add initial conditions
    g.push_back(X(:,0) - P(23:30))
    cout << "P(23:30) " << P(23:30) << endl;

    int idx = 0; // index for X variables (states) 
    int idu = 0; // index for U variables (controls) 
    for(int e = 1; e < N+1; e++){
        SX st = X(Slice(idx,idx+n_states));
        SX k = P(23+e);
        SX steering = st(0) + U(idu);

        // Progress rate
        SX sdot = (st(4)*cos(st(3)) - st(5)*sin(st(3)))/(1-st(2)*k);

        // Slip difference
        SX beta_dyn = atan(st(5)/st(4));
        SX beta_kin = atan(steering*Lr/(Lr+Lf));
        SX diff_beta = beta_dyn - beta_kin;

        // Objective function
        obj += -q_s*sdot + dRd*pow(U(idu),2) + dRa*pow(U(idu+1),2) + q_slip*pow(diff_beta,2) + q_mu*pow(st(3),2) + q_n*pow(st(2),2);

        SX st_next = X(Slice(idx+n_states,idx+2*n_states)); // next state
        SX con_now = U(Slice(idu,idu+n_controls)) + st(Slice(0,2)); // current stage global controls (steering,throttle)
        SX st_now = st(Slice(2,st.rows())); // current stage states (n,mu,vx,vy,w)

        vector<SX> f_value = continuous_dynamics(st_now,con_now,P,k); // ODE with next states prediction

        SX states = SX::sym("states", f_value.size());
        for(int row = 0; row < f_value.size(); row++){
            states(row) = st_now(row) + T*f_value.at(row); // euler
        }

        SX st_next_euler = SX::vertcat({con_now,states}); //next state calculated 

        // Add continuity constraints
        g.push_back( st_next - st_next_euler );

        // Next indexes 
        idx+=this->n_states;
        idu+=this->n_controls;
    }

}

// Set constraints --> track constraints
void Optimizer::track_constraints(){

    for(int i=0;i<N+1;i++){

    SX n = X(2+i*7);
    SX mu = X(3+i*7);

    g.push_back(n+longue/2*sin(abs(mu))+width/2*cos(mu)); // track constraints
    g.push_back(-n+longue/2*sin(abs(mu))+width/2*cos(mu));

  }
}

// Set constraints --> forces ellipse constraint
void Optimizer::forces_constraints(){

    for(int i=0;i<N+1;i++){

    SX alpha_R = atan((X(5+i*7)-P(5)*X(6+i*7))/X(4+i*7));               // atan((vy-Lr*w)/(vx))
    SX alpha_F = atan((X(5+i*7)+P(4)*X(6+i*7))/X(4+i*7)) - X(0+i*7);    // atan((vy+Lf*w)/(vx)) - delta

    SX Fr = P(6)*sin(P(8)*atan(P(10)*alpha_R));                                                            // Fr = Dr*sin(Cr*atan(Br*alpha_R))
    SX Ff = P(7)*sin(P(9)*atan(P(11)*alpha_F));                                                            // Ff = Df*sin(Cf*atan(Bf*alpha_F))
    SX Fx = P(2)*X(1+i*7) - P(2)*P(12)*P(13) - 0.5*P(14)*P(15)*P(16)*pow(X(4+i*7),2);  // Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2

    // g.push_back( pow(P(18)*Fx,2) + pow(Fr,2) - pow(P(21]*P(6),2)/P(2) );   // (p_long*Fx)² + Fr² <= (lambda*Dr)²/m
    // g.push_back( pow(P(18)*Fx,2) + pow(Ff,2) - pow(P(21]*P(7),2)/P(2) );   // (p_long*Fx)² + Ff² <= (lambda*Df)²/m

    g.push_back( pow(Fx/(this->ax_max*this->m),2) + pow(Fr/(this->ay_max*this->m),2) ); // (Fx/(Ax_max*m))² + (Fr/(Ay_max*m))² <= lambda
    g.push_back( pow(Fx/(this->ax_max*this->m),2) + pow(Ff/(this->ay_max*this->m),2) ); // (Fx/(Ax_max*m))² + (Ff/(Ay_max*m))² <= lambda

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------Auxiliar functions--------------------------------------------------


vector<SX> Optimizer::continuous_dynamics( SX st, SX con, SX P, SX k){

    vector<SX> f_values;

    SX alpha_R = atan((st(3)-P(5)*st(4))/st(2));                                  // alphaR = atan((vy-Lr*w)/(vx))
    SX alpha_F = atan((st(3)+P(4)*st(4))/st(2)) - con(0);                         // alphaL = atan((vy+Lf*w)/(vx)) - delta

    SX Fr = P(6)*sin(P(8)*atan(P(10)*alpha_R));                                   // Fr = Dr*sin(Cr*atan(Br*alpha_R))
    SX Ff = P(7)*sin(P(9)*atan(P(11)*alpha_F));                                   // Ff = Df*sin(Cf*atan(Bf*alpha_F))
    SX Fx = P(2)*con(1) - P(2)*P(12)*P(13) - 0.5*P(14)*P(15)*P(16)*pow(st(2),2);  // Fx = m*a - m*u_r*g - 0.5*Cd*rho*Ar*vx^2

    SX sdot = (st(2)*cos(st(1)) - st(3)*sin(st(1)))/(1-st(0)*k);                  // sdot = (vx*cos(mu) - vy*sin(mu))/(1 - n*k)

    SX ndot = (st(2)*sin(st(1)) + st(3)*cos(st(1)))/sdot;                         // ndot   =  (vx*sin(mu) + vy*cos(mu))/sdot
    SX mudot = st(4)/sdot - k;                                                    // mudot  =  w/sdot - k
    SX vxdot = 1/P(2)*(Fx - Ff*sin(con(0)) + P(2)*st(3)*st(4))/sdot;              // vxdot  =  (1/m)*(Fx - Ff*sin(delta) + m*vy*w)/sdot
    SX vydot = 1/P(2)*(Fr + Ff*cos(con(0)) - P(2)*st(2)*st(4))/sdot;              // vydot  =  (1/m)*(Fr + Ff*cos(delta) - m*vx*w)/sdot
    SX wdot = 1/P(3)*(Ff*P(4)*cos(con(0)) - Fr*P(5))/sdot;                        // wdot   =  (1/I)*(Ff*Lf*cos(delta) - Fr*Lr)/sdot

    f_values.push_back(ndot);
    f_values.push_back(mudot);
    f_values.push_back(vxdot);
    f_values.push_back(vydot);
    f_values.push_back(wdot);

    return f_values;

}

vector<double> Optimizer::vconcat(const vector<double>& x, const vector<double>& y){
    vector<double> v(x.size() + y.size(),0.0);
	move(x.begin(), x.end(), v.begin());
	move(y.begin(), y.end(), v.begin() + x.size());
    return v;
}


void Optimizer::printVec(vector<double> &input, int firstElements=0){
    if(firstElements!=0){
      for (auto it = input.begin(); it != input.end()-input.size()+firstElements; it++) {
        std::cout << *it << "\n";
      }
    }else{
        for (auto it = input.begin(); it != input.end(); it++) {
        std::cout << *it << "\n";
      }
    }   
}