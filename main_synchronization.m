function cost_value = main_synchronization(Par)
global a_markov
%% Simulation time
Ts = 0.001;                       % Sampling time
T  = 10;                          % Simulation period
N = T/Ts;                         % Samples

%% Markov process to generate transition of modes
transition_probabilities = [0.3 0.7; 0.7 0.3];
starting_value = 1;
chain_length = T/2;
chain = zeros(1,chain_length);
chain(1) = starting_value;
mode = chain(1)*ones(N/chain_length,1)';
for i = 2:chain_length
    this_step_distribution = transition_probabilities(chain(i-1),:);
    cumulative_distribution = cumsum(this_step_distribution);
    r = rand();
    chain(i) = min(find(cumulative_distribution>r,1));
    mode = [mode chain(i)*ones(N/chain_length,1)'];
end
%% Optimized Parameters
k1 = Par(1); k2 = Par(2); k3 = Par(3);
rho1 = Par(4); rho2 = Par(5); rho3 = Par(6);
gamma1 = Par(7); gamma2 = Par(8); gamma3 = Par(9);

%% Initial state dynamics (x,y,z of HR neuron)
Xm   = [3;-2;1];  % Master system initial state
Xs   = [0;0;0];                     % Slave system initial state
Umax = 100;                         % Maximum control input to apply a state
e0   = Xm-Xs;                       % Initial error

%% Chaotic system parameters to use in error dynamics below
a_values = [1.2;0.8]; a = 1; b = 3; d = 5; r = 0.006; s = 4;

%% Main Synchronisation loop
for n = 1:T/Ts
    %% Continuous time
    t(n) = n*Ts;
    
    %% Change of Markov Parameter
    a_markov = a_values(mode(n));
       
    %% Synchronization error
    e(:,n) = Xs(:,n)-Xm(:,n);
    
    %% Sliding surface
    s1(n) = k1*(e(1,n)-e0(1)*exp(-rho1*t(n)));
    s2(n) = k2*(e(2,n)-e0(2)*exp(-rho2*t(n)));
    s3(n) = k3*(e(3,n)-e0(3)*exp(-rho3*t(n)));
    
    %% Control inputs
    e(:,n) = Xs(:,n)-Xm(:,n); % x synchronization error
    U(1,n) = -e(2,n)+a*e(1,n)^3-b*e(1,n)^2+e(3,n)-rho1*e0(1)*exp(-rho1*t(n))-gamma1*s1(n);
    U(2,n) = d*e(1,n)^2+e(2,n)-rho2*e0(2)*exp(-rho2*t(n))-gamma2*s2(n);
    U(3,n) = -r*s*e(1,n)+r*e(3,n)-rho3*e0(3)*exp(-rho3*t(n))-gamma3*s3(n);
    
    %% Limitation to input values
    if abs(U(1,n))>Umax; U(1,n) = sign(U(1,n))*Umax; end  % x control input saturation
    if abs(U(2,n))>Umax; U(2,n) = sign(U(2,n))*Umax; end  % y control input saturation
    if abs(U(3,n))>Umax; U(3,n) = sign(U(3,n))*Umax; end  % z control input saturation
    
    %% Master system integration
    I(n) = 3.2;
    Xm(:,n+1) = master_integration(Xm(:,n),I(n),Ts);
    
    %% Slave system integration same I but it has also input vector
    Xs(:,n+1) = slave_integration(Xs(:,n),I(n),U(:,n),Ts);
    
end
%% Total MSE function
cost_value = 0.5*(e(1,:)*e(1,:)'+e(2,:)*e(2,:)'+e(3,:)*e(3,:)')*Ts/T;
end
