%% TESTING OPTIMAL PARAMETERS
clc, clear all, close all
global a_s % Markov parameter
load best_par;

%% Optimized Parameters
k1     = best_par(1)
k2     = best_par(2)
k3     = best_par(3)
rho1   = best_par(4)
rho2   = best_par(5)
rho3   = best_par(6)
gamma1 = best_par(7)
gamma2 = best_par(8)
gamma3 = best_par(9)

%% Simulation time
Ts = 0.01;                        % Sampling time
T  = 30;                          % Simulation period

%% Initial state dynamics (x,y,z of HR neuron)
Xm   = [3;-2;1];                  % Master system initial state
Xs   = [0;0;0];                   % Slave system initial state
Umax = 100;                        % Maximum control input to apply a state
e0   = Xm-Xs;                     % Initial error

%% Chaotic system parameters to use in error dynamics below
a = 1; b = 3; c = 1; d = 5; r = 0.006; s = 4; xr = -1.6; 
%% Finite time calculation for x state
s1_0 = k1*e0(1);
V0 = 0.5*s1_0^2;
eta1 = 0.5; ksi = 0.001;
delta11 = 2^((eta1+1)/2)*k2*ksi;
delta21 = 2*k1*gamma1;
eta_bar = (eta1+1)/2;
ts1 = 1/(delta21*(1-eta_bar))*log((delta11+delta21*V0)^(1-eta_bar)/delta11) 
n0  = floor(ts1/Ts);

%% Main Synchronisation loop
for n = 1:T/Ts
    %% Continuous time
    t(n) = n*Ts;
    
    %% Change of Markov Parameter
    if (0<n && n<=0.4*T/Ts)
        a_s = 1;
    elseif (0.4*T/Ts && n<0.7*T/Ts)
        a_s = 1.2;
    else
        a_s = 0.8;
    end
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
    
    %% Slave system integration (it has two inputs as I and U(3 dimensional) !!!
    Xs(:,n+1) = slave_integration(Xs(:,n),I(n),U(:,n),Ts);
    
end
%% Total MSE function
cost_value = 0.5*(e(1,:)*e(1,:)'+e(2,:)*e(2,:)'+e(3,:)*e(3,:)')*Ts/T

%% PLOTS
close all
Z = 2;
GA = 0.3;
KK = 0.1;
PP = 10;

figure;
plot(t,Xs(1,1:end-1),':r','linewidth',1.5*Z)
hold on
plot(t,Xm(1,1:end-1),'-b','linewidth',.4*Z)
box off
grid
%axis([0,10,-5,5])
f = gcf;
ax = gca;
ax.GridAlpha = GA;
h = legend('Slave','Master','location','northwest');
h.FontSize = PP;
set(h, 'Box', 'off')
set(gcf,'Color','White');
set(gca,'linewidth',KK)
set(gcf,'Color','White');
%title('(a)', 'FontSize', PP);
xlabel('Time [s]','fontsize',PP);
ylabel('x state synchronisation','fontsize',PP)
%exportgraphics(f,'xsim.eps','ContentType','vector','Resolution',300)

figure;
plot(t,Xs(2,1:end-1),':r','linewidth',1.5*Z)
hold on
plot(t,Xm(2,1:end-1),'-b','linewidth',.4*Z)
box off
grid
%axis([0,10,-5,5])
f = gcf;
ax = gca;
ax.GridAlpha = GA;
h = legend('Slave','Master','location','southeast');
h.FontSize = PP;
set(h, 'Box', 'off')
set(gcf,'Color','White');
set(gca,'linewidth',KK)
set(gcf,'Color','White');
%title('(a)', 'FontSize', PP);
xlabel('Time [s]','fontsize',PP);
ylabel('y state synchronisation','fontsize',PP)
%exportgraphics(f,'ysim.eps','ContentType','vector','Resolution',300)

figure;
plot(t,Xs(3,1:end-1),':r', 'linewidth',1.5*Z)
hold on
plot(t,Xm(3,1:end-1),'-b','linewidth',.4*Z)
box off
grid
%axis([0,10,-5,5])
f = gcf;
ax = gca;
ax.GridAlpha = GA;
h = legend('Slave','Master','location','southeast');
h.FontSize = PP;
set(h, 'Box', 'off')
set(gcf,'Color','White');
set(gca,'linewidth',KK)
set(gcf,'Color','White');
%title('(a)', 'FontSize', PP);
xlabel('Time [s]','fontsize',PP);
ylabel('z state synchronisation','fontsize',PP)
%exportgraphics(f,'zsim.eps','ContentType','vector','Resolution',300)

figure;
plot(t,U(1,:),'-k', 'linewidth',1.2*Z)
hold on
plot(t,U(2,:),'--g','linewidth',1.2*Z)
hold on
plot(t,U(3,:),':m','linewidth',1.2*Z)
box off
grid
%axis([0,10,-5,5])
f = gcf;
ax = gca;
ax.GridAlpha = GA;
h = legend('u_1','u_2','u_3');
h.FontSize = PP;
set(h, 'Box', 'off')
set(gcf,'Color','White');
set(gca,'linewidth',KK)
set(gcf,'Color','White');
%title('(a)', 'FontSize', PP);
xlabel('Time [s]','fontsize',PP);
ylabel('Control signals','fontsize',PP)
%exportgraphics(f,'usim.eps','ContentType','vector','Resolution',300)
