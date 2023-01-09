%% AISA BASED OPTIMIZATION of FINITE-TIME SMC of CHAOTIC SYSTEMS
% January-2023
% Written by Prof. Selami Beyhan
% You can send your feedbacks to selami.beyhan@gmail.com
% A note: There is no warranty that optimization code find optimal parameters always.
% It may provide different optimal results at each run.
% Please cite for AISA optimization:
% @article{bogar2020adolescent,
%   title={Adolescent Identity Search Algorithm (AISA): A novel metaheuristic approach
%   for solving optimization problems},
%   author={Bogar, Esref and Beyhan, Selami},
%   journal={Applied Soft Computing},
%   volume={95},
%   pages={106503},
%   year={2020},
%   publisher={Elsevier}}
%% Initial Parameters of Optimization
clc; clear; close all; 
rng('shuffle')  
nPop    = 30;                         % Population size/increase for better results
maxiter = 100;                         % Iteration number or termination criterion
D = 9;                                % D = dimension of problem or the number of parameters
ub = [200 200 200 10 10 10 100 100 100]; % ub: upper bound, change according to problem
lb = 0.0*ones(1,D);                   % lb: lower bound, change according to problem
for i=1:nPop
    Candidates(i,:) = rand(1,D).*(ub-lb)+lb;          % Random initialization
    canCost(i,:)    = Cost_Function(Candidates(i,:)); % calculating fitness values of each candidate
end
%% Main Loop of AISA
tic
flag = 0; 
for i = 2:maxiter
    if flag==0
        [Candidates_LSE] = FeatureExtraction(Candidates,canCost,lb,ub);
    end
    [val,index] = min(canCost); 
    flag=1;
    for j=1:nPop
        r1=rand;
        if r1<=1/3 
            x(j,:) = Candidates(j,:)-(rand).*(Candidates(j,:)-Candidates_LSE); % Eq. 13
        elseif r1<=2/3 && r1>1/3 
            gg = randi(nPop);
            if gg==index
                gg = randi(nPop);
            end
            x(j,:) = Candidates(j,:)-(rand).*(Candidates(gg,:)-Candidates(index,:)); % Eq. 14
        else 
            array = [Candidates(randi(nPop),randi(D))*ones(1,D)];
            x(j,:) = Candidates(j,:)+rand(1,D).*(array(randi(1),:)-Candidates(j,:)); % Eq. 15
        end
        for k=1:D
            if x(j,k)>ub(k) || x(j,k)<lb(k)
                x(j,k) = lb(k) + rand*(ub(k)-lb(k));
            end
        end 
        xcanCost = Cost_Function(x(j,:));
        if xcanCost<canCost(j,:)
            flag = 0;
            Candidates(j,:) = x(j,:);
            canCost(j) = xcanCost;
        end
    end
    [val,index] = min(canCost);
    iter = i
    Cost = val
end
toc
%% Save optimal parameters
[val,index] = min(canCost);
best_par = Candidates(index,:);
save best_par



