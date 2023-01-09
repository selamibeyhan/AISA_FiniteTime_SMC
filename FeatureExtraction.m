function [map] = FeatureExtraction(x,yd,lb,ub)
xbase = x;

%% Chebyshev Polynomial NN Based Feature Selection
% Normalization to [-1,1]
for i = 1:length(x(1,:))
    x(:,i) = 2*(x(:,i)-lb)/(ub-lb)-1;
end
% Regressor matrix
phi = [];
for i = 1:length(x(1,:))
    psi = [];
    phi = [phi,x(:,i),2*x(:,i).^2-1,4*x(:,i).^3-3*x(:,i)];
    psi = [psi,x(:,i),2*x(:,i).^2-1,4*x(:,i).^3-3*x(:,i)];
    phivec{i} = psi;
end

%% RLSE Algorithm for optimal parameters
lamda = 0.9;		
delta = 1e10;	
sysorder = size(x,2)*3;
P = delta*eye(sysorder);
Q = zeros(sysorder,1);
for n = 1:size(x,1) 
	u = phi(n,:)';
	k = P*u/(lamda+u'*P*u);
    yhat = Q'*u;
	Q = Q + k*(yd(n)-yhat);
	P = (P - k.*P*u)/lamda;
end 

%% Selection of best individuals
[mat,padded] = vec2mat(Q,3);
map = [];
for i = 1:length(x(1,:))
    y_hatnew(i,:) = phivec{i}*mat(i,:)';
    [cost,index]  = sort(y_hatnew(i,:));
    bestindex(i)  = index(1);
    map = [map,xbase(bestindex(i),i)];
end

