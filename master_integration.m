function X = master_integration(X,U,Ts)
func = 'master_dynamics';
F1 = Ts*feval(func,X,U);
F2 = Ts*feval(func,X+0.5*F1,U);
F3 = Ts*feval(func,X+0.5*F2,U);
F4 = Ts*feval(func,X+F3,U);
X = X + 1/6*F1+1/3*F2+1/3*F3+1/6*F4;
