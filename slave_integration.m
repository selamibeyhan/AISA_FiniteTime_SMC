function X = slave_integration(X,I,U,Ts)
func = 'slave_dynamics';
F1 = Ts*feval(func,X,I,U);
F2 = Ts*feval(func,X+0.5*F1,I,U);
F3 = Ts*feval(func,X+0.5*F2,I,U);
F4 = Ts*feval(func,X+F3,I,U);
X = X + 1/6*F1+1/3*F2+1/3*F3+1/6*F4;
