function df = slave_dynamics(X,I,U)
global a_s
b = 3; c = 1; d = 5; r = 0.006; s = 4; xr = -1.6; 
df = [X(2)-a_s*X(1)^3+b*X(1)^2-X(3)+I+U(1);
        c*X(3)^2-d*X(1)^2-X(2)+U(2);
        r*(s*(X(1)-xr)-X(3))+U(3)];

