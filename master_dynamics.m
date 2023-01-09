function df = master_dynamics(X,I)
a = 1; b = 3; c = 1; d = 5; 
r = 0.006; s = 4; xr = -1.6;  
df = [X(2)-a*X(1)^3+b*X(1)^2-X(3)+I;
        c*X(3)^2-d*X(1)^2-X(2);
        r*(s*(X(1)-xr)-X(3))];

