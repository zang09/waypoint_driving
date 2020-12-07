function dot_X = mr_kinematics(X,t,u)

L = 0.6; %length between wheels


K = [0.5*cos(X(3)), 0.5*cos(X(3)); 0.5*sin(X(3)), 0.5*sin(X(3)); 1/L, -1/L];  %right left

dot_X = (K*u')';

end

