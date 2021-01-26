function y_n=rk4(fn,y,u,ts,j)
% RK4  Runge-Kutta differential equation solution method
%
%  []=rk4(fn,y,u,ts,i)
%  where, fn is function name recalled
%  X_con(i+1,:) = rk4('Dynamics_IT_NLC_Control',X_con(i,:), u(i),ts,i);  % integral

%  Se-Min Kim
%  The University of Yonsei, Control Lab.
%  Copyright 1999. 1.14 


t0=0;
t=t0+(j-1)*ts;
h=ts;

k1=h*feval(fn,y,t,u);
k2=h*feval(fn,y+k1/2,t+h/2,u);
k3=h*feval(fn,y+k2/2,t+h/2,u);
k4=h*feval(fn,y+k3,t+h,u);

y_n=y+(k1+2*k2+2*k3+k4)/6;