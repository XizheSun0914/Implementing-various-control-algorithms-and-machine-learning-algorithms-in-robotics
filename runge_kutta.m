function [t,x] = runge_kutta (f,x0,h,a,b)  
%% using runge kutta method to solve ordinary differential equations 
%%(functions, initial values, time step, initial time, end time)
n=floor((b-a)./h);  % number of steps
t(1)=a;
x(:,1)=x0; % initialition
for i = 1:n
    t(i+1)=t(i)+h;
    K1=f(t(i),x(:,i));
    K2=f(t(i)+h/2,x(:,i)+h*K1/2);
    K3=f(t(i)+h/2,x(:,i)+h*K2/2);
    K4=f(t(i)+h,x(:,i)+h*K3);
    x(:,i+1)=x(:,i)+h*(K1+2*K2+2*K3+K4)/6;
end
