function [sys,x0,str,ts,simStateCompliance] = virtualenvironment(t,x,u,flag,kBall,bBall,rBall,cBall,mu)

switch flag,
  
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 3
    sys=mdlOutputs(t,x,u,kBall,bBall,rBall,cBall,mu);

  case { 1, 2, 4, 9 }
    sys=[];

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;  % dynamically sized
sizes.NumInputs      = 6;  % dynamically sized
sizes.DirFeedthrough = 1;  % has direct feedthrough
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   % sample time

% specify that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';


function sys = mdlOutputs(t,x,u,kBall,bBall,rBall,cBall,mu)
KBall=[kBall 0 0;0 kBall 0;0 0 kBall];
BBall=[bBall 0 0;0 bBall 0;0 0 bBall];
x=[u(4);u(5);u(6)];
vel=[u(1);u(2);u(3)];
cx=[cBall'; x'];
dist=pdist(cx);
nBall=[x-cBall]/dist;

vp = vel-vel'*nBall*nBall;

if norm(vp)~=0
    fBall = -vp /norm(vp);
else
    fBall = zeros(3,1);
end

if dist<rBall
    fn=(rBall-dist)*KBall*nBall-BBall*(vel'*nBall)*nBall;
    if fn'*nBall<0
        fn=[0;0;0];
    end
else
    fn=[0;0;0];
end

ff = mu*norm(fn)*fBall;
f_e = fn+ff;

sys = [f_e(1);f_e(2);f_e(3)];
