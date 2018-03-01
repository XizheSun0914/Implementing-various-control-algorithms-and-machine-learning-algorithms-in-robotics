function HapticDevice(block)
%% use Runge-Kutta method to solve the ordinary differential euqation for the haptic device
setup(block);
  

function setup(block)

  % 2 inputports are Fh and Fd; 5 outputports are Position X, Velocity,
  % Acceleration, count number(count_n) for solve ODE step by step,
  % t(count_n) express the first three outputs(X, V, A) at corresponding time
  
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 5;
  
  block.InputPort(1).Dimensions=1;
  block.InputPort(2).Dimensions=1;
  block.InputPort(2).DirectFeedthrough=false;
  block.OutputPort(1).Dimensions=1;
  block.OutputPort(2).Dimensions=1;
  block.OutputPort(3).Dimensions=1;
  block.OutputPort(4).Dimensions=1;
  block.OutputPort(5).Dimensions=1;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;
  block.InputPort(2).Complexity  = 'Real';
  
  % Override the output port properties.
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';
  block.OutputPort(4).DatatypeID  = 0; % double
  block.OutputPort(4).Complexity  = 'Real';
  block.OutputPort(5).DatatypeID  = 0; % double
  block.OutputPort(5).Complexity  = 'Real';
  
  % Register the parameters. Ffriction, Bh, Mh, tinterval
  block.NumDialogPrms     = 6;
  tinterval = block.DialogPrm(4).Data;

  block.SampleTimes = [tinterval 0];
 
  block.InputPort(1).SamplingMode = tinterval;
  block.InputPort(2).SamplingMode = tinterval;
  block.OutputPort(1).SamplingMode = tinterval;
  block.OutputPort(2).SamplingMode = tinterval;
  block.OutputPort(3).SamplingMode = tinterval;
  block.OutputPort(4).SamplingMode = tinterval;
  block.OutputPort(5).SamplingMode = tinterval;
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

%% declare four discrete state variables. position X, Velocity, count number, corresponding time     
function DoPostPropSetup(block)
  block.NumDworks = 4;
  
  block.Dwork(1).Name            = 'x';    %position at time n-1
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'v';    % velocity at time n-1
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'count';    % counter 
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'tcount';    % time
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%% initiate attributes of the system
function Start(block)

  x0=block.DialogPrm(5).Data;
  v0=block.DialogPrm(6).Data;

  block.Dwork(1).Data = x0;  %% initial position x0
  block.Dwork(2).Data = v0;  %% initial velocity v0
  block.Dwork(3).Data = 1;  %% initial count number
  block.Dwork(4).Data = 0;  %% initial time
 

function Outputs(block)

%% Runge_Kutta mehtod step by step

  Ffriction = block.DialogPrm(1).Data;
  Bh = block.DialogPrm(2).Data;
  Mh = block.DialogPrm(3).Data;
  tinterval = block.DialogPrm(4).Data;
  
  Fh = block.InputPort(1).Data;
  Fd = block.InputPort(2).Data;
  
  count_n_1 = block.Dwork(3).Data;
  count_n = count_n_1+1;
  f=@(t,x)[x(2);(Fh+Fd-Ffriction)/Mh-Bh/Mh*x(2)];
  t(count_n_1) = block.Dwork(4).Data;
  t(count_n) = t(count_n_1) + tinterval;
  x(1,count_n_1)=block.Dwork(1).Data;
  x(2,count_n_1)=block.Dwork(2).Data;
  
  K1 = f(t(count_n_1),x(:,count_n_1));
  K2 = f(t(count_n_1)+tinterval/2,x(:,count_n_1)+tinterval*K1/2);
  K3 = f(t(count_n_1)+tinterval/2,x(:,count_n_1)+tinterval*K2/2);
  K4 = f(t(count_n_1)+tinterval,x(:,count_n_1)+tinterval*K3);
  x(:,count_n)=x(:,count_n_1)+tinterval*(K1+2*K2+2*K3+K4)/6;
  a = (Fh+Fd-Ffriction)/Mh-Bh/Mh*x(2,count_n);
  
  block.OutputPort(1).Data = x(1,count_n);  %% position 
  block.OutputPort(2).Data = x(2,count_n);  %% velocity
  block.OutputPort(3).Data = a;             %% acceleration
  block.OutputPort(4).Data = count_n;
  block.OutputPort(5).Data = t(count_n);
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(4).Data;
  block.Dwork(4).Data = block.OutputPort(5).Data;


    
function Terminate(block)
