function controller_more(block)

setup(block);
  

function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 3;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  % Override the output port properties.
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';

  % Register the parameters.  damping, dof, tsampling, p0
  block.NumDialogPrms     = 4;
  
  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  block.SampleTimes = [tsampling 0];

  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = 1;
  block.OutputPort(3).Dimensions = 1;
  
  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'Fd_n_1';
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Xd_n_1';
  block.Dwork(2).Dimensions      = 3;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'Xi_n_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  p0=block.DialogPrm(4).Data;
  
  block.Dwork(1).Data = [0;0;0];
  block.Dwork(2).Data = p0; 
  block.Dwork(3).Data = 1;
   

function Outputs(block)
  
  b = block.DialogPrm(1).Data;
  tsampling = block.DialogPrm(3).Data;
  B = b/tsampling*eye(3);
  C = chol(B);
  
  Fd_n_1 = block.Dwork(1).Data;
  Xi_n_1 = block.Dwork(3).Data;
  Fe_n = block.InputPort(1).Data;
  Xd_n = block.InputPort(2).Data;
  Xd_n_1 = block.Dwork(2).Data;
  dX = Xd_n-Xd_n_1;
  
  Phi = 2*Xi_n_1*C'*dX-inv(C)*Fd_n_1;
  fe_n = norm(Fe_n);
  
  if fe_n~=0
      ud_n = Fe_n/norm(Fe_n);
      alpha = ud_n'*inv(B)*ud_n;
      Xi_n = (Xi_n_1*alpha*fe_n^2)/(Phi'*Phi);
      
  else
      ud_n = [0;0;1];
      alpha = ud_n'*inv(B)*ud_n;
      Xi_n = Xi_n_1;
  end
  if  Xi_n > 1
          Xi_n = 1;
  end
  
  fd_max = sqrt((Xi_n/(alpha*Xi_n_1))*(Phi'*Phi));
  fd_min = -fd_max;
  
  if fe_n > fd_max
      fd_n = fd_max;
  elseif fe_n < fd_min
      fd_n = fd_min;
  else
      fd_n = fe_n;
  end
  Fd_n = fd_n.*ud_n;
  
  block.OutputPort(1).Data = Fd_n;
  block.OutputPort(2).Data = Xi_n;
  block.OutputPort(3).Data = fd_max;
  
  
function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.InputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  
    
function Terminate(block)

