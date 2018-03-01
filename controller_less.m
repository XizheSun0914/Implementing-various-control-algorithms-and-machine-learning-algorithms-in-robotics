function controller_less(block)

setup(block);
  
function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 2;
  
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

  % Register the parameters. bHI, dof, tsampling, p0
  block.NumDialogPrms     = 4;

  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = 1;

  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);
  
  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 4;
  
  dof = block.DialogPrm(2).Data;
  
  block.Dwork(1).Name            = 'Fd_n_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Xd_n_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'E_n_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'Fe_n_1';
  block.Dwork(4).Dimensions      = dof;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  p0 = block.DialogPrm(4).Data;
  dof = block.DialogPrm(2).Data;
  
  block.Dwork(1).Data = zeros(dof,1);
  block.Dwork(2).Data = p0;
  block.Dwork(3).Data = 0;
  block.Dwork(4).Data = zeros(dof,1);
   

function Outputs(block)
  
  b = block.DialogPrm(1).Data;
  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  B = b/tsampling;
  
  Fe_n = block.InputPort(1).Data;
  Fe_n_1 = block.Dwork(4).Data;
  Fd_n_1 = block.Dwork(1).Data;
  Xd_n = block.InputPort(2).Data;
  Xd_n_1 = block.Dwork(2).Data;
  dX = Xd_n-Xd_n_1;
  E_n_1 = block.Dwork(3).Data;
  fe = norm(Fe_n);
  if norm(Fe_n_1)==0 && norm(Fe_n)~=0
      E_n = 0;
  else
  E_n = dX'*B*dX-Fd_n_1'*dX+E_n_1;
  end
  if fe~=0
    ud_n = Fe_n/fe;
    alpha = ud_n'*inv(B)*ud_n;
    fd_max = sqrt(4*E_n/alpha);
  else
    ud_n = zeros(dof,1);
    fd_max = 0;
  end
  
  fd_min = -fd_max;
  
  if fe > fd_max
      fd_n = fd_max;
  elseif fe < fd_min
      fd_n = fd_min;
  else
      fd_n = fe;
  end
  Fd_n = fd_n*ud_n;
  
  block.OutputPort(1).Data = Fd_n;
  block.OutputPort(2).Data = E_n;
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;  %% Fd(n-1)
  block.Dwork(2).Data = block.InputPort(2).Data;  %% Xd(n-1)
  block.Dwork(3).Data = block.OutputPort(2).Data;  %% E(n-1)
  block.Dwork(4).Data = block.InputPort(1).Data;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

