function controller(block)

setup(block);
  
function setup(block)

  block.NumInputPorts  = 3;  %% x, fWall, position
  block.NumOutputPorts = 4;  %% u, fCon, mu_max, stiffness

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(3).DatatypeID  = 0;  % double
  block.InputPort(3).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';
  block.OutputPort(4).DatatypeID  = 0; % double
  block.OutputPort(4).Complexity  = 'Real';

  %% tau, dof, tsampling, p0, xWall, nWall
  block.NumDialogPrms     = 6;
  
  tsampling = block.DialogPrm(3).Data;
  dof = block.DialogPrm(2).Data;
  
  block.SampleTimes = [tsampling 0];

  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.InputPort(3).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = dof;
  block.OutputPort(3).Dimensions = 1;
  block.OutputPort(4).Dimensions = 1;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.InputPort(3).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  block.OutputPort(4).SamplingMode = tsampling;

  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)

  dof = block.DialogPrm(2).Data;

  block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'x_n_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'f_n_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'mu_max';
  block.Dwork(3).Dimensions      = dof;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  p0 = block.DialogPrm(4).Data;  

  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = 0; 
  block.Dwork(3).Data = 0;
  

function Outputs(block)
  
  tau = block.DialogPrm(1).Data;
  tsampling = block.DialogPrm(3).Data;
  xWall = block.DialogPrm(4).Data;
  nWall = block.DialogPrm(5).Data;
  
  x_n = block.InputPort(1).Data;
  fWall_n = block.InputPort(2).Data;
  pos = block.InputPort(3).Data;
  
  x_n_1 = block.Dwork(1).Data;
  fWall_n_1 = block.Dwork(2).Data;
  mu_max = block.Dwork(3).Data;
  
  fWall_dot = -(fWall_n-fWall_n_1)/tsampling;
  if (x_n-x_n_1)~=0
      mu_n = -1*(fWall_n-fWall_n_1)/(x_n-x_n_1);
  else
      mu_n = mu_max;
  end
  if mu_n > mu_max
      mu_max = mu_n;
  end
  if mu_max~=0
      u_n = fWall_dot*1/mu_max;
  else
      u_n = 0;
  end
  fCon = fWall_n - fWall_dot*tau;   
  
  if (pos-xWall)*nWall<0
      stiff = fCon/abs(pos-xWall);
  else
      stiff = 0;
  end
  
  block.OutputPort(1).Data = u_n;
  block.OutputPort(2).Data = fCon;
  block.OutputPort(3).Data = mu_max;
  block.OutputPort(4).Data = stiff;


function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.InputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(3).Data;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

