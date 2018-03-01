function LessConservative(block)
 
setup(block);
  

function setup(block)

  block.NumInputPorts  = 2; %Pos, Fe
  block.NumOutputPorts = 2; %Fd,Energy
  
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';

  %% bDevice, tsampling, x0
  block.NumDialogPrms     = 3;

  tsampling = block.DialogPrm(2).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

  
function DoPostPropSetup(block)
  block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'pos_1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Fd_1';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'Energy_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  x0 = block.DialogPrm(3).Data;

  block.Dwork(1).Data = x0;
  block.Dwork(2).Data = 0; 
  block.Dwork(3).Data = 0; 
   

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  Fe = block.InputPort(2).Data;
  
  pos_1 = block.Dwork(1).Data;
  Fd_1 = block.Dwork(2).Data;
  Energy_1 = block.Dwork(3).Data;
  
  bDevice = block.DialogPrm(1).Data;
  tsampling = block.DialogPrm(2).Data;
  B = bDevice/tsampling;
  dpos = pos-pos_1;
  Energy = Energy_1 + dpos'*B*dpos - Fd_1'*dpos;
  if norm(Fe)~=0
      u = Fe/norm(Fe);
      alpha = u'*inv(B)*u;
      fdmax = sqrt(4*Energy/alpha);
  else
      u = 0;
      alpha = 0;
      fdmax = 0;
  end
  fdmin = -fdmax;
  
  if norm(Fe)>fdmax
      fd = fdmax;
  elseif norm(Fe)<fdmin
      fd = fdmin;
  else
      fd = norm(Fe);
  end
  Fd = fd*u;
  

  block.OutputPort(1).Data = Fd;
  block.OutputPort(2).Data = Energy;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);