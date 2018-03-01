function haptic_device(block)
%% haptic device
setup(block);
  

function setup(block)

 
  block.NumInputPorts  = 1;  %% force
  block.NumOutputPorts = 1;  %% velocity

  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';

  %% m, bHI, v0, dof
  block.NumDialogPrms     = 4;
  
  dof = block.DialogPrm(4).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  
  block.NumContStates = 1;

  block.SampleTimes = [0 0];

  block.SimStateCompliance = 'DefaultSimState';
  
 
  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Derivatives', @Derivatives);
  
  block.RegBlockMethod('Terminate', @Terminate);


function Start(block)

block.ContStates.Data(1) = block.DialogPrm(3).Data;
   

function Outputs(block)
  
  block.OutputPort(1).Data = block.ContStates.Data(1);


function Derivatives(block)

m = block.DialogPrm(1).Data;
bHI = block.DialogPrm(2).Data;
v = block.OutputPort(1).Data;
f = block.InputPort(1).Data;
a = (f-bHI*v)/m;

block.Derivatives.Data = a;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);
